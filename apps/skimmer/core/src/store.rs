// SPDX-License-Identifier: GPL-3.0-only
//! Every decode in a SQLite file, for statistics instead of an ever-growing
//! ALL.TXT.
//!
//! WAL, so the GUI reads while the skimmer writes; `synchronous=NORMAL`,
//! which in WAL loses at most the last transactions on power loss and never
//! corrupts the file; rows go in batches of one transaction. Decodes are
//! kept as they came (band, sender, locator split out so queries are indexed
//! lookups), plus a `stations` table of the last locator each call sent, so a
//! later message without a locator still has a bearing.

use std::path::Path;
use std::time::{Duration, Instant};

use rusqlite::{Connection, OpenFlags, params};

use crate::{Decode, geo, modes, spot};

const TABLES: &str = "
CREATE TABLE IF NOT EXISTS decodes (
    id      INTEGER PRIMARY KEY,
    t       INTEGER NOT NULL,   -- slot start, UTC seconds
    mode    TEXT    NOT NULL,
    band    TEXT    NOT NULL,
    dial_hz INTEGER NOT NULL,
    freq_hz INTEGER NOT NULL,   -- RF of tone 0
    snr     INTEGER NOT NULL,
    dt      REAL    NOT NULL,
    call    TEXT,               -- the sender, when the text names one
    grid    TEXT,               -- its locator, when the text carries one
    server  TEXT    NOT NULL DEFAULT '',  -- which SpyServer heard it
    cq      TEXT,               -- NULL: not a CQ; '': plain CQ; else DX, POTA, NA...
    text    TEXT    NOT NULL,
    msg_key BLOB,               -- the message's bits, packed: the same message heard twice has one
    sync    REAL,               -- sync correlation score of the decode
    hard_errors INTEGER,        -- hard-decision errors the FEC corrected
    resolved INTEGER            -- 1: the text needed the callsign table (a <...> resolved)
);
CREATE TABLE IF NOT EXISTS stations (
    call TEXT PRIMARY KEY,
    grid TEXT NOT NULL,
    seen INTEGER NOT NULL       -- UTC seconds of the message that gave it
) WITHOUT ROWID;
CREATE TABLE IF NOT EXISTS servers (
    name TEXT PRIMARY KEY,
    grid TEXT NOT NULL            -- where it is: the origin of bearing and distance
) WITHOUT ROWID;
";

/// The columns that carry a row's `RowDetail` (since #592), added to older files.
const DETAIL_COLUMNS: [(&str, &str); 4] = [
    ("msg_key", "BLOB"),
    ("sync", "REAL"),
    ("hard_errors", "INTEGER"),
    ("resolved", "INTEGER"),
];

/// `d.sync`, `d.hard_errors`, `d.resolved` for a select list, or `NULL` where the
/// file has no such column.
fn detail_select(conn: &Connection) -> [&'static str; 3] {
    let pick = |name: &str, col: &'static str| {
        if has_column(conn, name).unwrap_or(false) {
            col
        } else {
            "NULL"
        }
    };
    [
        pick("sync", "d.sync"),
        pick("hard_errors", "d.hard_errors"),
        pick("resolved", "d.resolved"),
    ]
}

fn has_column(conn: &Connection, name: &str) -> rusqlite::Result<bool> {
    conn.prepare("SELECT 1 FROM pragma_table_info('decodes') WHERE name = ?1")?
        .exists([name])
}

/// A message key (one bit per byte) packed into bytes, most significant bit
/// first, so a 91-bit FT8 key is 12 bytes in the file. Empty: no key.
pub fn pack_key(bits: &[u8]) -> Option<Vec<u8>> {
    if bits.is_empty() {
        return None;
    }
    Some(
        bits.chunks(8)
            .map(|c| {
                c.iter()
                    .enumerate()
                    .fold(0u8, |b, (i, &bit)| b | (bit & 1) << (7 - i))
            })
            .collect(),
    )
}

const INDEXES: &str = "
CREATE INDEX IF NOT EXISTS decodes_t      ON decodes (t);
CREATE INDEX IF NOT EXISTS decodes_band_t ON decodes (band, t);
CREATE INDEX IF NOT EXISTS decodes_call_t ON decodes (call, t) WHERE call IS NOT NULL;
CREATE INDEX IF NOT EXISTS decodes_server_t ON decodes (server, t);
";

/// The amateur band a dial frequency falls in, or `other`.
pub fn band_of(hz: f64) -> &'static str {
    const BANDS: &[(&str, f64, f64)] = &[
        ("2200m", 135.7e3, 137.8e3),
        ("630m", 472e3, 479e3),
        ("160m", 1.8e6, 2.0e6),
        ("80m", 3.5e6, 4.0e6),
        ("60m", 5.25e6, 5.45e6),
        ("40m", 7.0e6, 7.3e6),
        ("30m", 10.1e6, 10.15e6),
        ("20m", 14.0e6, 14.35e6),
        ("17m", 18.068e6, 18.168e6),
        ("15m", 21.0e6, 21.45e6),
        ("12m", 24.89e6, 24.99e6),
        ("10m", 28.0e6, 29.7e6),
        ("6m", 50.0e6, 54.0e6),
        ("2m", 144.0e6, 148.0e6),
    ];
    BANDS
        .iter()
        .find(|&&(_, lo, hi)| (lo..=hi).contains(&hz))
        .map_or("other", |b| b.0)
}

fn open(path: &Path, flags: OpenFlags) -> rusqlite::Result<Connection> {
    let c = Connection::open_with_flags(path, flags)?;
    c.busy_timeout(Duration::from_secs(5))?;
    Ok(c)
}

/// Writes decodes in batches; the batch is also written when the writer is
/// dropped.
pub struct Writer {
    conn: Connection,
    pending: Vec<Row>,
    since: Instant,
}

struct Row {
    t: i64,
    server: String,
    mode: &'static str,
    band: &'static str,
    dial_hz: i64,
    freq_hz: i64,
    snr: i64,
    dt: f64,
    call: Option<String>,
    grid: Option<String>,
    cq: Option<String>,
    text: String,
    msg_key: Option<Vec<u8>>,
    sync: f64,
    hard_errors: i64,
    resolved: bool,
    /// Not a new row: the text of the one already written (or still waiting
    /// here) with this server, slot, channel, frequency and key, now resolved.
    update: bool,
}

/// Rows buffered at most this long, or this many, before a transaction.
const FLUSH_EVERY: Duration = Duration::from_secs(2);
const FLUSH_ROWS: usize = 200;

impl Writer {
    /// `servers` are `(name, locator)`: where each SpyServer is, kept in the
    /// file so a bearing is read from the place that heard the signal.
    pub fn open(path: &Path, servers: &[(String, String)]) -> rusqlite::Result<Writer> {
        let conn = open(
            path,
            OpenFlags::SQLITE_OPEN_READ_WRITE | OpenFlags::SQLITE_OPEN_CREATE,
        )?;
        conn.pragma_update(None, "journal_mode", "WAL")?;
        conn.pragma_update(None, "synchronous", "NORMAL")?;
        conn.execute_batch(TABLES)?;
        // A file from before there were several servers.
        let has_server = conn
            .prepare("SELECT 1 FROM pragma_table_info('decodes') WHERE name = 'server'")?
            .exists([])?;
        if !has_server {
            conn.execute_batch("ALTER TABLE decodes ADD COLUMN server TEXT NOT NULL DEFAULT ''")?;
        }
        // A file from before the decoder's row detail was kept (NULL there).
        for (col, ty) in DETAIL_COLUMNS {
            if !has_column(&conn, col)? {
                conn.execute_batch(&format!("ALTER TABLE decodes ADD COLUMN {col} {ty}"))?;
            }
        }
        conn.execute_batch(INDEXES)?;
        for (name, grid) in servers {
            conn.execute(
                "INSERT INTO servers (name, grid) VALUES (?1, ?2)
                 ON CONFLICT(name) DO UPDATE SET grid = excluded.grid",
                params![name, grid],
            )?;
        }
        Ok(Writer {
            conn,
            pending: Vec::new(),
            since: Instant::now(),
        })
    }

    pub fn push(&mut self, server: &str, d: &Decode) {
        let name = modes::mode_name(d.mode);
        let (call, grid) = spot::sender(name, &d.text);
        let row = Row {
            server: server.to_string(),
            t: d.slot_utc_ns.map_or(0, |ns| ns.div_euclid(1_000_000_000)),
            mode: name,
            band: band_of(d.dial_hz),
            dial_hz: d.dial_hz.round() as i64,
            freq_hz: d.freq_hz.round() as i64,
            snr: d.snr_db.round() as i64,
            dt: f64::from(d.dt_s),
            call,
            grid,
            cq: spot::cq_kind(&d.text),
            text: d.text.clone(),
            msg_key: pack_key(&d.detail.key),
            sync: f64::from(d.detail.sync_score),
            hard_errors: i64::from(d.detail.hard_errors),
            resolved: d.detail.hash_resolved,
            update: d.update,
        };
        // An update of a row still waiting is made in place.
        let waiting = row.update.then(|| {
            self.pending.iter_mut().find(|p| {
                !p.update
                    && p.server == row.server
                    && p.t == row.t
                    && p.dial_hz == row.dial_hz
                    && p.freq_hz == row.freq_hz
                    && p.msg_key == row.msg_key
            })
        });
        match waiting.flatten() {
            Some(p) => {
                p.call = row.call;
                p.grid = row.grid;
                p.cq = row.cq;
                p.text = row.text;
                p.resolved = row.resolved;
            }
            None => self.pending.push(row),
        }
        if self.pending.len() >= FLUSH_ROWS || self.since.elapsed() >= FLUSH_EVERY {
            self.flush();
        }
    }

    #[cfg(test)]
    fn drop_flush(&mut self) {
        self.flush();
    }

    /// Write what is buffered; call now and then so a quiet band does not
    /// leave the last decodes waiting.
    pub fn flush(&mut self) {
        self.since = Instant::now();
        if self.pending.is_empty() {
            return;
        }
        let rows = std::mem::take(&mut self.pending);
        if let Err(e) = self.write(&rows) {
            eprintln!("skimmer: database write failed: {e}");
        }
    }

    fn write(&mut self, rows: &[Row]) -> rusqlite::Result<()> {
        let tx = self.conn.transaction()?;
        {
            let mut ins = tx.prepare_cached(
                "INSERT INTO decodes (t, mode, band, dial_hz, freq_hz, snr, dt, call, grid, cq, text, server,
                                      msg_key, sync, hard_errors, resolved)
                 VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13, ?14, ?15, ?16)",
            )?;
            // An update finds the row by what identifies it; a file with no key
            // (a mode that gave none) is matched by its old text's place alone.
            let mut upd = tx.prepare_cached(
                "UPDATE decodes SET call = ?1, grid = ?2, cq = ?3, text = ?4, resolved = ?5
                 WHERE server = ?6 AND t = ?7 AND dial_hz = ?8 AND freq_hz = ?9
                   AND msg_key IS ?10",
            )?;
            let mut st = tx.prepare_cached(
                "INSERT INTO stations (call, grid, seen) VALUES (?1, ?2, ?3)
                 ON CONFLICT(call) DO UPDATE SET grid = excluded.grid, seen = excluded.seen
                 WHERE excluded.seen >= seen",
            )?;
            for r in rows {
                if r.update {
                    upd.execute(params![
                        r.call, r.grid, r.cq, r.text, r.resolved, r.server, r.t, r.dial_hz,
                        r.freq_hz, r.msg_key
                    ])?;
                } else {
                    ins.execute(params![
                        r.t,
                        r.mode,
                        r.band,
                        r.dial_hz,
                        r.freq_hz,
                        r.snr,
                        r.dt,
                        r.call,
                        r.grid,
                        r.cq,
                        r.text,
                        r.server,
                        r.msg_key,
                        r.sync,
                        r.hard_errors,
                        r.resolved
                    ])?;
                }
                if let (Some(c), Some(g)) = (&r.call, &r.grid) {
                    st.execute(params![c, g, r.t])?;
                }
            }
        }
        tx.commit()
    }
}

impl Drop for Writer {
    fn drop(&mut self) {
        self.flush();
    }
}

/// What to look for. Every field narrows the result; an empty one does not.
/// Time, band, mode, SNR, regexes and CQ kind are conditions of the SQL
/// (indexed on time and band); distance and bearing are computed from
/// the locators by functions registered on the connection.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct Query {
    /// UTC seconds, inclusive.
    pub since: i64,
    pub until: i64,
    /// My locator: the origin of distance and bearing for what no server's
    /// own locator covers.
    pub me: String,
    /// Regular expressions (case-insensitive, unanchored: write `^` and `$`)
    /// on the sender's call, its locator and the message text.
    pub call: String,
    pub grid: String,
    pub text: String,
    pub bands: Vec<String>,
    pub modes: Vec<String>,
    /// Only what these servers heard; empty is all.
    pub servers: Vec<String>,
    pub snr_min: Option<i64>,
    pub snr_max: Option<i64>,
    pub km_min: Option<f64>,
    pub km_max: Option<f64>,
    /// Bearing sector, degrees clockwise from north; `from > to` wraps
    /// through north (315 to 45 is the northern quarter).
    pub bearing_from: Option<f64>,
    pub bearing_to: Option<f64>,
    /// `None`: any message; `Some("*")`: any CQ; `Some("")`: a plain CQ;
    /// `Some("DX")`, `Some("POTA")`...: that modifier.
    pub cq: Option<String>,
}

/// The locator of a row: its own, else the last one its sender sent.
const GRID: &str = "COALESCE(d.grid, s.grid)";
const FROM: &str = "decodes d LEFT JOIN stations s ON s.call = d.call
                    LEFT JOIN servers v ON v.name = d.server";
/// Where a row was heard from: its server's locator, else the one asked
/// for (one `?`).
const ORIGIN: &str = "COALESCE(NULLIF(v.grid, ''), ?)";

impl Query {
    /// `WHERE` conditions and their parameters.
    fn sql(&self) -> Result<(String, Vec<rusqlite::types::Value>), String> {
        use rusqlite::types::Value as V;
        let mut w = vec!["d.t BETWEEN ? AND ?".to_string()];
        let mut p: Vec<V> = vec![self.since.into(), self.until.into()];
        for (col, re) in [
            ("d.call", &self.call),
            (GRID, &self.grid),
            ("d.text", &self.text),
        ] {
            let re = re.trim();
            if re.is_empty() {
                continue;
            }
            regex::Regex::new(&format!("(?i){re}")).map_err(|e| format!("{re}: {e}"))?;
            w.push(format!("{col} REGEXP ?"));
            p.push(re.to_string().into());
        }
        if !self.bands.is_empty() {
            w.push(format!(
                "d.band IN ({})",
                vec!["?"; self.bands.len()].join(",")
            ));
            p.extend(self.bands.iter().map(|x| V::from(x.clone())));
        }
        if !self.modes.is_empty() {
            // `FST4*` is every FST4 sub-mode (`FST4-60`, ...).
            let any = self
                .modes
                .iter()
                .map(|m| {
                    if m.ends_with('*') {
                        "d.mode LIKE ?"
                    } else {
                        "d.mode = ?"
                    }
                })
                .collect::<Vec<_>>()
                .join(" OR ");
            w.push(format!("({any})"));
            p.extend(self.modes.iter().map(|m| match m.strip_suffix('*') {
                Some(prefix) => V::from(format!("{prefix}%")),
                None => V::from(m.clone()),
            }));
        }
        if !self.servers.is_empty() {
            w.push(format!(
                "d.server IN ({})",
                vec!["?"; self.servers.len()].join(",")
            ));
            p.extend(self.servers.iter().map(|x| V::from(x.clone())));
        }
        if let Some(x) = self.snr_min {
            w.push("d.snr >= ?".into());
            p.push(x.into());
        }
        if let Some(x) = self.snr_max {
            w.push("d.snr <= ?".into());
            p.push(x.into());
        }
        let me = self.me.trim();
        if self.km_min.is_some() || self.km_max.is_some() {
            if let Some(x) = self.km_min {
                w.push(format!("dist_km({ORIGIN}, {GRID}) >= ?"));
                p.extend([me.to_string().into(), x.into()]);
            }
            if let Some(x) = self.km_max {
                w.push(format!("dist_km({ORIGIN}, {GRID}) <= ?"));
                p.extend([me.to_string().into(), x.into()]);
            }
        }
        if let (Some(a), Some(b)) = (self.bearing_from, self.bearing_to) {
            w.push(format!("in_sector({ORIGIN}, {GRID}, ?, ?)"));
            p.extend([me.to_string().into(), a.into(), b.into()]);
        }
        match self.cq.as_deref() {
            None => {}
            Some("*") => w.push("d.cq IS NOT NULL".into()),
            Some(k) => {
                w.push("d.cq = ?".into());
                p.push(k.to_string().into());
            }
        }
        Ok((w.join(" AND "), p))
    }
}

/// Distinct senders and decodes per hour and band: the band's state over
/// time.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Activity {
    /// UTC hour since the epoch (`t / 3600`).
    pub hour: i64,
    pub band: String,
    pub stations: i64,
    pub decodes: i64,
    pub best_snr: i64,
}

/// One sender in a result.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Station {
    pub call: String,
    pub grid: Option<String>,
    /// Comma-separated bands it was heard on.
    pub bands: String,
    /// Comma-separated servers that heard it.
    pub servers: String,
    pub count: i64,
    pub best_snr: i64,
    pub first: i64,
    pub last: i64,
    pub bearing: Option<f64>,
    pub km: Option<f64>,
}

/// One decode in a result.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Spot {
    pub t: i64,
    pub server: String,
    pub call: Option<String>,
    pub grid: Option<String>,
    pub band: String,
    pub mode: String,
    /// Audio offset of tone 0, Hz.
    pub audio_hz: i64,
    pub snr: i64,
    pub dt: f64,
    /// `None`: not a CQ; `""`: plain; else DX, POTA...
    pub cq: Option<String>,
    pub text: String,
    pub bearing: Option<f64>,
    pub km: Option<f64>,
    /// What the decoder knew of the row (`None`: a file from before it was kept).
    pub sync: Option<f64>,
    pub hard_errors: Option<i64>,
    /// The text needed the callsign table: a `<...>` was resolved.
    pub resolved: Option<bool>,
}

/// A sender heard in one slice of time: what the map animation draws.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Point {
    /// Start of the slice, UTC seconds.
    pub t: i64,
    pub call: String,
    pub grid: String,
    pub band: String,
    pub snr: i64,
}

/// What a query found, in numbers.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Summary {
    pub decodes: i64,
    pub stations: i64,
}

fn register(conn: &Connection) -> rusqlite::Result<()> {
    use rusqlite::functions::FunctionFlags as F;
    let det = F::SQLITE_UTF8 | F::SQLITE_DETERMINISTIC;
    // X REGEXP Y calls regexp(Y, X); the pattern is compiled once per query.
    conn.create_scalar_function("regexp", 2, det, |ctx| {
        let re = ctx.get_or_create_aux(0, |vr| -> Result<_, regex::Error> {
            regex::Regex::new(&format!("(?i){}", vr.as_str().unwrap_or("")))
        })?;
        Ok(match ctx.get_raw(1).as_str_or_null() {
            Ok(Some(t)) => re.is_match(t),
            _ => false,
        })
    })?;
    // (lat, lon) of a locator argument, or None.
    fn pt(ctx: &rusqlite::functions::Context<'_>, i: usize) -> Option<(f64, f64)> {
        ctx.get_raw(i)
            .as_str_or_null()
            .ok()
            .flatten()
            .and_then(geo::grid_center)
    }
    conn.create_scalar_function("dist_km", 2, det, |ctx| {
        Ok(pt(ctx, 0)
            .zip(pt(ctx, 1))
            .map(|(a, b)| geo::bearing_distance(a, b).1))
    })?;
    conn.create_scalar_function("bearing_deg", 2, det, |ctx| {
        Ok(pt(ctx, 0)
            .zip(pt(ctx, 1))
            .map(|(a, b)| geo::bearing_distance(a, b).0))
    })?;
    conn.create_scalar_function("in_sector", 4, det, |ctx| {
        let (from, to): (f64, f64) = (ctx.get(2)?, ctx.get(3)?);
        Ok(pt(ctx, 0).zip(pt(ctx, 1)).map(|(a, b)| {
            let bearing = geo::bearing_distance(a, b).0;
            if from <= to {
                (from..=to).contains(&bearing)
            } else {
                bearing >= from || bearing <= to
            }
        }))
    })?;
    Ok(())
}

/// Read-only queries; any number may run beside the writer.
pub struct Reader {
    conn: Connection,
}

impl Reader {
    pub fn open(path: &Path) -> rusqlite::Result<Reader> {
        let conn = open(path, OpenFlags::SQLITE_OPEN_READ_ONLY)?;
        register(&conn)?;
        Ok(Reader { conn })
    }

    fn bearing_km(me: &str, grid: Option<&str>) -> (Option<f64>, Option<f64>) {
        let bd = geo::grid_center(me)
            .zip(grid.and_then(geo::grid_center))
            .map(|(a, b)| geo::bearing_distance(a, b));
        (bd.map(|x| x.0), bd.map(|x| x.1))
    }

    /// Senders and decodes per hour and band.
    pub fn activity(&self, q: &Query) -> Result<Vec<Activity>, String> {
        let (w, p) = q.sql()?;
        let sql = format!(
            "SELECT d.t / 3600, d.band, COUNT(DISTINCT d.call), COUNT(*), MAX(d.snr)
             FROM {FROM} WHERE {w} GROUP BY d.t / 3600, d.band ORDER BY 1"
        );
        let mut st = self.conn.prepare(&sql).map_err(|e| e.to_string())?;
        let rows = st
            .query_map(rusqlite::params_from_iter(p), |r| {
                Ok(Activity {
                    hour: r.get(0)?,
                    band: r.get(1)?,
                    stations: r.get(2)?,
                    decodes: r.get(3)?,
                    best_snr: r.get(4)?,
                })
            })
            .map_err(|e| e.to_string())?;
        rows.collect::<Result<_, _>>().map_err(|e| e.to_string())
    }

    /// The senders found, the most heard first. Bearing and distance are read
    /// from the first (by locator) of the servers that heard the sender, or
    /// from `me` when none has a locator.
    pub fn stations(&self, q: &Query, limit: usize) -> Result<Vec<Station>, String> {
        let (w, p) = q.sql()?;
        let sql = format!(
            "SELECT d.call, MAX({GRID}), group_concat(DISTINCT d.band), COUNT(*), MAX(d.snr),
                    MIN(d.t), MAX(d.t), group_concat(DISTINCT d.server), MIN(NULLIF(v.grid, ''))
             FROM {FROM} WHERE {w} AND d.call IS NOT NULL
             GROUP BY d.call ORDER BY 4 DESC LIMIT {limit}"
        );
        let mut st = self.conn.prepare(&sql).map_err(|e| e.to_string())?;
        let rows = st
            .query_map(rusqlite::params_from_iter(p), |r| {
                let grid: Option<String> = r.get(1)?;
                let from: Option<String> = r.get(8)?;
                let (bearing, km) =
                    Self::bearing_km(from.as_deref().unwrap_or(&q.me), grid.as_deref());
                Ok(Station {
                    call: r.get(0)?,
                    grid,
                    bands: r.get(2)?,
                    servers: r.get::<_, Option<String>>(7)?.unwrap_or_default(),
                    count: r.get(3)?,
                    best_snr: r.get(4)?,
                    first: r.get(5)?,
                    last: r.get(6)?,
                    bearing,
                    km,
                })
            })
            .map_err(|e| e.to_string())?;
        rows.collect::<Result<_, _>>().map_err(|e| e.to_string())
    }

    /// The decodes found, newest first, with bearing and distance from the
    /// server that heard each.
    pub fn decodes(&self, q: &Query, limit: usize) -> Result<Vec<Spot>, String> {
        let (w, mut wp) = q.sql()?;
        let me = q.me.trim().to_string();
        let [sync, hard, resolved] = detail_select(&self.conn);
        let sql = format!(
            "SELECT d.t, d.call, {GRID}, d.band, d.mode, d.freq_hz - d.dial_hz, d.snr, d.dt, d.cq, d.text,
                    d.server, bearing_deg({ORIGIN}, {GRID}), dist_km({ORIGIN}, {GRID}),
                    {sync}, {hard}, {resolved}
             FROM {FROM} WHERE {w} ORDER BY d.t DESC LIMIT {limit}"
        );
        // The two origins in the select list come before the WHERE's parameters.
        let mut p: Vec<rusqlite::types::Value> = vec![me.clone().into(), me.into()];
        p.append(&mut wp);
        let mut st = self.conn.prepare(&sql).map_err(|e| e.to_string())?;
        let rows = st
            .query_map(rusqlite::params_from_iter(p), |r| {
                Ok(Spot {
                    t: r.get(0)?,
                    call: r.get(1)?,
                    grid: r.get(2)?,
                    band: r.get(3)?,
                    mode: r.get(4)?,
                    audio_hz: r.get(5)?,
                    snr: r.get(6)?,
                    dt: r.get(7)?,
                    cq: r.get(8)?,
                    text: r.get(9)?,
                    server: r.get(10)?,
                    bearing: r.get(11)?,
                    km: r.get(12)?,
                    sync: r.get(13)?,
                    hard_errors: r.get(14)?,
                    resolved: r.get::<_, Option<i64>>(15)?.map(|v| v != 0),
                })
            })
            .map_err(|e| e.to_string())?;
        rows.collect::<Result<_, _>>().map_err(|e| e.to_string())
    }

    /// Who was heard in each `slice_s` of the range, for the map and its
    /// animation: one point per (slice, call, band) with a known locator.
    pub fn points(&self, q: &Query, slice_s: i64, limit: usize) -> Result<Vec<Point>, String> {
        let s = slice_s.max(60);
        let (w, p) = q.sql()?;
        let sql = format!(
            "SELECT d.t / {s} * {s}, d.call, {GRID}, d.band, MAX(d.snr)
             FROM {FROM} WHERE {w} AND d.call IS NOT NULL AND {GRID} IS NOT NULL
             GROUP BY 1, d.call, d.band ORDER BY 1 LIMIT {limit}"
        );
        let mut st = self.conn.prepare(&sql).map_err(|e| e.to_string())?;
        let rows = st
            .query_map(rusqlite::params_from_iter(p), |r| {
                Ok(Point {
                    t: r.get(0)?,
                    call: r.get(1)?,
                    grid: r.get(2)?,
                    band: r.get(3)?,
                    snr: r.get(4)?,
                })
            })
            .map_err(|e| e.to_string())?;
        rows.collect::<Result<_, _>>().map_err(|e| e.to_string())
    }

    /// How many decodes at each SNR, with the query's own SNR limits left
    /// out: the distribution a limit is chosen against.
    pub fn snr_histogram(&self, q: &Query) -> Result<Vec<(i64, i64)>, String> {
        let open = Query {
            snr_min: None,
            snr_max: None,
            ..q.clone()
        };
        let (w, p) = open.sql()?;
        let sql = format!("SELECT d.snr, COUNT(*) FROM {FROM} WHERE {w} GROUP BY d.snr ORDER BY 1");
        let mut st = self.conn.prepare(&sql).map_err(|e| e.to_string())?;
        let rows = st
            .query_map(rusqlite::params_from_iter(p), |r| {
                Ok((r.get(0)?, r.get(1)?))
            })
            .map_err(|e| e.to_string())?;
        rows.collect::<Result<_, _>>().map_err(|e| e.to_string())
    }

    pub fn summary(&self, q: &Query) -> Result<Summary, String> {
        let (w, p) = q.sql()?;
        let sql = format!("SELECT COUNT(*), COUNT(DISTINCT d.call) FROM {FROM} WHERE {w}");
        self.conn
            .query_row(&sql, rusqlite::params_from_iter(p), |r| {
                Ok(Summary {
                    decodes: r.get(0)?,
                    stations: r.get(1)?,
                })
            })
            .map_err(|e| e.to_string())
    }

    /// The servers there are decodes from, with their locators (empty when
    /// the file has none): the legacy single server is named `""`.
    pub fn servers(&self) -> rusqlite::Result<Vec<(String, String)>> {
        let mut q = self.conn.prepare(
            "SELECT d.server, COALESCE(v.grid, '') FROM (SELECT server FROM decodes GROUP BY server) d
             LEFT JOIN servers v ON v.name = d.server ORDER BY 1",
        )?;
        let rows = q.query_map([], |r| Ok((r.get(0)?, r.get(1)?)))?;
        rows.collect()
    }

    /// The bands anything was recorded on.
    pub fn bands(&self) -> rusqlite::Result<Vec<String>> {
        let mut q = self
            .conn
            .prepare("SELECT band FROM decodes GROUP BY band")?;
        let rows = q.query_map([], |r| r.get(0))?;
        rows.collect()
    }

    /// Time span and size of what is stored: first and last decode, rows.
    pub fn span(&self) -> rusqlite::Result<(Option<i64>, Option<i64>, i64)> {
        self.conn
            .query_row("SELECT MIN(t), MAX(t), COUNT(*) FROM decodes", [], |r| {
                Ok((r.get(0)?, r.get(1)?, r.get(2)?))
            })
    }
}

// ---------------------------------------------------------------------------
// Looking after the file: what is in it, clearing out, a copy, a CSV.
// ---------------------------------------------------------------------------

/// What the file holds and how big it is.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DbInfo {
    pub path: String,
    /// The main file and its write-ahead log, bytes.
    pub bytes: u64,
    pub wal_bytes: u64,
    /// Bytes the file could shrink by (free pages); `VACUUM` returns them.
    pub reclaimable: u64,
    pub decodes: i64,
    pub stations: i64,
    pub first: Option<i64>,
    pub last: Option<i64>,
    /// `(server, decodes)`; the legacy single server is `""`.
    pub servers: Vec<(String, i64)>,
}

fn rw(path: &Path) -> Result<Connection, String> {
    let c = open(path, OpenFlags::SQLITE_OPEN_READ_WRITE).map_err(|e| e.to_string())?;
    // The skimmer may be recording: wait for it rather than fail at once.
    c.busy_timeout(Duration::from_secs(20))
        .map_err(|e| e.to_string())?;
    Ok(c)
}

pub fn info(path: &Path) -> Result<DbInfo, String> {
    let c = open(path, OpenFlags::SQLITE_OPEN_READ_ONLY).map_err(|e| e.to_string())?;
    let one = |sql: &str| -> rusqlite::Result<i64> { c.query_row(sql, [], |r| r.get(0)) };
    let (first, last): (Option<i64>, Option<i64>) = c
        .query_row("SELECT MIN(t), MAX(t) FROM decodes", [], |r| {
            Ok((r.get(0)?, r.get(1)?))
        })
        .map_err(|e| e.to_string())?;
    let mut q = c
        .prepare("SELECT server, COUNT(*) FROM decodes GROUP BY server ORDER BY 2 DESC")
        .map_err(|e| e.to_string())?;
    let servers = q
        .query_map([], |r| Ok((r.get(0)?, r.get(1)?)))
        .map_err(|e| e.to_string())?
        .collect::<Result<_, _>>()
        .map_err(|e| e.to_string())?;
    let page = one("PRAGMA page_size").map_err(|e| e.to_string())?;
    let free = one("PRAGMA freelist_count").map_err(|e| e.to_string())?;
    let wal = std::fs::metadata(format!("{}-wal", path.display())).map_or(0, |m| m.len());
    Ok(DbInfo {
        path: path.display().to_string(),
        bytes: std::fs::metadata(path).map_or(0, |m| m.len()),
        wal_bytes: wal,
        reclaimable: (page * free) as u64,
        decodes: one("SELECT COUNT(*) FROM decodes").map_err(|e| e.to_string())?,
        stations: one("SELECT COUNT(*) FROM stations").map_err(|e| e.to_string())?,
        first,
        last,
        servers,
    })
}

/// How many decodes are older than `before` (UTC seconds), optionally of one server.
pub fn count_before(path: &Path, before: i64, server: Option<&str>) -> Result<i64, String> {
    let c = open(path, OpenFlags::SQLITE_OPEN_READ_ONLY).map_err(|e| e.to_string())?;
    c.query_row(
        "SELECT COUNT(*) FROM decodes WHERE t < ?1 AND (?2 IS NULL OR server = ?2)",
        params![before, server],
        |r| r.get(0),
    )
    .map_err(|e| e.to_string())
}

/// Delete the decodes older than `before` (UTC seconds), optionally of one
/// server only; the locators learned before then go too. Returns the number
/// of decodes removed. The space is returned to the system by [`vacuum`].
pub fn delete_before(path: &Path, before: i64, server: Option<&str>) -> Result<i64, String> {
    let mut c = rw(path)?;
    let tx = c.transaction().map_err(|e| e.to_string())?;
    let n = tx
        .execute(
            "DELETE FROM decodes WHERE t < ?1 AND (?2 IS NULL OR server = ?2)",
            params![before, server],
        )
        .map_err(|e| e.to_string())?;
    if server.is_none() {
        tx.execute("DELETE FROM stations WHERE seen < ?1", params![before])
            .map_err(|e| e.to_string())?;
    }
    tx.commit().map_err(|e| e.to_string())?;
    Ok(n as i64)
}

/// Record everything `from` heard as heard by `to` (the legacy single server
/// is `from = ""`), and make `to` a server of its own. Returns the number of
/// decodes renamed.
pub fn rename_server(path: &Path, from: &str, to: &str) -> Result<i64, String> {
    let mut c = rw(path)?;
    let tx = c.transaction().map_err(|e| e.to_string())?;
    let n = tx
        .execute(
            "UPDATE decodes SET server = ?2 WHERE server = ?1",
            params![from, to],
        )
        .map_err(|e| e.to_string())?;
    tx.execute(
        "INSERT INTO servers (name, grid) VALUES (?1, '') ON CONFLICT(name) DO NOTHING",
        params![to],
    )
    .map_err(|e| e.to_string())?;
    tx.execute(
        "DELETE FROM servers WHERE name = ?1 AND name != ?2",
        params![from, to],
    )
    .map_err(|e| e.to_string())?;
    tx.commit().map_err(|e| e.to_string())?;
    Ok(n as i64)
}

/// Return the free space to the system: the log is folded into the file and the
/// file rewritten compactly. Needs the file to itself for a moment; with the
/// skimmer recording it waits up to twenty seconds, then says so.
pub fn vacuum(path: &Path) -> Result<(), String> {
    let c = rw(path)?;
    c.pragma_update(None, "wal_checkpoint", "TRUNCATE")
        .map_err(|e| e.to_string())?;
    c.execute_batch("VACUUM").map_err(|e| {
        if e.to_string().contains("locked") || e.to_string().contains("busy") {
            "the database is in use; disconnect the skimmer and try again".to_string()
        } else {
            e.to_string()
        }
    })
}

/// A compact, consistent copy of the whole file at `dest`, made while the
/// skimmer records.
pub fn backup(path: &Path, dest: &Path) -> Result<(), String> {
    if dest.exists() {
        return Err(format!("{} exists already", dest.display()));
    }
    let c = open(path, OpenFlags::SQLITE_OPEN_READ_ONLY).map_err(|e| e.to_string())?;
    c.execute("VACUUM INTO ?1", params![dest.display().to_string()])
        .map_err(|e| e.to_string())?;
    Ok(())
}

fn csv_field(s: &str) -> String {
    if s.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

/// The decodes a query finds, as CSV at `dest`, oldest first. Returns the rows written.
pub fn export_csv(path: &Path, q: &Query, dest: &Path) -> Result<i64, String> {
    use std::io::Write;
    let r = Reader::open(path).map_err(|e| e.to_string())?;
    let (w, mut wp) = q.sql()?;
    let me = q.me.trim().to_string();
    let [sync, hard, resolved] = detail_select(&r.conn);
    let sql = format!(
        "SELECT d.t, d.server, d.call, {GRID}, d.band, d.mode, d.dial_hz, d.freq_hz - d.dial_hz,
                d.snr, d.dt, d.cq, d.text, bearing_deg({ORIGIN}, {GRID}), dist_km({ORIGIN}, {GRID}),
                {sync}, {hard}, {resolved}
         FROM {FROM} WHERE {w} ORDER BY d.t"
    );
    let mut p: Vec<rusqlite::types::Value> = vec![me.clone().into(), me.into()];
    p.append(&mut wp);
    let mut st = r.conn.prepare(&sql).map_err(|e| e.to_string())?;
    let mut out = std::io::BufWriter::new(
        std::fs::File::create(dest).map_err(|e| format!("{}: {e}", dest.display()))?,
    );
    writeln!(
        out,
        "utc,server,call,grid,band,mode,dial_hz,audio_hz,snr_db,dt_s,cq,message,bearing_deg,distance_km,sync,hard_errors,resolved"
    )
    .map_err(|e| e.to_string())?;
    let mut rows = st
        .query(rusqlite::params_from_iter(p))
        .map_err(|e| e.to_string())?;
    let mut n = 0;
    while let Some(row) = rows.next().map_err(|e| e.to_string())? {
        let t: i64 = row.get(0).map_err(|e| e.to_string())?;
        let (days, sod) = (t.div_euclid(86_400), t.rem_euclid(86_400));
        let (y, m, d) = crate::civil_from_days(days);
        let text = |i: usize| -> String {
            row.get::<_, Option<String>>(i)
                .ok()
                .flatten()
                .unwrap_or_default()
        };
        let num = |i: usize| -> String {
            row.get::<_, Option<f64>>(i)
                .ok()
                .flatten()
                .map_or(String::new(), |v| format!("{v:.1}"))
        };
        let int = |i: usize| -> i64 { row.get::<_, Option<i64>>(i).ok().flatten().unwrap_or(0) };
        // Blank where the file has none (an older one, or a mode with no value).
        let opt_int = |i: usize| -> String {
            row.get::<_, Option<i64>>(i)
                .ok()
                .flatten()
                .map_or(String::new(), |v| v.to_string())
        };
        writeln!(
            out,
            "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z,{},{},{},{},{},{},{},{},{:.1},{},{},{},{},{},{},{}",
            sod / 3600,
            sod / 60 % 60,
            sod % 60,
            csv_field(&text(1)),
            csv_field(&text(2)),
            csv_field(&text(3)),
            csv_field(&text(4)),
            csv_field(&text(5)),
            int(6),
            int(7),
            int(8),
            row.get::<_, f64>(9).unwrap_or(0.0),
            csv_field(&text(10)),
            csv_field(&text(11)),
            num(12),
            num(13),
            num(14),
            opt_int(15),
            opt_int(16)
        )
        .map_err(|e| e.to_string())?;
        n += 1;
    }
    out.flush().map_err(|e| e.to_string())?;
    Ok(n)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mfsk_core::Mode;

    fn decode(t: i64, dial: f64, snr: f32, text: &str) -> Decode {
        Decode {
            channel: 0,
            mode: Mode::Ft8,
            slot_utc_ns: Some(t * 1_000_000_000),
            dial_hz: dial,
            freq_hz: dial + 1500.0,
            snr_db: snr,
            dt_s: 0.1,
            text: text.into(),
            detail: crate::DecodeDetail::default(),
            update: false,
        }
    }

    fn db() -> std::path::PathBuf {
        let p = std::env::temp_dir().join(format!(
            "skimmer-store-{}-{:?}.db",
            std::process::id(),
            std::thread::current().id()
        ));
        for ext in ["", "-wal", "-shm"] {
            let _ = std::fs::remove_file(format!("{}{ext}", p.display()));
        }
        p
    }

    #[test]
    fn bands() {
        assert_eq!(band_of(14.074e6), "20m");
        assert_eq!(band_of(7.074e6), "40m");
        assert_eq!(band_of(10.136e6), "30m");
        assert_eq!(band_of(9.0e6), "other");
    }

    fn q(h: i64) -> Query {
        Query {
            since: h,
            until: h + 7200,
            me: "PM95".into(),
            ..Query::default()
        }
    }

    #[test]
    fn stored_and_queried_while_the_writer_is_open() {
        let p = db();
        let mut w = Writer::open(&p, &[]).unwrap();
        let h = 1_700_000_000 / 3600 * 3600;
        w.push("", &decode(h, 14.074e6, -10.0, "CQ K1ABC FN42"));
        w.push("", &decode(h + 15, 14.074e6, -5.0, "JA1XYZ K1ABC -07"));
        w.push("", &decode(h + 3600, 14.074e6, -12.0, "CQ K1ABC"));
        w.push("", &decode(h + 30, 7.074e6, -20.0, "CQ DX W9XYZ EN34"));
        w.flush();
        // The writer still holds the file open: WAL lets a reader in.
        let r = Reader::open(&p).unwrap();
        assert_eq!(r.span().unwrap().2, 4);
        assert_eq!(
            r.summary(&q(h)).unwrap(),
            Summary {
                decodes: 4,
                stations: 2
            }
        );

        // K1ABC's later message without a locator still has FN42, and shows
        // in two separate hours.
        let k = Query {
            call: "^k1abc$".into(),
            ..q(h)
        };
        let st = r.stations(&k, 10).unwrap();
        assert_eq!((st.len(), st[0].count, st[0].best_snr), (1, 3, -5));
        assert_eq!(st[0].grid.as_deref(), Some("FN42"));
        assert!(
            (0.0..90.0).contains(&st[0].bearing.unwrap()),
            "{:?}",
            st[0].bearing
        );
        assert_eq!(r.activity(&k).unwrap().len(), 2);

        // Band, mode, SNR.
        let on = |f: &dyn Fn(&mut Query)| {
            let mut x = q(h);
            f(&mut x);
            r.summary(&x).unwrap().decodes
        };
        assert_eq!(on(&|x| x.bands = vec!["40m".into()]), 1);
        assert_eq!(on(&|x| x.modes = vec!["FT4".into()]), 0);
        assert_eq!(on(&|x| x.modes = vec!["FT8*".into()]), 4);
        assert_eq!(r.bands().unwrap().len(), 2);
        assert_eq!(r.servers().unwrap(), vec![(String::new(), String::new())]);
        assert_eq!(on(&|x| x.snr_min = Some(-11)), 2);
        assert_eq!(on(&|x| x.snr_max = Some(-12)), 2);
        let hist = r
            .snr_histogram(&Query {
                snr_max: Some(-12),
                ..q(h)
            })
            .unwrap();
        assert_eq!(
            hist.iter().map(|x| x.1).sum::<i64>(),
            4,
            "the SNR limit is left out of its own histogram"
        );
        assert_eq!(on(&|x| (x.snr_min, x.snr_max) = (Some(-12), Some(-10))), 2);
        // CQ kinds.
        assert_eq!(on(&|x| x.cq = Some("*".into())), 3);
        assert_eq!(on(&|x| x.cq = Some("DX".into())), 1);
        assert_eq!(on(&|x| x.cq = Some(String::new())), 2);
        // Locator and message regexes.
        assert_eq!(on(&|x| x.grid = "^EN".into()), 1);
        assert_eq!(on(&|x| x.text = "-07$".into()), 1);
        // Distance: Tokyo to FN42 is far, to EN34 farther still.
        let near = on(&|x| x.km_max = Some(9_000.0));
        let far = on(&|x| x.km_min = Some(9_000.0));
        assert!(near == 0 && far == 4, "{near} {far}");
        // Bearing: both lie to the north-east (Tokyo -> USA); 315..45 wraps
        // through north.
        assert_eq!(
            on(&|x| (x.bearing_from, x.bearing_to) = (Some(0.0), Some(90.0))),
            4
        );
        assert_eq!(
            on(&|x| (x.bearing_from, x.bearing_to) = (Some(180.0), Some(270.0))),
            0
        );
        assert_eq!(
            on(&|x| (x.bearing_from, x.bearing_to) = (Some(300.0), Some(60.0))),
            4
        );
        // A bad regex is an error that says which.
        assert!(
            r.summary(&Query {
                call: "(".into(),
                ..q(h)
            })
            .is_err()
        );

        let spots = r.decodes(&q(h), 10).unwrap();
        assert_eq!(spots.len(), 4);
        assert_eq!(spots[0].t, h + 3600);
        assert_eq!(spots[0].audio_hz, 1500);

        // Slices of 30 minutes: K1ABC twice (two hours), W9XYZ once.
        let pts = r.points(&q(h), 1800, 100).unwrap();
        assert_eq!(pts.len(), 3);
        drop(w);
        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn unflushed_rows_are_written_on_drop() {
        let p = db();
        {
            let mut w = Writer::open(&p, &[]).unwrap();
            w.push("", &decode(1_700_000_000, 14.074e6, -10.0, "CQ K1ABC FN42"));
        }
        assert_eq!(Reader::open(&p).unwrap().span().unwrap().2, 1);
        let _ = std::fs::remove_file(&p);
    }

    /// The JSON the GUI sends (camelCase, nulls for unset fields) is the query.
    #[test]
    fn query_from_the_gui_json() {
        let q: Query = serde_json::from_str(
            r#"{"since":0,"until":9,"me":"PM95","call":"","grid":"","text":"","bands":[],"modes":["FT8"],
                "snrMin":null,"snrMax":-20,"kmMin":null,"kmMax":null,"bearingFrom":300,"bearingTo":60,"cq":null}"#,
        )
        .unwrap();
        assert_eq!((q.snr_min, q.snr_max), (None, Some(-20)));
        assert_eq!((q.bearing_from, q.bearing_to), (Some(300.0), Some(60.0)));
        assert_eq!(q.modes, vec!["FT8".to_string()]);
    }

    /// Two servers in different places: a bearing is read from the place that
    /// heard the signal, and a query can name the server.
    #[test]
    fn bearings_come_from_the_server_that_heard() {
        let p = db();
        let servers = [
            ("tokyo".to_string(), "PM95".to_string()),
            ("boston".to_string(), "FN42".to_string()),
        ];
        let mut w = Writer::open(&p, &servers).unwrap();
        let h = 1_700_000_000 / 3600 * 3600;
        // JA1XYZ (PM95) heard from both; from Tokyo it is local, from Boston
        // it is to the north-west across the pole.
        w.push("tokyo", &decode(h, 14.074e6, -5.0, "CQ JA1XYZ PM95"));
        w.push("boston", &decode(h + 15, 14.074e6, -15.0, "CQ JA1XYZ PM95"));
        w.drop_flush();
        let r = Reader::open(&p).unwrap();
        let q = |f: &dyn Fn(&mut Query)| {
            let mut x = Query {
                since: h,
                until: h + 600,
                ..Query::default()
            };
            f(&mut x);
            r.summary(&x).unwrap().decodes
        };
        assert_eq!(q(&|_| {}), 2);
        assert_eq!(q(&|x| x.servers = vec!["boston".into()]), 1);
        // Within 500 km of where it was heard: only Tokyo's.
        assert_eq!(q(&|x| x.km_max = Some(500.0)), 1);
        assert_eq!(q(&|x| x.km_min = Some(5_000.0)), 1);
        let spots = r
            .decodes(
                &Query {
                    since: h,
                    until: h + 600,
                    ..Query::default()
                },
                10,
            )
            .unwrap();
        let by = |name: &str| spots.iter().find(|s| s.server == name).unwrap();
        assert!(by("tokyo").km.unwrap() < 100.0);
        assert!(by("boston").km.unwrap() > 9_000.0);
        let st = r
            .stations(
                &Query {
                    since: h,
                    until: h + 600,
                    ..Query::default()
                },
                10,
            )
            .unwrap();
        assert_eq!(st[0].servers.split(',').count(), 2);
        drop(w);
        let _ = std::fs::remove_file(&p);
    }

    /// A file written before there were several servers gains the column.
    #[test]
    fn an_old_file_is_upgraded() {
        let p = db();
        {
            let c = Connection::open(&p).unwrap();
            c.execute_batch(
                "CREATE TABLE decodes (id INTEGER PRIMARY KEY, t INTEGER NOT NULL, mode TEXT NOT NULL,
                  band TEXT NOT NULL, dial_hz INTEGER NOT NULL, freq_hz INTEGER NOT NULL, snr INTEGER NOT NULL,
                  dt REAL NOT NULL, call TEXT, grid TEXT, cq TEXT, text TEXT NOT NULL);
                 INSERT INTO decodes (t, mode, band, dial_hz, freq_hz, snr, dt, text)
                  VALUES (1700000000, 'FT8', '20m', 14074000, 14075500, -5, 0.1, 'CQ K1ABC FN42');",
            )
            .unwrap();
        }
        let mut w = Writer::open(&p, &[]).unwrap();
        w.push("a", &decode(1_700_000_015, 14.074e6, -7.0, "CQ K1ABC FN42"));
        w.drop_flush();
        let r = Reader::open(&p).unwrap();
        assert_eq!(r.span().unwrap().2, 2);
        let q = Query {
            since: 0,
            until: 2_000_000_000,
            servers: vec![String::new()],
            ..Query::default()
        };
        assert_eq!(
            r.summary(&q).unwrap().decodes,
            1,
            "the old row has no server"
        );
        drop(w);
        let _ = std::fs::remove_file(&p);
    }

    fn with_detail(mut d: Decode, key: &[u8], update: bool) -> Decode {
        d.detail = crate::DecodeDetail {
            key: key.to_vec(),
            sync_score: 2.5,
            hard_errors: 3,
            hash_resolved: update,
            copied_last_tx: false,
        };
        d.update = update;
        d
    }

    #[test]
    fn a_key_is_packed_most_significant_bit_first() {
        assert_eq!(pack_key(&[]), None);
        assert_eq!(pack_key(&[1, 0, 1]), Some(vec![0b1010_0000]));
        // 91 bits is 12 bytes; the last has 3 bits in it.
        let k: Vec<u8> = (0..91).map(|i| (i % 2) as u8).collect();
        let p = pack_key(&k).unwrap();
        assert_eq!(p.len(), 12);
        assert_eq!(p[0], 0b0101_0101);
        assert_eq!(p[11], 0b0100_0000);
    }

    #[test]
    fn the_rows_detail_is_stored_and_read_back() {
        let p = db();
        let mut w = Writer::open(&p, &[]).unwrap();
        let h = 1_700_000_000 / 3600 * 3600;
        w.push(
            "",
            &with_detail(
                decode(h, 14.074e6, -10.0, "CQ K1ABC FN42"),
                &[1, 0, 1],
                false,
            ),
        );
        w.push("", &decode(h + 15, 14.074e6, -5.0, "CQ W9XYZ EN34"));
        w.flush();
        let r = Reader::open(&p).unwrap();
        let spots = r.decodes(&q(h), 10).unwrap();
        let k = spots.iter().find(|s| s.text.contains("K1ABC")).unwrap();
        assert_eq!(
            (k.sync, k.hard_errors, k.resolved),
            (Some(2.5), Some(3), Some(false))
        );
        // A row given no detail has the defaults, not NULL.
        let w9 = spots.iter().find(|s| s.text.contains("W9XYZ")).unwrap();
        assert_eq!((w9.sync, w9.hard_errors), (Some(0.0), Some(0)));
        drop(w);
        let _ = std::fs::remove_file(&p);
    }

    /// A row delivered as `<...>` and then resolved is one row in the file, with
    /// the resolved text, whether the update arrives before the first write or after.
    #[test]
    fn an_update_replaces_the_row_it_names() {
        let p = db();
        let mut w = Writer::open(&p, &[]).unwrap();
        let h = 1_700_000_000 / 3600 * 3600;
        let first = |t: &str, key: &[u8]| with_detail(decode(h, 14.074e6, -10.0, t), key, false);
        let better = |t: &str, key: &[u8]| with_detail(decode(h, 14.074e6, -10.0, t), key, true);

        // Written already.
        w.push("", &first("CQ <...> PM95", &[1, 0, 1]));
        w.flush();
        w.push("", &better("CQ <JA1ABC> PM95", &[1, 0, 1]));
        // Still waiting.
        w.push("", &first("K1ABC <...> -07", &[0, 1, 1]));
        w.push("", &better("K1ABC <W9XYZ> -07", &[0, 1, 1]));
        // Another message at the same place is not touched.
        w.push("", &first("CQ <...> FN42", &[1, 1, 1]));
        w.flush();

        let r = Reader::open(&p).unwrap();
        let mut texts: Vec<_> = r
            .decodes(&q(h), 10)
            .unwrap()
            .into_iter()
            .map(|s| (s.text, s.resolved))
            .collect();
        texts.sort();
        assert_eq!(
            texts,
            vec![
                ("CQ <...> FN42".to_string(), Some(false)),
                ("CQ <JA1ABC> PM95".to_string(), Some(true)),
                ("K1ABC <W9XYZ> -07".to_string(), Some(true)),
            ]
        );
        drop(w);
        let _ = std::fs::remove_file(&p);
    }

    /// An analysis opened on a file from before the detail was kept still reads
    /// it, with nothing for the new columns.
    #[test]
    fn a_file_without_the_detail_columns_is_read() {
        let p = db();
        {
            let c = Connection::open(&p).unwrap();
            c.execute_batch(
                "CREATE TABLE decodes (id INTEGER PRIMARY KEY, t INTEGER NOT NULL, mode TEXT NOT NULL,
                  band TEXT NOT NULL, dial_hz INTEGER NOT NULL, freq_hz INTEGER NOT NULL, snr INTEGER NOT NULL,
                  dt REAL NOT NULL, call TEXT, grid TEXT, cq TEXT, text TEXT NOT NULL,
                  server TEXT NOT NULL DEFAULT '');
                 CREATE TABLE stations (call TEXT PRIMARY KEY, grid TEXT NOT NULL, seen INTEGER NOT NULL);
                 CREATE TABLE servers (name TEXT PRIMARY KEY, grid TEXT NOT NULL);
                 INSERT INTO decodes (t, mode, band, dial_hz, freq_hz, snr, dt, text)
                  VALUES (1700000000, 'FT8', '20m', 14074000, 14075500, -5, 0.1, 'CQ K1ABC FN42');",
            )
            .unwrap();
        }
        let r = Reader::open(&p).unwrap();
        let q = Query {
            since: 0,
            until: 2_000_000_000,
            ..Query::default()
        };
        let spots = r.decodes(&q, 10).unwrap();
        assert_eq!(spots.len(), 1);
        assert_eq!(
            (spots[0].sync, spots[0].hard_errors, spots[0].resolved),
            (None, None, None)
        );
        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn looking_after_the_file() {
        let p = db();
        let mut w = Writer::open(&p, &[("a".to_string(), "PM95".to_string())]).unwrap();
        let h = 1_700_000_000 / 3600 * 3600;
        w.push("a", &decode(h, 14.074e6, -5.0, "CQ K1ABC FN42"));
        w.push("a", &decode(h + 7200, 14.074e6, -7.0, "CQ \"W9XYZ\", EN34"));
        w.push("b", &decode(h + 7215, 14.074e6, -9.0, "CQ JA1XYZ PM95"));
        w.flush();
        drop(w);

        let i = info(&p).unwrap();
        assert_eq!(
            (i.decodes, i.stations),
            (3, 2),
            "the quoted call is not a call"
        );
        assert_eq!(i.servers.len(), 2);
        assert_eq!(count_before(&p, h + 3600, None).unwrap(), 1);
        assert_eq!(count_before(&p, h + 3600, Some("b")).unwrap(), 0);

        // A CSV of the query, with a comma and quotes in a message.
        let dest = p.with_extension("csv");
        let q = Query {
            since: h,
            until: h + 8000,
            me: "PM95".into(),
            ..Query::default()
        };
        assert_eq!(export_csv(&p, &q, &dest).unwrap(), 3);
        let text = std::fs::read_to_string(&dest).unwrap();
        assert!(text.starts_with("utc,server,call"), "{text}");
        assert!(text.contains("\"CQ \"\"W9XYZ\"\", EN34\""), "{text}");
        assert!(text.contains(",K1ABC,FN42,20m,FT8,"), "{text}");
        let _ = std::fs::remove_file(&dest);

        // A copy while the file is in use, then clearing out and compacting.
        let copy = p.with_extension("copy.db");
        let _ = std::fs::remove_file(&copy);
        backup(&p, &copy).unwrap();
        assert!(
            backup(&p, &copy).is_err(),
            "never over a file that is there"
        );
        assert_eq!(Reader::open(&copy).unwrap().span().unwrap().2, 3);
        assert_eq!(delete_before(&p, h + 3600, None).unwrap(), 1);
        assert_eq!(delete_before(&p, h + 8000, Some("b")).unwrap(), 1);
        vacuum(&p).unwrap();
        assert_eq!(info(&p).unwrap().decodes, 1);
        // Decodes with no server name become a named server's.
        assert_eq!(rename_server(&p, "a", "x").unwrap(), 1);
        assert_eq!(info(&p).unwrap().servers, vec![("x".to_string(), 1)]);
        assert_eq!(
            Reader::open(&copy).unwrap().span().unwrap().2,
            3,
            "the copy is untouched"
        );
        for ext in ["", "-wal", "-shm"] {
            let _ = std::fs::remove_file(format!("{}{ext}", copy.display()));
        }
        let _ = std::fs::remove_file(&p);
    }
}
