// SPDX-License-Identifier: GPL-3.0-or-later
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

const SCHEMA: &str = "
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
    text    TEXT    NOT NULL
);
CREATE INDEX IF NOT EXISTS decodes_t      ON decodes (t);
CREATE INDEX IF NOT EXISTS decodes_band_t ON decodes (band, t);
CREATE INDEX IF NOT EXISTS decodes_call_t ON decodes (call, t) WHERE call IS NOT NULL;
CREATE TABLE IF NOT EXISTS stations (
    call TEXT PRIMARY KEY,
    grid TEXT NOT NULL,
    seen INTEGER NOT NULL       -- UTC seconds of the message that gave it
) WITHOUT ROWID;
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
    mode: &'static str,
    band: &'static str,
    dial_hz: i64,
    freq_hz: i64,
    snr: i64,
    dt: f64,
    call: Option<String>,
    grid: Option<String>,
    text: String,
}

/// Rows buffered at most this long, or this many, before a transaction.
const FLUSH_EVERY: Duration = Duration::from_secs(2);
const FLUSH_ROWS: usize = 200;

impl Writer {
    pub fn open(path: &Path) -> rusqlite::Result<Writer> {
        let conn = open(
            path,
            OpenFlags::SQLITE_OPEN_READ_WRITE | OpenFlags::SQLITE_OPEN_CREATE,
        )?;
        conn.pragma_update(None, "journal_mode", "WAL")?;
        conn.pragma_update(None, "synchronous", "NORMAL")?;
        conn.execute_batch(SCHEMA)?;
        Ok(Writer {
            conn,
            pending: Vec::new(),
            since: Instant::now(),
        })
    }

    pub fn push(&mut self, d: &Decode) {
        let name = modes::mode_name(d.mode);
        let (call, grid) = spot::sender(name, &d.text);
        self.pending.push(Row {
            t: d.slot_utc_ns.map_or(0, |ns| ns.div_euclid(1_000_000_000)),
            mode: name,
            band: band_of(d.dial_hz),
            dial_hz: d.dial_hz.round() as i64,
            freq_hz: d.freq_hz.round() as i64,
            snr: d.snr_db.round() as i64,
            dt: f64::from(d.dt_s),
            call,
            grid,
            text: d.text.clone(),
        });
        if self.pending.len() >= FLUSH_ROWS || self.since.elapsed() >= FLUSH_EVERY {
            self.flush();
        }
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
                "INSERT INTO decodes (t, mode, band, dial_hz, freq_hz, snr, dt, call, grid, text)
                 VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10)",
            )?;
            let mut st = tx.prepare_cached(
                "INSERT INTO stations (call, grid, seen) VALUES (?1, ?2, ?3)
                 ON CONFLICT(call) DO UPDATE SET grid = excluded.grid, seen = excluded.seen
                 WHERE excluded.seen >= seen",
            )?;
            for r in rows {
                ins.execute(params![
                    r.t, r.mode, r.band, r.dial_hz, r.freq_hz, r.snr, r.dt, r.call, r.grid, r.text
                ])?;
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

/// A station heard in a period.
#[derive(Clone, Debug, PartialEq)]
pub struct Heard {
    pub call: String,
    pub band: String,
    pub grid: String,
    pub count: i64,
    pub best_snr: i64,
    pub last: i64,
    /// Degrees from north and km, from the observer's locator; `None` when
    /// none was given.
    pub bearing: Option<f64>,
    pub km: Option<f64>,
}

/// Decodes of one hour on one band.
#[derive(Clone, Debug, PartialEq)]
pub struct Activity {
    /// UTC hour since the epoch (`t / 3600`).
    pub hour: i64,
    pub band: String,
    pub stations: i64,
    pub decodes: i64,
}

/// A station's hours on the air.
#[derive(Clone, Debug, PartialEq)]
pub struct Presence {
    pub hour: i64,
    pub band: String,
    pub count: i64,
    pub best_snr: i64,
}

/// Read-only queries; any number may run beside the writer.
pub struct Reader {
    conn: Connection,
}

impl Reader {
    pub fn open(path: &Path) -> rusqlite::Result<Reader> {
        Ok(Reader {
            conn: open(path, OpenFlags::SQLITE_OPEN_READ_ONLY)?,
        })
    }

    /// Stations heard in `[since, until]` (UTC seconds) on `band` (all when
    /// `None`) with a known locator, with their bearing from `me`.
    pub fn heard(
        &self,
        me: Option<&str>,
        band: Option<&str>,
        since: i64,
        until: i64,
    ) -> rusqlite::Result<Vec<Heard>> {
        let mut q = self.conn.prepare_cached(
            "SELECT d.call, d.band, s.grid, COUNT(*), MAX(d.snr), MAX(d.t)
             FROM decodes d JOIN stations s ON s.call = d.call
             WHERE d.t BETWEEN ?1 AND ?2 AND (?3 IS NULL OR d.band = ?3)
             GROUP BY d.call, d.band",
        )?;
        let from = me.and_then(geo::grid_center);
        let rows = q.query_map(params![since, until, band], |r| {
            let grid: String = r.get(2)?;
            let bd = from
                .zip(geo::grid_center(&grid))
                .map(|(a, b)| geo::bearing_distance(a, b));
            Ok(Heard {
                call: r.get(0)?,
                band: r.get(1)?,
                grid,
                count: r.get(3)?,
                best_snr: r.get(4)?,
                last: r.get(5)?,
                bearing: bd.map(|x| x.0),
                km: bd.map(|x| x.1),
            })
        })?;
        rows.collect()
    }

    /// Distinct senders and decodes per hour and band: the band's state over
    /// time.
    pub fn activity(&self, since: i64, until: i64) -> rusqlite::Result<Vec<Activity>> {
        let mut q = self.conn.prepare_cached(
            "SELECT t / 3600, band, COUNT(DISTINCT call), COUNT(*)
             FROM decodes WHERE t BETWEEN ?1 AND ?2
             GROUP BY t / 3600, band ORDER BY 1",
        )?;
        let rows = q.query_map(params![since, until], |r| {
            Ok(Activity {
                hour: r.get(0)?,
                band: r.get(1)?,
                stations: r.get(2)?,
                decodes: r.get(3)?,
            })
        })?;
        rows.collect()
    }

    /// The hours `call` was heard in `[since, until]`.
    pub fn presence(&self, call: &str, since: i64, until: i64) -> rusqlite::Result<Vec<Presence>> {
        let mut q = self.conn.prepare_cached(
            "SELECT t / 3600, band, COUNT(*), MAX(snr) FROM decodes
             WHERE call = ?1 AND t BETWEEN ?2 AND ?3
             GROUP BY t / 3600, band ORDER BY 1",
        )?;
        let rows = q.query_map(params![call, since, until], |r| {
            Ok(Presence {
                hour: r.get(0)?,
                band: r.get(1)?,
                count: r.get(2)?,
                best_snr: r.get(3)?,
            })
        })?;
        rows.collect()
    }

    /// Calls starting with `prefix`, the most heard first. A range scan on
    /// the `(call, t)` index, so it stays fast on millions of rows.
    pub fn calls(&self, prefix: &str, limit: usize) -> rusqlite::Result<Vec<(String, i64)>> {
        let lo = prefix.to_ascii_uppercase();
        let hi = format!("{lo}\u{10ffff}");
        let mut q = self.conn.prepare_cached(
            "SELECT call, COUNT(*) FROM decodes WHERE call >= ?1 AND call < ?2
             GROUP BY call ORDER BY 2 DESC LIMIT ?3",
        )?;
        let rows = q.query_map(params![lo, hi, limit as i64], |r| {
            Ok((r.get(0)?, r.get(1)?))
        })?;
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

    #[test]
    fn stored_and_queried_while_the_writer_is_open() {
        let p = db();
        let mut w = Writer::open(&p).unwrap();
        let h = 1_700_000_000 / 3600 * 3600;
        w.push(&decode(h, 14.074e6, -10.0, "CQ K1ABC FN42"));
        w.push(&decode(h + 15, 14.074e6, -5.0, "JA1XYZ K1ABC -07"));
        w.push(&decode(h + 3600, 14.074e6, -12.0, "CQ K1ABC"));
        w.push(&decode(h + 30, 7.074e6, -20.0, "CQ W9XYZ EN34"));
        w.flush();
        // The writer still holds the file open: WAL lets a reader in.
        let r = Reader::open(&p).unwrap();
        assert_eq!(r.span().unwrap().2, 4);

        // From Tokyo: K1ABC (FN42) lies north-east, W9XYZ (EN34) too.
        let heard = r.heard(Some("PM95"), Some("20m"), h, h + 7200).unwrap();
        assert_eq!(heard.len(), 1);
        let k = &heard[0];
        assert_eq!(
            (k.call.as_str(), k.grid.as_str(), k.count, k.best_snr),
            ("K1ABC", "FN42", 3, -5)
        );
        assert!((0.0..90.0).contains(&k.bearing.unwrap()), "{:?}", k.bearing);
        assert_eq!(r.heard(None, None, h, h + 7200).unwrap().len(), 2);

        // K1ABC's later message without a locator still counts, and shows
        // in two separate hours.
        let pr = r.presence("K1ABC", h, h + 7200).unwrap();
        assert_eq!(pr.len(), 2);
        assert_eq!(pr[0].count, 2);

        let act = r.activity(h, h + 7200).unwrap();
        assert!(act.iter().any(|a| a.band == "40m" && a.stations == 1));
        assert_eq!(r.calls("K1", 5).unwrap(), vec![("K1ABC".to_string(), 3)]);
        drop(w);
        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn unflushed_rows_are_written_on_drop() {
        let p = db();
        {
            let mut w = Writer::open(&p).unwrap();
            w.push(&decode(1_700_000_000, 14.074e6, -10.0, "CQ K1ABC FN42"));
        }
        assert_eq!(Reader::open(&p).unwrap().span().unwrap().2, 1);
        let _ = std::fs::remove_file(&p);
    }
}
