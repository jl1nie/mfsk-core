//! Web-based settings form (`esp_idf_svc::http::server::EspHttpServer`).
//!
//! Settings that have no place on the panel (the NTP server) are edited
//! from a browser on the same WiFi network instead of a device-local
//! menu; the panel's CONFIG page carries the ones an operator changes in the
//! field. The page also serves the logs (`ALL.TXT` and friends). `GET /` renders the current `Settings` as
//! a plain HTML form; `POST /save` validates, then persists to NVS —
//! **validate-then-write**, so a rejected submission never leaves the
//! stored settings partially updated.
//!
//! No new Cargo dependency: `esp_idf_svc::http::server` and the
//! `Method`/`Headers`/`Read`/`Write` traits it needs are already
//! reachable through the existing `esp-idf-svc` dependency (verified
//! against the checked-out crate source, same as `ntp.rs`).
//!
//! **No authentication.** Anyone on the same LAN who can reach the
//! device's IP can change every setting here and read the logs.
//! Deliberately out of scope — see the design memory for the tradeoff.
//!
//! Percent-decoding is hand-rolled rather than pulling in a
//! form/urlencoding crate: every field this form accepts (an NTP host
//! name) is ASCII, so a byte-wise decode is enough and keeps this
//! module dependency-free like the rest of the crate's HTTP/URL code.

use std::fmt::Write as _;
use std::sync::{Arc, Mutex};

use embedded_svc::http::Headers;
use esp_idf_svc::http::server::{Configuration as HttpConfiguration, EspHttpServer};
use esp_idf_svc::http::Method;
use esp_idf_svc::io::{Read, Write};
use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use crate::settings::{self, Settings};

/// Body size cap for `POST /save`. Every field is short (the longest,
/// `ntp_server`, is capacity-48 in [`Settings`]); this is generous headroom
/// for urlencoding overhead, not a real form size.
const MAX_BODY_LEN: usize = 1024;

/// Stack the HTTP server task runs on. `EspHttpServer::new`'s own
/// default (6144) is sized for the upstream JSON example's minimal
/// handlers; this crate's handlers additionally lock a `Mutex`, build
/// a multi-KB `String` for the form page, and call into
/// [`settings::load`]/[`settings::save`], so this follows that same
/// example's precedent of raising it (`10240` there, for JSON
/// parsing) rather than assuming the default is enough.
const HTTP_SERVER_STACK_SIZE: usize = 10240;

/// **2026-08-15, real-hardware finding**: `Configuration::default()`'s
/// own `task_caps` is `MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT` —
/// internal DRAM only, no PSRAM. On `m5stack-cores3-app`'s `wspr-app`
/// binary that consistently failed (`ESP_ERR_HTTPD_TASK` — the
/// underlying `xTaskCreate` for this 10 KiB stack couldn't find a
/// large-enough contiguous internal-DRAM block): that binary's own
/// scan/DDC/display task stacks (108 KiB combined) plus WiFi's runtime
/// buffer pools leave internal DRAM tight by the time this runs — see
/// `wspr-app`'s own design memory for the `log_heap()` trail
/// (171→99→86→62 KB before WiFi even starts). Requesting
/// `MALLOC_CAP_SPIRAM` instead moves this task's stack to PSRAM (8 MB,
/// comfortably free), sidestepping that contention entirely — an
/// officially-supported ESP-IDF S3 feature for non-ISR task stacks
/// (`CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY`, set in
/// `m5stack-cores3-app/sdkconfig.defaults`; this crate is also
/// consumed by `m5stack-s3-app`/`m5stack-core2-app`, whose own
/// sdkconfigs don't set that Kconfig — `EspHttpServer::new` would
/// simply fail there if it ever became a real constraint, same
/// graceful-degradation path this crate's callers already handle).
const HTTP_SERVER_TASK_CAPS: u32 =
    esp_idf_svc::sys::MALLOC_CAP_SPIRAM | esp_idf_svc::sys::MALLOC_CAP_8BIT;

/// Start the config web server. Bound to the shared NVS handle: the
/// `GET /` and `POST /save` handlers both lock it independently per
/// request, not once for the server's lifetime, so a slow request
/// can't stall the app's own per-slot NVS reads for longer than one
/// handler call.
///
/// Returned server must be kept alive by the caller (dropping it
/// stops listening, same as every other `Esp*` service in this
/// crate).
/// Files the board serves for download (`GET /<name>`), and how to read
/// them.
///
/// `read` is the board's, not a `std::fs` call here, because this
/// server's task runs on a PSRAM stack ([`HTTP_SERVER_TASK_CAPS`]) and
/// a flash read from a PSRAM stack aborts the board
/// (`esp_task_stack_is_sane_cache_disabled`). The CoreS3 passes its
/// storage task's `read_chunk`, which does the read on an internal
/// stack and hands the bytes back.
#[derive(Clone, Copy)]
pub struct FileSource {
    pub names: &'static [&'static str],
    /// Up to `len` bytes of `name` from `offset`; `Some(empty)` at the
    /// end, `None` if absent.
    pub read: fn(name: &str, offset: u64, len: usize) -> Option<Vec<u8>>,
}

/// Bytes per read while streaming a download: one request to the
/// storage task per chunk, so large enough to keep a 2 MB `all.txt`
/// to a few hundred round trips, small enough to stay one PSRAM
/// allocation.
const DOWNLOAD_CHUNK: usize = 8192;

pub fn start(
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
    files: Option<FileSource>,
) -> anyhow::Result<EspHttpServer<'static>> {
    let mut server = EspHttpServer::new(&HttpConfiguration {
        stack_size: HTTP_SERVER_STACK_SIZE,
        task_caps: HTTP_SERVER_TASK_CAPS,
        ..Default::default()
    })?;

    let nvs_get = nvs.clone();
    server.fn_handler::<anyhow::Error, _>("/", Method::Get, move |req| {
        let current = {
            let guard = nvs_get.lock().expect("settings NVS mutex poisoned");
            settings::load(&guard)
        };
        let page = render_form(&current, None, files);
        req.into_ok_response()?.write_all(page.as_bytes())?;
        Ok(())
    })?;

    for &name in files.map_or(&[][..], |f| f.names) {
        let read = files.map(|f| f.read).expect("names imply a source");
        server.fn_handler::<anyhow::Error, _>(&format!("/{name}"), Method::Get, move |req| {
            let Some(first) = read(name, 0, DOWNLOAD_CHUNK) else {
                req.into_status_response(404)?
                    .write_all(b"no such file yet")?;
                return Ok(());
            };
            let disposition = format!("attachment; filename=\"{name}\"");
            let mut resp = req.into_response(
                200,
                None,
                &[
                    ("Content-Type", "text/plain; charset=utf-8"),
                    ("Content-Disposition", disposition.as_str()),
                ],
            )?;
            let mut offset = first.len() as u64;
            let mut chunk = first;
            while !chunk.is_empty() {
                resp.write_all(&chunk)?;
                chunk = match read(name, offset, DOWNLOAD_CHUNK) {
                    Some(c) => c,
                    None => break,
                };
                offset += chunk.len() as u64;
            }
            Ok(())
        })?;
    }

    server.fn_handler::<anyhow::Error, _>("/save", Method::Post, move |mut req| {
        let len = req.content_len().unwrap_or(0) as usize;
        if len > MAX_BODY_LEN {
            req.into_status_response(413)?
                .write_all(b"form body too large")?;
            return Ok(());
        }

        let mut buf = vec![0u8; len];
        req.read_exact(&mut buf)?;
        let body = core::str::from_utf8(&buf).unwrap_or("");
        let fields = parse_form(body);

        let current = {
            let guard = nvs.lock().expect("settings NVS mutex poisoned");
            settings::load(&guard)
        };

        match validate(&fields, &current) {
            Ok(new_settings) => {
                {
                    let guard = nvs.lock().expect("settings NVS mutex poisoned");
                    settings::save(&guard, &new_settings)?;
                }
                log::info!(
                    "http_config: settings saved (ntp={} server='{}')",
                    new_settings.ntp_enabled,
                    new_settings.ntp_server
                );
                let page = render_form(&new_settings, Some("Saved."), files);
                req.into_ok_response()?.write_all(page.as_bytes())?;
            }
            Err(msg) => {
                log::warn!("http_config: rejected save — {msg}");
                let page = render_form(&current, Some(&format!("Rejected: {msg}")), files);
                req.into_status_response(400)?.write_all(page.as_bytes())?;
            }
        }
        Ok(())
    })?;

    Ok(server)
}

// ---- form rendering ----------------------------------------------

fn html_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            _ => out.push(c),
        }
    }
    out
}

fn render_form(s: &Settings, banner: Option<&str>, files: Option<FileSource>) -> String {
    let mut html = String::with_capacity(2048);
    let _ = write!(
        html,
        "<!doctype html><html><head><meta charset=\"utf-8\">\
         <title>Settings</title></head><body>\
         <h1>Settings</h1>"
    );
    if let Some(b) = banner {
        let _ = write!(html, "<p><strong>{}</strong></p>", html_escape(b));
    }
    let _ = write!(html, "<form method=\"POST\" action=\"/save\">");

    let checked = if s.ntp_enabled { " checked" } else { "" };
    let _ = write!(
        html,
        "<p><label><input type=\"checkbox\" name=\"ntp_enabled\"{checked}> \
         Sync clock via NTP (ignored while the panel's TIME row is AIR DT, \
         which never starts NTP)</label></p>"
    );
    let _ = write!(
        html,
        "<p><label>NTP server<br>\
         <input name=\"ntp_server\" maxlength=\"48\" value=\"{}\"></label></p>",
        html_escape(&s.ntp_server)
    );

    let _ = write!(html, "<p><button type=\"submit\">Save</button></p></form>");
    if let Some(f) = files {
        let _ = write!(html, "<h2>Logs</h2><ul>");
        for name in f.names {
            let _ = write!(html, "<li><a href=\"/{name}\">{name}</a></li>");
        }
        let _ = write!(html, "</ul>");
    }
    let _ = write!(html, "</body></html>");
    html
}

// ---- form parsing --------------------------------------------------

/// Every field the form submits, still as raw (percent-decoded) text
/// — [`validate`] is what turns this into a [`Settings`] or an error.
#[derive(Default)]
struct FormFields {
    ntp_enabled: bool,
    ntp_server: Option<String>,
}

/// Percent-decode one `x-www-form-urlencoded` value.
///
/// Byte-wise, not UTF-8-aware: every value this form produces is
/// ASCII (an NTP host name), so treating each decoded
/// byte as one `char` is exact for the inputs this module actually
/// receives rather than a general-purpose decoder.
fn percent_decode(v: &str) -> String {
    let bytes = v.as_bytes();
    let mut out = String::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'+' => {
                out.push(' ');
                i += 1;
            }
            b'%' if i + 2 < bytes.len() => {
                let hex = core::str::from_utf8(&bytes[i + 1..i + 3]).unwrap_or("");
                match u8::from_str_radix(hex, 16) {
                    Ok(byte) => {
                        out.push(byte as char);
                        i += 3;
                    }
                    Err(_) => {
                        out.push('%');
                        i += 1;
                    }
                }
            }
            b => {
                out.push(b as char);
                i += 1;
            }
        }
    }
    out
}

fn parse_form(body: &str) -> FormFields {
    let mut fields = FormFields::default();
    for pair in body.split('&') {
        if pair.is_empty() {
            continue;
        }
        let (key, raw_value) = match pair.split_once('=') {
            Some((k, v)) => (k, v),
            None => (pair, ""),
        };
        let value = percent_decode(raw_value);
        match key {
            // A checked HTML checkbox sends `ntp_enabled=on`; an
            // unchecked one omits the field entirely — there is no
            // "false" value to parse, only presence vs. absence.
            "ntp_enabled" => fields.ntp_enabled = true,
            "ntp_server" => fields.ntp_server = Some(value),
            _ => {}
        }
    }
    fields
}

// ---- validation ------------------------------------------------------

/// Build a candidate [`Settings`] from submitted fields, falling back
/// to `current`'s value for anything the form omitted (defensive —
/// the form this module renders always submits every field except
/// the checkbox), or failing with a human-readable reason.
///
/// Every check runs before anything is returned — the caller only
/// writes to NVS on `Ok`, so a rejected submission never partially
/// overwrites the stored settings.
fn validate(fields: &FormFields, current: &Settings) -> Result<Settings, String> {
    let ntp_enabled = fields.ntp_enabled;
    let ntp_server_raw = fields
        .ntp_server
        .as_deref()
        .unwrap_or(&current.ntp_server)
        .trim();
    if ntp_enabled && ntp_server_raw.is_empty() {
        return Err("NTP server is required while NTP sync is enabled".to_string());
    }
    let ntp_server = heapless::String::<48>::try_from(ntp_server_raw)
        .map_err(|_| "NTP server too long (max 48 chars)".to_string())?;

    Ok(Settings {
        ntp_enabled,
        ntp_server,
    })
}
