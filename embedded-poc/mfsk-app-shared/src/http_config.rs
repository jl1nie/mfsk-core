//! Web-based settings form (`esp_idf_svc::http::server::EspHttpServer`).
//!
//! CoreS3 has no on-device input (no touch driver, no buttons — see
//! this crate's own `wspr_bands`/`settings` doc comments and the
//! design memory this app was planned from), so every setting the app
//! exposes is edited from a browser on the same WiFi network instead
//! of a device-local menu. `GET /` renders the current `Settings` as
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
//! device's IP can change every setting here, including which
//! wsprnet endpoint spots go to. Deliberately out of scope for this
//! round — see the design memory for the tradeoff.
//!
//! Percent-decoding matches [`crate::wsprnet`]'s own hand-rolled
//! encoder rather than pulling in a form/urlencoding crate: every
//! field this form accepts (callsign, grid, decimal numbers, a
//! wsprnet URL) is ASCII, so a byte-wise decode is enough and keeps
//! this module dependency-free like the rest of the crate's HTTP/URL
//! code.

use std::fmt::Write as _;
use std::sync::{Arc, Mutex};

use embedded_svc::http::Headers;
use esp_idf_svc::http::Method;
use esp_idf_svc::http::server::{Configuration as HttpConfiguration, EspHttpServer};
use esp_idf_svc::io::{Read, Write};
use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use crate::settings::{self, Settings};
use crate::wspr_bands::WSPR_BANDS;

/// Body size cap for `POST /save`. Every field is short (the longest,
/// `wsprnet_spot_config`, is capacity-64 in [`Settings`]); this is
/// generous headroom for urlencoding overhead, not a real form size.
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
const HTTP_SERVER_TASK_CAPS: u32 = esp_idf_svc::sys::MALLOC_CAP_SPIRAM | esp_idf_svc::sys::MALLOC_CAP_8BIT;

/// Start the config web server. Bound to the shared NVS handle: the
/// `GET /` and `POST /save` handlers both lock it independently per
/// request, not once for the server's lifetime, so a slow request
/// can't stall the app's own per-slot NVS reads for longer than one
/// handler call.
///
/// Returned server must be kept alive by the caller (dropping it
/// stops listening, same as every other `Esp*` service in this
/// crate).
pub fn start(nvs: Arc<Mutex<EspNvs<NvsDefault>>>) -> anyhow::Result<EspHttpServer<'static>> {
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
        let page = render_form(&current, None);
        req.into_ok_response()?.write_all(page.as_bytes())?;
        Ok(())
    })?;

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
                log::info!("http_config: settings saved (call='{}')", new_settings.call);
                let page = render_form(&new_settings, Some("Saved."));
                req.into_ok_response()?.write_all(page.as_bytes())?;
            }
            Err(msg) => {
                log::warn!("http_config: rejected save — {msg}");
                let page = render_form(&current, Some(&format!("Rejected: {msg}")));
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

fn render_form(s: &Settings, banner: Option<&str>) -> String {
    let mut html = String::with_capacity(2048);
    let _ = write!(
        html,
        "<!doctype html><html><head><meta charset=\"utf-8\">\
         <title>WSPR receiver settings</title></head><body>\
         <h1>WSPR receiver settings</h1>"
    );
    if let Some(b) = banner {
        let _ = write!(html, "<p><strong>{}</strong></p>", html_escape(b));
    }
    let _ = write!(html, "<form method=\"POST\" action=\"/save\">");

    let _ = write!(
        html,
        "<p><label>Callsign<br><input name=\"call\" maxlength=\"16\" value=\"{}\"></label></p>",
        html_escape(&s.call)
    );
    let _ = write!(
        html,
        "<p><label>Grid locator (4 or 6 chars)<br>\
         <input name=\"grid\" maxlength=\"8\" value=\"{}\"></label></p>",
        html_escape(&s.grid)
    );
    let _ = write!(
        html,
        "<p><label>TX power, dBm (0-60, storage only — this receiver cannot transmit)<br>\
         <input name=\"tx_power_dbm\" type=\"number\" min=\"0\" max=\"60\" value=\"{}\"></label></p>",
        s.tx_power_dbm
    );

    let _ = write!(html, "<p><label>Band<br><select name=\"band\">");
    for (i, band) in WSPR_BANDS.iter().enumerate() {
        let selected = if i as u8 == s.band_idx { " selected" } else { "" };
        let _ = write!(
            html,
            "<option value=\"{i}\"{selected}>{} ({:.4} MHz)</option>",
            band.label, band.dial_mhz
        );
    }
    let _ = write!(html, "</select></label></p>");

    let _ = write!(
        html,
        "<p><label>wsprnet reporting — <code>off</code>, <code>dummy</code> \
         (log only), or a POST URL<br>\
         <input name=\"wsprnet\" maxlength=\"80\" value=\"{}\"></label></p>",
        html_escape(&s.wsprnet_spot_config)
    );

    let checked = if s.ntp_enabled { " checked" } else { "" };
    let _ = write!(
        html,
        "<p><label><input type=\"checkbox\" name=\"ntp_enabled\"{checked}> \
         Sync clock via NTP (required to hit the WSPR 2-minute slot grid \
         from a cold boot)</label></p>"
    );
    let _ = write!(
        html,
        "<p><label>NTP server<br>\
         <input name=\"ntp_server\" maxlength=\"48\" value=\"{}\"></label></p>",
        html_escape(&s.ntp_server)
    );

    let _ = write!(html, "<p><button type=\"submit\">Save</button></p></form>");
    let _ = write!(html, "</body></html>");
    html
}

// ---- form parsing --------------------------------------------------

/// Every field the form submits, still as raw (percent-decoded) text
/// — [`validate`] is what turns this into a [`Settings`] or an error.
#[derive(Default)]
struct FormFields {
    call: Option<String>,
    grid: Option<String>,
    tx_power_dbm: Option<String>,
    band: Option<String>,
    wsprnet: Option<String>,
    ntp_enabled: bool,
    ntp_server: Option<String>,
}

/// Percent-decode one `x-www-form-urlencoded` value — the inverse of
/// `crate::wsprnet`'s own (private) `encode_value` (not reused
/// directly: that function is private to its own module and this is
/// decode, not encode, but the two agree on the same escaping rules).
///
/// Byte-wise, not UTF-8-aware: every value this form produces is
/// ASCII (callsign, grid, digits, a URL), so treating each decoded
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
            "call" => fields.call = Some(value),
            "grid" => fields.grid = Some(value),
            "tx_power_dbm" => fields.tx_power_dbm = Some(value),
            "band" => fields.band = Some(value),
            "wsprnet" => fields.wsprnet = Some(value),
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
    let call_raw = fields.call.as_deref().unwrap_or(&current.call).trim().to_uppercase();
    let call = heapless::String::<16>::try_from(call_raw.as_str())
        .map_err(|_| "callsign too long (max 16 chars)".to_string())?;

    let grid_raw = fields.grid.as_deref().unwrap_or(&current.grid).trim().to_uppercase();
    if !grid_raw.is_empty() && grid_raw.len() != 4 && grid_raw.len() != 6 {
        return Err("grid locator must be empty, 4, or 6 characters".to_string());
    }
    if !grid_raw.chars().all(|c| c.is_ascii_alphanumeric()) {
        return Err("grid locator must be letters/digits only".to_string());
    }
    let grid = heapless::String::<8>::try_from(grid_raw.as_str())
        .map_err(|_| "grid locator too long".to_string())?;

    let tx_power_dbm: i8 = match fields.tx_power_dbm.as_deref().map(str::trim) {
        Some(raw) if !raw.is_empty() => {
            raw.parse().map_err(|_| "tx power must be a whole number".to_string())?
        }
        _ => current.tx_power_dbm,
    };
    if !(0..=60).contains(&tx_power_dbm) {
        return Err("tx power must be between 0 and 60 dBm".to_string());
    }

    let band_raw = fields.band.as_deref().unwrap_or("").trim();
    let band_idx = if band_raw.is_empty() {
        current.band_idx
    } else {
        let idx: usize = band_raw.parse().map_err(|_| "invalid band selection".to_string())?;
        if idx >= WSPR_BANDS.len() {
            return Err("invalid band selection".to_string());
        }
        idx as u8
    };

    let wsprnet_raw = fields
        .wsprnet
        .as_deref()
        .unwrap_or(&current.wsprnet_spot_config)
        .trim();
    let wsprnet_spot_config = heapless::String::<80>::try_from(wsprnet_raw)
        .map_err(|_| "wsprnet setting too long (max 80 chars)".to_string())?;

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
        call,
        grid,
        tx_power_dbm,
        band_idx,
        wsprnet_spot_config,
        ntp_enabled,
        ntp_server,
    })
}
