// SPDX-License-Identifier: GPL-3.0-only
//! Host harness for the parts of `embedded-poc/mfsk-app-shared` that
//! do not touch the ESP-IDF.
//!
//! That crate depends on `esp-idf-svc`, so it only builds for Xtensa
//! and is excluded from this workspace — which means unit tests written
//! inside it never run anywhere. This repository has been bitten by
//! exactly that before (five protocols' golden tests silently skipping
//! in CI, see `docs/notes` and `MFSK_REQUIRE_CORPUS`), so a module whose
//! logic is worth testing gets compiled here instead of being trusted.
//!
//! Modules are pulled in by `#[path]` rather than copied: there is one
//! source file, and it is the one that ships.
//!
//! Only add a module here if it is genuinely target-independent. If it
//! needs `esp_idf_svc`, it does not belong.

/// Unix-epoch → UTC calendar time — pure integer arithmetic, no I/O.
#[path = "../../../embedded-poc/mfsk-app-shared/src/civil_time.rs"]
pub mod civil_time;

/// The capture window a receiver opens on a UTC slot boundary — pure
/// sample arithmetic over a phase reading passed in, so the batch
/// splitting and the gap between windows are testable without a radio.
#[path = "../../../embedded-poc/mfsk-app-shared/src/capture_window.rs"]
pub mod capture_window;

/// CONFIG > FREQ tables and their paging — pure data; the NVS half is
/// `#[cfg(espidf)]`.
#[path = "../../../embedded-poc/mfsk-app-shared/src/freq_presets.rs"]
pub mod freq_presets;

/// The JTTY transmit sequencer — pure sample arithmetic over the audio clock
/// (#499): waiting for a clear channel, PTT lead and tail, and which samples are
/// the message and which the receiver must be fed zeros for.
#[path = "../../../embedded-poc/mfsk-app-shared/src/jtty_tx.rs"]
pub mod jtty_tx;

/// The JTTY receiver's sample clock and staging ring on a board (#499, E1):
/// positioned gaps, and a deficit against the timer that tells a lost run of
/// samples from a source clock that is merely slow — pure arithmetic over
/// sample counts and microseconds passed in.
#[path = "../../../embedded-poc/mfsk-app-shared/src/jtty_rx_clock.rs"]
pub mod jtty_rx_clock;

/// Whether a boot brings WiFi up — the `WIFI` and `TIME` rows of the CONFIG
/// page as a truth table (#381): `TIME: AIR DT` turns it off.
#[path = "../../../embedded-poc/mfsk-app-shared/src/wifi_policy.rs"]
pub mod wifi_policy;

/// Slot parity — pure arithmetic; `time_sync` needs it.
#[path = "../../../embedded-poc/mfsk-app-shared/src/parity.rs"]
pub mod parity;

/// The FT4 capture window's place on the slot grid — from
/// `embedded-shared`, not `mfsk-app-shared`, and here rather than in a
/// hosttest crate of its own because this one is what CI runs. It is
/// integer arithmetic over sample counts with no dependency at all;
/// the module it was split out of pulls in `esp_idf_svc` and so cannot
/// be compiled off-target, which left a live run against a radio as
/// the only thing checking it.
#[path = "../../../embedded-poc/embedded-shared/src/apps/ft4_grid.rs"]
pub mod ft4_grid;

/// Slot-boundary time sync. Pulled in for `ClockSource`: the rule that
/// only an NTP-disciplined clock may be written back to the RTC is the
/// kind of thing that was wrong for months without anything failing
/// (#354), so it gets a test where tests run.
#[path = "../../../embedded-poc/mfsk-app-shared/src/time_sync.rs"]
pub mod time_sync;

/// Persisted grid-phase fix — the NVS I/O is `#[cfg(espidf)]`, but the
/// `GridFix::correction_for` re-wrap and staleness logic is pure and
/// the thing most worth a test (a wrong sign here mis-aligns FT4).
#[path = "../../../embedded-poc/mfsk-app-shared/src/grid_fix.rs"]
pub mod grid_fix;

#[path = "../../../embedded-poc/mfsk-app-shared/src/grid_state.rs"]
pub mod grid_state;

/// The CQ-side QSO state machine for portable activations — pure, by
/// design, so that every exchange it can get into is a test here
/// rather than an afternoon on a summit.
#[path = "../../../embedded-poc/mfsk-app-shared/src/activator.rs"]
pub mod activator;

/// ADIF records and `ALL.TXT` lines — pure formatting; what they are
/// checked against is WSJT-X's own writers, since a log that does not
/// merge into WSJT-X's is a log someone has to retype.
#[path = "../../../embedded-poc/mfsk-app-shared/src/adif.rs"]
pub mod adif;
#[path = "../../../embedded-poc/mfsk-app-shared/src/all_txt.rs"]
pub mod all_txt;

/// Icom CI-V framing — the bytes both the StickS3 (BLE) and the CoreS3
/// (USB CDC) exchange with an IC-705, tested where tests run rather
/// than on a radio.
#[path = "../../../embedded-poc/mfsk-app-shared/src/civ_frame.rs"]
pub mod civ_frame;

/// The FT8/FT4 decoded-row list. Pulled in as a `ui` module tree so
/// the file's own `crate::ui::state::...` paths resolve unchanged.
///
/// Here because [`ui::decoded_list::row_text`] shipped a buffer
/// narrower than the row width it computed against, and
/// `heapless::String::push_str` writes nothing rather than truncating:
/// one decoded row in five rendered with an empty message column on a
/// JA morning. Nothing on the board could have caught it — the panel
/// is the only consumer and it has no assertions.
pub mod ui;

#[cfg(test)]
mod decoded_row_tests {
    use super::ui::decoded_list::row_text;
    use super::ui::state::DecodedRow;

    fn row(msg: &str) -> DecodedRow {
        let mut m: heapless::String<22> = heapless::String::new();
        m.push_str(&msg[..msg.len().min(22)]).unwrap();
        DecodedRow {
            df_hz: 1891,
            snr_db: -11,
            hard_errors: 0,
            dt_ds: 0,
            msg: m,
            slot_seq: 0,
            first_seq: 0,
        }
    }

    /// The regression: 20 characters is an ordinary `/P` exchange and
    /// it used to render as an empty message column.
    #[test]
    fn a_twenty_character_message_reaches_the_row() {
        let t = row_text(&row("JG3AGB/P JE1NGI PM95"));
        assert!(
            t.ends_with("JG3AGB/P JE1NGI PM95"),
            "message dropped from row: {t:?}"
        );
    }

    /// Every length a `DecodedRow` can hold must appear, since `msg`
    /// is already capped at 22 upstream. Anything that fits the row
    /// must arrive whole.
    #[test]
    fn no_length_up_to_the_row_cap_is_silently_dropped() {
        for n in 1..=22usize {
            let msg: String = "JA1ABC/P JE1NGI PM95XY".chars().take(n).collect();
            let t = row_text(&row(&msg));
            assert!(
                t.ends_with(&msg),
                "length {n} dropped: {t:?} does not end with {msg:?}"
            );
        }
    }

    /// The prefix is the part the operator reads when the message is
    /// gone, so it has to keep its shape: dB, DT, frequency.
    #[test]
    fn the_prefix_is_wsjtx_column_order() {
        let t = row_text(&row("CQ JA1XXN PM96"));
        assert!(t.starts_with("-11 +0.0 1891 "), "prefix changed: {t:?}");
    }
}

/// Every message the activator can emit, through the packer the board
/// will hand it to, and back.
#[cfg(test)]
mod activator_pack_tests {
    use super::activator::{Config, CqModifier, TxMsg};
    use mfsk_core::msg::wsjt77::{pack77, pack77_free_text, unpack77};

    fn round_trip(cfg: &Config, m: &TxMsg) -> Option<String> {
        let (a, b, c) = m.fields(cfg);
        let bits = if b.is_empty() {
            pack77_free_text(&a)?
        } else {
            pack77(&a, &b, &c)?
        };
        unpack77(&bits)
    }

    fn std(to: &str, ex: &str) -> TxMsg {
        TxMsg::Std {
            to: to.try_into().unwrap(),
            exchange: ex.try_into().unwrap(),
        }
    }

    #[test]
    fn every_message_packs_and_reads_back_as_sent() {
        for my in ["JL1NIE", "JL1NIE/P"] {
            let mut cfg = Config::new(my, "PM95");
            let mut msgs = vec![TxMsg::Cq];
            for ex in ["-07", "R-15", "+03", "RR73", "73"] {
                msgs.push(std("W1AW", ex));
            }
            for cq in [
                CqModifier::None,
                CqModifier::Token("SOTA".try_into().unwrap()),
                CqModifier::Token("DX".try_into().unwrap()),
                CqModifier::Number(7),
                CqModifier::FreeText("QRV JA-1234".try_into().unwrap()),
            ] {
                cfg.cq = cq;
                for m in &msgs {
                    let want = m.text(&cfg);
                    assert_eq!(
                        round_trip(&cfg, m).as_deref(),
                        Some(want.as_str()),
                        "{my}: {want}"
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod log_format_tests {
    use super::adif::{self, Qso};
    use super::all_txt::{rx_line, tx_line};

    /// 2026-09-22 10:15:00 UTC.
    const T0: i64 = 1_790_072_100;

    #[test]
    fn all_txt_lines_match_wsjtx_layout() {
        assert_eq!(
            rx_line(
                T0,
                Some(14_074_000),
                "FT8",
                -12,
                0.3,
                1234,
                "CQ JA1ABC PM95"
            ),
            "260922_101500    14.074 Rx FT8    -12  0.3 1234 CQ JA1ABC PM95\n"
        );
        assert_eq!(
            rx_line(
                T0,
                Some(7_047_500),
                "FT4",
                5,
                -1.25,
                987,
                "W1AW JA1ABC R-03"
            ),
            "260922_101500     7.048 Rx FT4      5 -1.2  987 W1AW JA1ABC R-03\n"
        );
        assert_eq!(
            tx_line(
                T0 + 15,
                Some(14_074_000),
                "FT8",
                1500,
                "JA1ABC JL1NIE/P -12"
            ),
            "260922_101515    14.074 Tx FT8      0  0.0 1500 JA1ABC JL1NIE/P -12\n"
        );
        // No CAT link: 0.000, not a guess.
        assert!(
            rx_line(T0, None, "FT8", 0, 0.0, 1000, "X").starts_with("260922_101500     0.000 Rx")
        );
    }

    fn qso() -> Qso<'static> {
        Qso {
            call: "W1AW",
            grid: "FN31",
            mode: "FT8",
            rst_sent: -7,
            rst_rcvd: Some(-12),
            on_unix: T0,
            off_unix: T0 + 45,
            dial_hz: Some(14_074_000),
            my_call: "JL1NIE/P",
            my_grid: "PM95",
            my_sota_ref: Some("JA/KN-006"),
            my_pota_ref: None,
        }
    }

    /// Field for field what `LogBook::QSOToADIF` writes for the same
    /// contact, plus the SOTA reference WSJT-X has no field for.
    #[test]
    fn adif_record_matches_wsjtx() {
        assert_eq!(
            adif::record(&qso()),
            "<call:4>W1AW <gridsquare:4>FN31 <mode:3>FT8 <rst_sent:3>-07 \
             <rst_rcvd:3>-12 <qso_date:8>20260922 <time_on:6>101500 \
             <qso_date_off:8>20260922 <time_off:6>101545 <band:3>20m \
             <freq:9>14.074000 <station_callsign:8>JL1NIE/P \
             <my_gridsquare:4>PM95 <my_sota_ref:9>JA/KN-006 <eor>\n"
        );
    }

    #[test]
    fn adif_ft4_is_an_mfsk_submode_and_unknowns_are_left_out() {
        let mut q = qso();
        q.mode = "FT4";
        q.rst_rcvd = None;
        q.dial_hz = None;
        q.grid = "";
        q.my_sota_ref = None;
        q.my_pota_ref = Some("JA-1234");
        let r = adif::record(&q);
        assert!(r.contains("<mode:4>MFSK <submode:3>FT4"), "{r}");
        assert!(r.contains("<gridsquare:0> "), "{r}");
        for absent in ["rst_rcvd", "<band", "<freq", "my_sota_ref"] {
            assert!(!r.contains(absent), "{absent} in {r}");
        }
        assert!(r.contains("<my_pota_ref:7>JA-1234"), "{r}");
    }

    #[test]
    fn bands() {
        assert_eq!(adif::band_for_hz(7_074_000), Some("40m"));
        assert_eq!(adif::band_for_hz(50_313_000), Some("6m"));
        assert_eq!(adif::band_for_hz(144_460_000), Some("2m"));
        assert_eq!(adif::band_for_hz(9_000_000), None);
    }

    #[test]
    fn adif_header_ends_in_eoh() {
        let h = adif::header(T0);
        assert!(h.starts_with(
            "ADIF Export\n<adif_ver:5>3.1.4\n<created_timestamp:15>20260922 101500\n"
        ));
        assert!(h.ends_with("<eoh>\n"));
    }
}

#[cfg(test)]
mod jtty_rx_clock_tests {
    use super::jtty_rx_clock::{FS, SampleClock, Staging, Verdict};

    /// Hand over `n`-sample blocks for `secs`, each `late_us` after it was
    /// recorded, at a source rate of `rate` samples a second; the verdicts.
    fn run(secs: i64, n: usize, rate: f64, late_us: impl Fn(usize) -> i64) -> Vec<Verdict> {
        let mut c = SampleClock::new();
        let mut out = Vec::new();
        let blocks = (secs as f64 * rate / n as f64) as usize;
        for k in 1..=blocks {
            let recorded_us = (k * n) as f64 * 1e6 / rate;
            out.push(c.delivered(n, recorded_us as i64 + late_us(k)));
        }
        out
    }

    #[test]
    fn a_timely_source_is_never_filled() {
        assert!(
            run(600, 256, 12_000.0, |_| 0)
                .iter()
                .all(|v| *v == Verdict::Ok)
        );
    }

    #[test]
    fn delivery_jitter_under_20_ms_is_not_a_gap() {
        // a UAC read or SIM block handed over up to 19 ms after it was recorded
        let v = run(120, 256, 12_000.0, |k| ((k * 7919) % 19) as i64 * 1_000);
        assert!(v.iter().all(|v| *v == Verdict::Ok), "{v:?}");
    }

    #[test]
    fn a_source_clock_off_by_hundreds_of_ppm_is_not_a_gap() {
        // an IC-705 measured at 12 003.9 sa/s, and the same the other way
        for rate in [12_003.9, 11_996.1] {
            let v = run(3_600, 256, rate, |_| 0);
            assert!(v.iter().all(|v| *v == Verdict::Ok), "rate {rate}");
        }
    }

    #[test]
    fn a_lost_run_is_filled_where_it_happened_and_only_once() {
        let mut c = SampleClock::new();
        let n = 256usize;
        let mut t = 0i64;
        for _ in 0..100 {
            t += n as i64 * 1_000_000 / FS;
            assert_eq!(c.delivered(n, t), Verdict::Ok);
        }
        // 50 ms of audio never arrives
        t += 50_000 + n as i64 * 1_000_000 / FS;
        match c.delivered(n, t) {
            Verdict::Fill(z) => {
                assert!((595..=605).contains(&z), "{z}");
                c.filled(z);
            }
            v => panic!("{v:?}"),
        }
        for _ in 0..100 {
            t += n as i64 * 1_000_000 / FS;
            assert_eq!(c.delivered(n, t), Verdict::Ok);
        }
    }

    #[test]
    fn more_than_a_second_missing_is_a_reset() {
        let mut c = SampleClock::new();
        assert_eq!(c.delivered(256, 21_333), Verdict::Ok);
        assert_eq!(c.delivered(256, 21_333 * 2 + 1_500_000), Verdict::Reset);
    }

    #[test]
    fn staging_puts_the_zeros_where_the_samples_were_lost() {
        let mut s = Staging::new(4);
        assert_eq!(s.push(&[1, 2, 3]), 0);
        assert_eq!(s.push(&[4, 5, 6]), 2); // 5, 6 do not fit
        s.gap(1); // and one more missing at the same place
        let mut out = Vec::new();
        assert_eq!(s.drain_into(&mut out), 3);
        assert_eq!(out, vec![1, 2, 3, 4, 0, 0, 0]);
        assert!(s.is_empty());
        // the next samples follow the zeros
        s.push(&[7]);
        s.drain_into(&mut out);
        assert_eq!(out.last(), Some(&7));
    }

    #[test]
    fn a_gap_between_blocks_lands_between_them() {
        let mut s = Staging::new(100);
        s.push(&[1, 2]);
        s.gap(3);
        s.push(&[9]);
        let mut out = Vec::new();
        assert_eq!(s.drain_into(&mut out), 3);
        assert_eq!(out, vec![1, 2, 0, 0, 0, 9]);
    }
}

#[cfg(test)]
mod jtty_all_txt_tests {
    use super::all_txt::message_line;

    #[test]
    fn a_jtty_line_leaves_snr_and_dt_blank_in_their_columns() {
        // 2026-09-22 10:15:07 UTC
        let l = message_line(1_790_072_107, None, "JTTY", 1506, "CQ JA1ABC CQ");
        assert_eq!(
            l,
            "260922_101507     0.000 Rx JTTY            1506 CQ JA1ABC CQ\n"
        );
        // the frequency column sits where rx_line puts it
        let r =
            super::all_txt::rx_line(1_790_072_107, None, "JTTY", -12, 0.3, 1506, "CQ JA1ABC CQ");
        assert_eq!(l.find("1506"), r.find("1506"));
    }
}
