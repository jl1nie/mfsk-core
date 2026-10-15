#!/usr/bin/env python3
"""Exercise the real Tauri frontend against synthetic FT8 over local rtl_tcp.

Build the app and `ft8_rtltcp` first; see apps/skimmer/README.md.
Each case has its own settings, database and logs, with reporting disabled.
"""
import argparse
import json
import pathlib
import socket
import sqlite3
import subprocess
import tempfile
import time
import os
import statistics


def stop(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run_case(args, case, root):
    folder = root / case
    folder.mkdir()
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", args.port or 0))
        port = probe.getsockname()[1]
    settings = {
        "servers": [{"name": f"IQ test {case}", "address": f"127.0.0.1:{port}",
                     "protocol": "rtl_tcp", "rtlFormat": "16", "rtlCalibrate": 0,
                     "enabled": True, "pskReporter": False}],
        "channels": [{"server": 0, "mode": "FT8", "dialHz": 14074000}],
        "clockSource": "system", "waterfall": True, "earlyDecode": True,
        "dbEnabled": True, "dbPath": str(folder / "skimmer.db"),
        "logDir": str(folder),
    }
    profile = folder / "settings.json"
    profile.write_text(json.dumps(settings, indent=2))
    env = dict(os.environ, MFSK_SKIMMER_SETTINGS=str(profile), MFSK_SKIMMER_AUTOSTART="1")
    print(f"{case}: starting GUI, 127.0.0.1:{port}, {args.seconds}s", flush=True)
    with (folder / "server.log").open("w") as server_log, (folder / "gui.log").open("w") as gui_log:
        server = subprocess.Popen([str(args.server), "--listen", f"127.0.0.1:{port}",
                                   "--impairment", case, "--snr", str(args.snr)], stdout=server_log, stderr=server_log)
        gui = None
        try:
            # Do not probe with a client: that would start an unnecessary fixture stream.
            deadline = time.monotonic() + 10
            while "FT8 rtl_tcp on" not in (folder / "server.log").read_text():
                if server.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("IQ server did not start")
                time.sleep(0.1)
            gui = subprocess.Popen([str(args.app)], env=env, stdout=gui_log, stderr=gui_log)
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                if gui.poll() is not None or server.poll() is not None:
                    raise RuntimeError("GUI or IQ server exited early")
                time.sleep(0.25)
            database = folder / "skimmer.db"
            with sqlite3.connect(f"file:{database}?mode=ro", uri=True) as db:
                rows = db.execute("SELECT t, text, freq_hz, dt, snr FROM decodes ORDER BY t").fetchall()
            health = (folder / "STATUS.log").read_text()
            messages = [r for r in rows if r[1] == "CQ K1ABC FN42"]
            gaps = sum(" gap " in line for line in health.splitlines())
            reanchors = sum(" reanchor " in line for line in health.splitlines())
            connected = " streaming 1024000 S/s " in health
            disconnected = " disconnected " in health
            passed = (len(messages) == 0 if case == "dropped" else len(messages) >= 2)
            passed = passed and connected and not disconnected and gaps == 0
            # Deleted source samples destroy the nominal clock; its reanchors are
            # part of this negative control, rather than a regression in the fix.
            if case != "dropped":
                passed = passed and reanchors == 0
            if case != "dropped":
                passed = passed and all(abs(r[2] - 14075500) < 3 for r in messages)
                # Do not let an accidentally strong signal pass a weak-signal test.
                passed = passed and bool(messages) and abs(
                    statistics.median(r[4] for r in messages) - args.snr
                ) <= 3
            result = dict(case=case, passed=passed, messages=len(messages), gaps=gaps,
                          reanchors=reanchors, connected=connected, disconnected=disconnected,
                          target_snr=args.snr, reported_snr=[r[4] for r in messages],
                          rows=rows, artifacts=str(folder))
            (folder / "result.json").write_text(json.dumps(result, indent=2))
            print(json.dumps(result), flush=True)
            if args.keep_open and case == args.cases[-1]:
                print("GUI and IQ server remain running; close the window or press Ctrl-C to stop.", flush=True)
                while gui.poll() is None and server.poll() is None:
                    time.sleep(0.25)
            return result
        finally:
            if gui is not None:
                stop(gui)
            stop(server)


def main():
    repo = pathlib.Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app", type=pathlib.Path,
                        default=repo / "apps/skimmer/gui/src-tauri/target/release/bundle/macos/mfsk skimmer.app/Contents/MacOS/skimmer-gui")
    parser.add_argument("--server", type=pathlib.Path,
                        default=repo / "target/release/examples/ft8_rtltcp")
    parser.add_argument("--seconds", type=int, default=70)
    parser.add_argument("--snr", type=float, default=-10.0, help="signal/noise in 2500 Hz (-30 to 0 dB)")
    parser.add_argument("--cases", nargs="+", choices=["clean", "filled", "dropped", "stall"],
                        default=["clean", "filled", "dropped", "stall"])
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--port", type=int, help="fixed localhost port instead of an available port")
    parser.add_argument("--keep-open", action="store_true", help="leave the last GUI/server open for inspection")
    args = parser.parse_args()
    if args.seconds < 60:
        parser.error("allow at least 60 seconds for startup and two complete FT8 slots")
    if not -30 <= args.snr <= 0:
        parser.error("SNR must be between -30 and 0 dB")
    if args.port is not None and not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    for path in [args.app, args.server]:
        if not path.is_file():
            parser.error(f"build first: {path}")
    root = args.output or pathlib.Path(tempfile.mkdtemp(prefix="mfsk-gui-iq-"))
    root.mkdir(parents=True, exist_ok=True)
    print(f"Artifacts: {root}", flush=True)
    results = [run_case(args, case, root) for case in args.cases]
    (root / "results.json").write_text(json.dumps(results, indent=2))
    raise SystemExit(0 if all(r["passed"] for r in results) else 1)


if __name__ == "__main__":
    main()
