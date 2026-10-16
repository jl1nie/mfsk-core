#!/usr/bin/env python3
"""Acquisition-rate / fixed-delay / API-loss study with the actual Tauri GUI.

Every server writes independent acquisition/delivery truth. One GUI connects to
all cases with reporting disabled and separate SQLite/settings artifacts.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import socket
import sqlite3
import statistics
import subprocess
import tempfile
import time
from gui_smoke import stop

CALLS = ['K1ABC', 'W9XYZ', 'JA1ABC', 'DL1ABC', 'VK3ABC', 'G1ABC']
CASES = [
    dict(name='drop_calibrated', mode='dropped', ppm=0, hole=65, api=90, delay=750, compensate=750, calibrate=30),
    dict(name='fill_nominal', mode='filled', ppm=0, hole=65, api=90, delay=750, compensate=750, calibrate=0),
    dict(name='combined_fill_uncorrected', mode='filled', ppm=-17000, hole=65, api=90, delay=750, compensate=750, calibrate=0),
    dict(name='combined_fill_calibrated', mode='filled', ppm=-17000, hole=65, api=90, delay=750, compensate=750, calibrate=30),
    dict(name='fixed_delay_uncompensated', mode='clean', ppm=0, hole=0, api=0, delay=3000, compensate=0, calibrate=0),
    dict(name='fixed_delay_compensated', mode='clean', ppm=0, hole=0, api=0, delay=3000, compensate=3000, calibrate=0),
    dict(name='long_fill_jitter', mode='filled', ppm=0, hole=1000, api=1025, delay=750, compensate=750, calibrate=0, period=28571.4286, minimum=True, infer=False, extra=True),
    dict(name='long_fill_direct', mode='filled', ppm=0, hole=1000, api=1025, delay=750, compensate=750, calibrate=0, period=28571.4286, minimum=False, infer=False, extra=True),
    dict(name='vary_drop_calibrated', mode='dropped', ppm=0, hole=65, api=90, delay=750, compensate=750, calibrate=30, vary=True),
    dict(name='vary_fill_nominal', mode='filled', ppm=0, hole=65, api=90, delay=750, compensate=750, calibrate=0, vary=True),
]


def evaluate(folder, cases, elapsed_end):
    health = (folder / 'STATUS.log').read_text()
    sources = {}
    for case in cases:
        # The producer may be flushing a record during this snapshot. An
        # unfinished LAST line is not a delivered block and is excluded.
        text = (folder / f"{case['name']}.truth.jsonl").read_text()
        complete = text[:text.rfind('\n') + 1]
        records = [json.loads(line) for line in complete.splitlines()]
        configs = [r for r in records if r['type'] == 'source']
        if len(configs) != 1:
            raise RuntimeError(f"{case['name']}: source reconnects invalidate the common window")
        sources[case['name']] = (configs[0], [r for r in records if r['type'] == 'block'])
    # A shared window after all 30s startup measurements, with a full 20s tail
    # for a complete last slot, pipeline latency and decoder completion.
    first = max(s[0]['t0_ns'] / 1e9 for s in sources.values())
    begin = math.ceil((first + 75) / 15) * 15
    capture_end = min(s[1][-1]['capture_end_utc_ns'] / 1e9 for s in sources.values())
    end = math.floor((min(elapsed_end, capture_end) - 20) / 15) * 15
    expected = {s: f'CQ {CALLS[(s // 15) % len(CALLS)]} FN42' for s in range(begin, end, 15)}
    if len(expected) < 3:
        raise RuntimeError('not enough complete source slots')
    with sqlite3.connect(f"file:{folder / 'skimmer.db'}?mode=ro", uri=True) as db:
        rows = db.execute('SELECT server,t,text,freq_hz,dt,snr FROM decodes ORDER BY t').fetchall()
    results = []
    for case in cases:
        meta, blocks = sources[case['name']]
        decoded, utc_correct, reliable, observations = set(), set(), set(), []
        for server, slot, text, freq, dt, snr in rows:
            if server != case['name']:
                continue
            matching = [s for s, wanted in expected.items() if text == wanted and abs(s-slot) <= 30]
            if not matching:
                continue
            source_slot = min(matching, key=lambda s: abs(s-slot))
            decoded.add(source_slot)
            if slot == source_slot:
                utc_correct.add(source_slot)
            if slot == source_slot and abs(freq-14075500) <= 2 and abs(dt) <= .05:
                reliable.add(source_slot)
            observations.append(dict(source_slot=source_slot, slot_utc_error_s=slot-source_slot,
                                     freq_error_hz=freq-14075500, dt_s=dt, snr_db=snr, text=text))
        lines = [line for line in health.splitlines() if line.startswith(case['name'] + ' ')]
        connected = any(' streaming ' in line for line in lines)
        disconnected = any(' disconnected ' in line for line in lines)
        dropped_worker = [int(m[1]) for line in lines if (m := re.search(r' slots \d+/(\d+)', line))]
        selected = [b for b in blocks if begin * 1e9 <= b['capture_end_utc_ns'] <= end * 1e9]
        acquisition = sum(b['end_sample_exclusive']-b['first_sample'] for b in selected)
        payload = sum(b['payload_samples'] for b in selected)
        erased = sum(b['erased_samples'] for b in selected)
        delays = [(b['delivery_utc_ns']-b['capture_end_utc_ns']) / 1e6 for b in selected]
        rates = [int(m[1]) for line in lines if (m := re.search(r' streaming (\d+) S/s', line))]
        assert delays and min(delays) >= 0, 'future IQ was sent'
        assert connected and not disconnected, 'transport disconnected'
        assert not any(dropped_worker), 'host decoder overload invalidates this experiment'
        result = dict(**case, target_snr=meta['target_snr'], opportunities=len(expected),
                      decoded=len(decoded), correct_slot_utc=len(utc_correct), reliable=len(reliable),
                      initial_rx_rates=rates, source_rate_hz=meta['actual_rate_hz'],
                      apparent_payload_rate_hz=meta['actual_rate_hz']*payload/acquisition,
                      erased_fraction=erased/acquisition, delay_min_ms=min(delays),
                      delay_median_ms=statistics.median(delays), delay_max_ms=max(delays),
                      gaps=sum(' gap ' in line for line in lines),
                      reanchors=sum(' reanchor ' in line for line in lines), rows=observations)
        results.append(result)
        print(json.dumps(result), flush=True)
    report = dict(window_begin_utc=begin, window_end_utc=end, results=results)
    (folder / 'results.json').write_text(json.dumps(report, indent=2))
    return results


def main():
    repo = Path(__file__).resolve().parents[2]
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path)
    p.add_argument('--seconds', type=int, default=210)
    p.add_argument('--snr', type=float, default=-18)
    p.add_argument('--cases', nargs='+', choices=[c['name'] for c in CASES], default=[c['name'] for c in CASES if not c.get('vary') and not c.get('extra')])
    p.add_argument('--app', type=Path, default=repo / 'apps/skimmer/gui/src-tauri/target/release/bundle/macos/mfsk skimmer.app/Contents/MacOS/skimmer-gui')
    args = p.parse_args()
    if args.seconds < 150 or not -30 <= args.snr <= 0:
        p.error('allow at least 150 seconds, SNR -30 to 0 dB')
    cases = [c for c in CASES if args.cases is None or c['name'] in args.cases]
    folder = args.output
    if folder is None:
        folder = Path(tempfile.mkdtemp(prefix='mfsk-gui-impairment-'))
    else:
        folder.mkdir(parents=True, exist_ok=False)
    print(f'Artifacts: {folder}', flush=True)
    servers, processes, handles = [], [], []
    gui = None
    try:
        for case in cases:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            log = (folder / f"{case['name']}.server.log").open('w')
            handles.append(log)
            command = [str(repo / 'target/release/examples/ft8_rtltcp'), '--listen', f'127.0.0.1:{port}',
                       '--impairment', case['mode'], '--ppm', str(case['ppm']), '--snr', str(args.snr),
                       '--fixed-delay-ms', str(case['delay']), '--loss-ms', str(case['hole']),
                       '--api-delay-ms', str(case['api']), '--loss-period-ms', str(case.get('period', 1850)), '--vary-messages',
                       '--truth', str(folder / f"{case['name']}.truth.jsonl")]
            if case.get('vary'):
                command.append('--vary-loss')
            process = subprocess.Popen(command, stdout=log, stderr=log)
            processes.append(process)
            deadline = time.monotonic() + 10
            while 'FT8 rtl_tcp on' not in (folder / f"{case['name']}.server.log").read_text():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('IQ server did not start')
                time.sleep(.1)
            servers.append(dict(name=case['name'], address=f'127.0.0.1:{port}', protocol='rtl_tcp',
                                rtlFormat='16', rtlCalibrate=case['calibrate'],
                                rtlMinimumArrival=case.get('minimum', True), rtlInferGaps=case.get('infer', False),
                                networkDelayMs=case['compensate'], enabled=True, pskReporter=False))
        settings = dict(servers=servers, channels=[dict(server=i, mode='FT8', dialHz=14074000) for i in range(len(cases))],
                        clockSource='system', waterfall=True, earlyDecode=True, dbEnabled=True,
                        dbPath=str(folder / 'skimmer.db'), logDir=str(folder))
        profile = folder / 'settings.json'
        profile.write_text(json.dumps(settings, indent=2))
        log = (folder / 'gui.log').open('w')
        handles.append(log)
        gui = subprocess.Popen([str(args.app)], stdout=log, stderr=log,
                               env=dict(os.environ, MFSK_SKIMMER_SETTINGS=str(profile), MFSK_SKIMMER_AUTOSTART='1'))
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            if gui.poll() is not None or any(process.poll() is not None for process in processes):
                raise RuntimeError('GUI or IQ server exited early')
            time.sleep(.25)
        results = evaluate(folder, cases, time.time())
        # These are positive controls, not a promise that impaired cases must decode.
        for r in results:
            if r['name'] in ('fill_nominal', 'combined_fill_calibrated', 'fixed_delay_compensated', 'vary_fill_nominal', 'long_fill_jitter'):
                if r['reliable'] != r['opportunities']:
                    raise RuntimeError(f"positive control did not recover every slot: {r['name']}")
    finally:
        if gui is not None:
            stop(gui)
        for process in processes:
            stop(process)
        for handle in handles:
            handle.close()


if __name__ == '__main__':
    main()
