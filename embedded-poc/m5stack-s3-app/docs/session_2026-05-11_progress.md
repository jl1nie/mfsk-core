# Session 2026-05-11 progress — m5stack-s3-app bring-up

Resume notes for picking this branch up on another machine. The author
expects to continue remotely, so anything that wasn't committed in the
prior session — Windows-side firewall state, WSL networking findings,
device-side observations — is captured here.

## What landed in main today

Three commits, fast-forwarded to `origin/main`:

- `a03d9e7` — `m5stack-s3-app: derive partition CSV path from
  CARGO_MANIFEST_DIR`. `build.rs` regenerates
  `sdkconfig.gen.defaults` per-checkout. `bootstrap-sdkconfig.sh` primes
  the file on a fresh clone (esp-idf-sys's build script runs before
  `build.rs`).
- `21af1f0` — `embedded-poc/scripts: flash-monitor for espflash 4.x`.
  `-T` → `--partition-table`.
- `6b8ca96` — `m5stack-s3: derive partition CSV path from
  CARGO_MANIFEST_DIR`. Same fix ported to the sibling compute-bench
  crate (closes the second instance of the `119657a` foot-gun).

Working branch `phase06-sdkconfig-abs-path` is at `6b8ca96` and pushed.
It currently equals `main`; reuse it or branch fresh for Phase 0.7.

## On a fresh clone

```sh
# m5stack-s3-app
cd embedded-poc/m5stack-s3-app
./bootstrap-sdkconfig.sh     # one-shot
# create cfg.toml from cfg-sample.toml (WiFi creds, pc_ip, port)
source ~/export-esp.sh
cargo build --release

# m5stack-s3 (compute bench)
cd ../m5stack-s3
./bootstrap-sdkconfig.sh
cargo build --release
```

Subsequent builds: `build.rs` re-writes the gen file automatically; no
bootstrap needed.

## Phase 0.6 UDP log: verified working LAN-side, WSL inbound blocked

End-to-end path confirmed with the live device:

- Device IP `192.168.3.13`, MAC `70:04:1d:da:75:54` (M5StickC Plus2 S3
  on `NETGEAR18F2EF`, channel 13).
- Firmware reaches `UDP log sink up → 192.168.3.8:9999` and emits the
  expected `alive tick=N free_heap=…` lines plus boot banner.
- A **Windows-side** UDP listener (`UdpClient.Receive` on port 9999)
  receives the datagrams cleanly.
- A **WSL-side** `nc -lu 9999` on `eth0` (`192.168.3.8/24`, mirrored
  mode) receives nothing from the device, even after applying:

```powershell
New-NetFirewallRule -DisplayName "WSL UDP 9999 (m5s3 log)" `
  -Direction Inbound -Protocol UDP -LocalPort 9999 -Action Allow

Set-NetFirewallHyperVVMSetting `
  -Name '{40E0AC32-46A5-438A-A0B2-2B649E9252E8}' `
  -DefaultInboundAction Allow
```

Mirrored mode itself is up (`eth0 == 192.168.3.8`, gateway pings, ARP
for the device resolves, WSL-internal self-test UDP works). Conclusion:
external→WSL UDP inbound is dropped somewhere in the Hyper-V vSwitch
layer despite the firewall rules. **Not yet tried**: `wsl --shutdown`
after the `Set-NetFirewallHyperVVMSetting` change. Try that before
declaring the path permanently broken.

Development workaround until then: listen on Windows directly.

```powershell
$listener = New-Object System.Net.Sockets.UdpClient 9999
while ($true) {
  $remote = New-Object System.Net.IPEndPoint([Net.IPAddress]::Any, 0)
  $bytes = $listener.Receive([ref]$remote)
  Write-Host ("{0}  from {1}" -f [Text.Encoding]::UTF8.GetString($bytes), $remote)
}
```

## Flash-cycle gotcha on S3 USB-OTG (espflash 4.4)

`espflash flash --monitor` consistently leaves the chip in DOWNLOAD
mode after a successful write (`rst:0x15 USB_UART_CHIP_RESET, boot:0x3
DOWNLOAD … waiting for download`). The CLAUDE.md note that the combined
`--monitor` mode skips that second reset no longer applies on
espflash 4.x. After every flash:

1. Press the device's physical **RESET** button to boot into the app.
2. Boot-time log lines are missed by the now-dead `flash --monitor`
   session — capture them with:

```sh
stty -F /dev/ttyACM0 raw -echo -hupcl 115200
timeout 30 cat /dev/ttyACM0 | tee /tmp/m5s3-boot.log
```

(Then press RESET while cat is running.) `-hupcl` keeps the kernel
from toggling DTR on open, which would re-trigger USB_UART_CHIP_RESET.

Also: re-flashing the same ELF prints `Segment … has not changed,
skipping write` and finishes in ~5 s. Touch a source line if a real
rewrite is required (CLAUDE.md has the full note).

## usbipd-win recovery

Prior session ended with `usbipd` refusing to bind because of a
stale `VBoxUSBMon` filter driver. Fixed by:

```powershell
sc.exe stop VBoxUSBMon
sc.exe delete VBoxUSBMon
winget uninstall usbipd
winget install --interactive --exact dorssel.usbipd-win
```

VirtualBox is not used on this machine. If it ever is, repair-install
VirtualBox first so `VBoxUSBMon` is recreated cleanly.

After install, the standard sequence works:

```powershell
usbipd list
usbipd bind --busid <BUSID>      # admin, once
usbipd attach --wsl --busid <BUSID>
```

If `attach` says "no WSL distribution," start a `wsl` shell in another
window first (the command looks for a running distro).

## Phase 0.7 — next session pickup

Plan is in `docs/phase_0_7_wifi_decoder_coexistence.md`. Status:
**not started**. The 7-step order in that doc still applies.

Suggested first move on the remote machine:

1. Branch off `main` (or reuse `phase06-sdkconfig-abs-path` —
   currently equal to main).
2. Step 1: `embedded-poc/embedded-shared/src/dual_core.rs` — split
   `init()` into:
   - `init(re_c1: *mut i16, im_c1: *mut i16)` — new, takes pointers
   - `init_with_static_basis()` — wrapper preserving today's behavior
     for `compute_bench` / `rx_wavsim`
   - Replace the two `static mut BASIS_*_C1` arrays with `AtomicPtr<i16>`s.
3. Cargo check on host **and** xtensa for `compute_bench` /
   `rx_wavsim` before moving on (these are the only users of the old
   API).
4. Step 2: `embedded-poc/m5stack-s3-app/src/decode_pipeline.rs` —
   drop the two `static mut BASIS_*` arrays, change `run()` to take
   four `&'static mut [i16]` slices.

Steps 3+ touch `main.rs` and need the live device for verification.

### Note: stale paths in the design doc

`docs/phase_0_7_wifi_decoder_coexistence.md` lines 245–253 reference
`/home/minoru/...` absolute paths. Fix when convenient — they don't
block work but are the same anti-pattern just landed at the sdkconfig
layer.

## Quick command reference

```sh
# Build (m5stack-s3-app)
cd embedded-poc/m5stack-s3-app
source ~/export-esp.sh
cargo build --release

# Flash + monitor 90 s
../scripts/flash-monitor.sh \
    target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-s3-app \
    logs/<tag>_$(date +%Y-%m-%d).log 90

# After flash: press RESET on the device, then peek boot log
stty -F /dev/ttyACM0 raw -echo -hupcl 115200
timeout 30 cat /dev/ttyACM0

# UDP listener (WSL-side, currently silent — see above)
../scripts/udp-log-listen.sh 9999 /tmp/m5s3-udp.log
```
