# SpyServer skimmer — sample app

Decodes FT8, FT4, FST4, WSPR, JT9, JT65 and Q65 from a
[SpyServer](https://airspy.com/download/) (Airspy HF+, Airspy, RTL-SDR)
through `mfsk_core::iq::IqReceiver`. Every channel is a mode at a USB dial
frequency, and all channels in the IQ span are decoded from one stream.

| path | what | builds in |
|---|---|---|
| `core/` | `skimmer-core`: SpyServer client, stream planner, time anchor, event loop | the workspace (CI lint + tests) |
| `cli/` | `skimmer`: prints decodes, appends an `ALL.TXT`-style log | the workspace (CI lint) |
| `gui/` | Tauri + Svelte front end over `skimmer-core` (v1: connection, channels, decodes, settings, log); see below | outside the workspace (needs a webview and Node) |

This is sample code. It shows how to drive the library from a live source.
It is not a supported application, and `mfsk-core` itself has no network
code.

## Run

```sh
cargo run --release -p skimmer -- --server 192.168.1.26:5555 \
    --ch FT8@7041000 --ch FT8@7074000:band=300-3000:dx=JA1ABC:depth=normal \
    --ch FT4@7047500 --log ALL.TXT
```

Per-channel options follow the dial (WSJT-X's parameter block, `--mycall`/`--mygrid` for the operator): `:band=LO-HI` (audio Hz searched),
`:dx=CALL` (an a-priori hint for that station, FT8/FT4/FST4/Q65) and
`:depth=fast|normal|deep` (WSJT-X's `ndepth`; default deep), `:rx=`/`:tol=`/`:tx=`
(Hz), `:ap=off|cq|full`, `:hiscall=`/`:hisgrid=`/`:progress=0..5` (the QSO the
QSO-context AP is derived from), `:contest=NAME`, `:avg=1`, `:deepsearch=1`,
`:eme=1`. In the GUI they are under each channel's ⚙ dialog (search, a-priori,
QSO in progress, other) and the operator's call and grid under Settings and apply to a running skimmer at
the channel's next slot, without a reconnect.

Options: `--yield` (leave control to an SDR# started later), `--tune` (hold control
and tune even with `--yield`), `--center HZ`,
`--rate S/s`, `--gain N` (the device gain index, when this client holds control;
the GUI sets it live under Settings → Radio), `--format float|int16`
(default float), `--pfb` (polyphase channelizer; cheaper from a handful of
channels on), `--iq-swap`, `--reanchor-ms MS` (default 500). Modes:
`FT8 FT4 FST4-15 FST4-30 FST4-60 FST4-120 FST4-300 WSPR JT9 JT65 Q65-15A
Q65-30A Q65-60A … Q65-300A`.

## GUI (`gui/`)

The GUI is a Tauri 2 shell (`gui/src-tauri`) around `skimmer-core` with a
Svelte 5 front end (`gui/src`). It has a connection bar, a channel list with
band presets, and a decode list. The decode list uses one row format for
every mode, highlights CQ and your own call, and can filter by channel, CQ
or a search string. Settings are saved and restored, and decodes are
appended to an `ALL.TXT`. The presets come from WSJT-X's
`default_frequency_list` (all-region and Region 3 entries), plus FT8 at
7041 kHz for JA.

### Installers

Each release attaches a Windows installer (`mfsk-skimmer-vX.Y.Z-windows-x64-setup.exe`)
and an Apple silicon disk image (`mfsk-skimmer-vX.Y.Z-macos-arm64.dmg`), built by
`.github/workflows/skimmer-installers.yml`. The same workflow runs on pull
requests that touch `apps/skimmer/`.

The macOS app is signed ad hoc, not with a Developer ID, so it is not
notarized. On first open, right-click it and choose *Open*. The first
connection to a SpyServer on the LAN asks for local network access; allow it.
If it was refused, turn it on under System Settings → Privacy & Security →
Local Network. A refusal shows in the app as `No route to host (os error 65)`.

### Building on Windows

Build the front end on the WSL side, then the app on Windows (WebView2):

```sh
cd apps/skimmer/gui && npm install && npm run check && npm run build   # WSL
```

```powershell
# Windows PowerShell; the source can stay on the WSL share
$env:CARGO_TARGET_DIR = "C:\Users\<you>\skimmer-gui-target"
Set-Location \\wsl.localhost\Ubuntu-24.04\home\<you>\src\mfsk-core\apps\skimmer\gui
cargo tauri build      # app + NSIS installer under release\bundle\nsis
```

The settings file is `%APPDATA%\io.github.jl1nie.mfsk-skimmer\settings.json`.
The default log path is `Documents\mfsk-skimmer\ALL.TXT`. When Documents is
redirected to OneDrive, the log syncs there; change the path in Settings if
that is not wanted.

Verified on Windows against the SpyServer below, beside SDR#: 26 FT8
decodes in one slot at 7041 kHz, CQ rows highlighted, and the log and the
settings both written. `cargo tauri dev` against `npm run dev` on WSL has
not been tried.

### Building on macOS

```sh
cd apps/skimmer/gui
npm ci && npm run check && npm run build
npm run tauri -- build --bundles app,dmg   # under src-tauri/target/release/bundle
```

Run outside CI, the DMG step drives Finder over AppleScript to lay out the
window and can fail when Terminal may not automate Finder; `CI=true` skips
that step, and the `.app` under `bundle/macos` is complete either way.
Settings go to `~/Library/Application Support/io.github.jl1nie.mfsk-skimmer/`.

`src-tauri/Info.plist` carries `NSLocalNetworkUsageDescription`, and
`tauri.conf.json` signs the bundle ad hoc (`signingIdentity: "-"`). Both are
needed. Without the description, or with only the linker's signature (a random
identifier, Info.plist not bound), macOS refuses the LAN connection with
`No route to host` and never asks. Verified on an Apple silicon Mac against the
SpyServer below: it connects once local network access is allowed.

## Sharing the radio with SDR#

Observed with SDR# and an Airspy HF+ (2026-10-03):

- The server gives control to one client at a time, and **SDR# takes it when it
  connects, whichever was first**: with the skimmer holding control, starting
  SDR# made the skimmer's next sync read `control=false` and SDR# moved the
  device (2026-10-03 23:46); closing SDR# returned control to the skimmer
  (`control=true`) with no reconnect. So SDR# can be started or closed at any
  time. The skimmer follows, and is a guest while SDR# has control.
  - **Alone**: the skimmer has control, tunes the radio to its IQ centre, and
    can write the gain (Settings → Radio).
  - **Beside SDR#**: the skimmer is the guest and never tunes the device or
    writes the gain; the slider shows what SDR# set. It sets only its own IQ
    (DDC) centre, which a client without control may place anywhere in the
    device's band. It uses the lowest rate that holds the most channels, and
    pauses the channels that fall outside the band.
  - `--yield` (GUI: *Give control back*) is the older behaviour: given control,
    leave and reconnect 10 s later. It woke the radio and dropped it on every
    retry, and is no longer needed to let SDR# in, so it is off by default.
- While a second client is connected, the controlling client cannot move the
  device outside its band (about 780 kHz on the HF+). SDR# can still tune
  anywhere inside that band, for example all of 40 m. To change band while the
  skimmer holds control, retune in SDR#: it takes control, and the skimmer
  plans again around the new centre (channels outside the band pause).

- The server counts a client until it hears the connection close. A laptop that
  sleeps with the skimmer still connected can keep its slot (the network
  hardware answers the server's keepalives), and SpyServer has no idle
  timeout. **Disconnect before sleeping the machine** — with a limit of 2 and
  SDR# connected, one sleeping skimmer leaves no room for another. The skimmer
  reconnects by itself after a stall, so waking is not a problem; the slot held
  while asleep is. A connection closed by the server at once reads
  `closed by the server (client limit?)`.

## Time

SpyServer sends no timestamps. The slot grid is anchored on the host clock:
the anchor is the minimum `arrival − samples/fs` over 30 s, since delay only
ever makes a message late. After that the device's sample clock keeps time,
and the skimmer re-anchors only when the estimate moves more than 500 ms.
Keep the host on NTP. Measured against an Airspy HF+ on the LAN: delivery
delay 3–11 ms, drift 7 ms in 9 min.

## Health

The status line, once a minute of samples, reports:

- `delay`: how late the last message arrived
- `drift`: how far the anchor estimate has moved
- `longest push`: a slot's decode runs inside `push`
- `queue`: bytes read and not yet decoded
- the gap and re-anchor counts

The socket is read on its own thread, so a long decode does not stop the
reads. Before that change, the first decode (400–650 ms) made the server drop
IQ messages on Windows.
