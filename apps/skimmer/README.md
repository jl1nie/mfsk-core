# SpyServer skimmer — sample app

Decodes FT8, FT4, FST4, WSPR, JT9, JT65, Q65 and JTTY from a
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

## Install (Windows and macOS)

**You need:** a SpyServer you can reach from this PC (an Airspy HF+, Airspy or
RTL-SDR behind [SpyServer](https://airspy.com/download/), on this LAN or
elsewhere), and a PC clock that is right to about a second, since every mode
decodes in time slots. The skimmer receives; it never transmits.

**1. Download** the installer for your system from the
[latest release](https://github.com/jl1nie/mfsk-core/releases/latest) (under
*Assets*; `X.Y.Z` is the release's version):

| platform | file |
|---|---|
| Windows 10/11 (x64) | `mfsk-skimmer-vX.Y.Z-windows-x64-setup.exe` |
| macOS (Apple silicon) | `mfsk-skimmer-vX.Y.Z-macos-arm64.dmg` |

There is no Intel Mac or Linux build. On those, build the command line version
below.

**2. Install.** Neither installer is signed with a publisher certificate, so each
system warns once.

- **Windows:** run the setup. If *Windows protected your PC* (SmartScreen)
  appears, choose *More info*, then *Run anyway*. It installs for the current user
  (no administrator rights) under `%LOCALAPPDATA%\mfsk skimmer` and adds *mfsk
  skimmer* to the Start menu. Settings and the database are in your user folders,
  not in the install folder (paths below). Uninstall from Windows' app list; your
  settings and database stay.
- **macOS:** open the disk image and drag the app to *Applications*. The app is
  signed ad hoc, not with a Developer ID, so it is not notarized and Gatekeeper
  stops the first launch.
  - macOS 14 and earlier: right-click the app and choose *Open*.
  - macOS 15 and later: open the app once (it is refused), then System Settings →
    Privacy & Security → scroll to *"mfsk skimmer" was blocked* → *Open Anyway*.
  - Either way, from a terminal:
    `xattr -dr com.apple.quarantine "/Applications/mfsk skimmer.app"`.

  The first connection to a SpyServer on the LAN asks for local network access;
  allow it. If it was refused, turn it on under System Settings → Privacy &
  Security → Local Network. A refusal shows in the app as `No route to host (os
  error 65)`. (The macOS build is made by CI; it has not been run on every Mac.)

**3. First run.** Open *Settings* (⚙), give the server's address (`host:5555`) and
your locator, add channels from the presets (they come from WSJT-X's frequency
list), and press *Connect*. Decodes appear in the list within the next slot (FT8
every 15 s; WSPR every 2 minutes). Where they are recorded is under *Analysis >
Database*.

**If it does not connect:** `No route to host` on macOS is the local network
permission above. `closed by the server (client limit?)` means the server's client
slots are taken (see *Sharing the radio with SDR#*). On a LAN, check that the
address and port are the ones SDR# uses.

**Updating:** run the newer installer over the old one; settings and the database
are kept. Each release carries the library fixes of its version, so the release
notes' *Fixed* items apply to the skimmer too.

The installers are built by `.github/workflows/skimmer-installers.yml` for every
release and attached to it; the same workflow runs on pull requests that touch
`apps/skimmer/` and by hand (*Run workflow*, which gives a branch's installers as
the run's artifacts). The sections below say how it works and how to build it
yourself.

## Command line (`cli/`)

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
QSO in progress, other) and the operator's call and grid under Settings > Server (per server) and apply to a running skimmer at
the channel's next slot, without a reconnect.

Options: `--yield` (leave control to an SDR# started later), `--tune` (hold control
and tune even with `--yield`), `--center HZ`,
`--slot-budget SHARE|off` (a slot's decode may run this share of its period, then stops and reports what it has; off by default; the status line counts the slots it cut), `--lanes N` (decoder threads per channel, 4 by default: the next slot is decoded on another thread while the last is still being decoded, even if that outlasts its slot; each thread has its own callsign table, and a channel with averaging uses one), `--no-early` (FT8 rows only when the slot ends; by default the rows found by ~11.8 s are printed then, as WSJT-X shows them, and the rest at the end), `--detail` (each row's sync score, the errors the FEC corrected and its message key),
`--rate S/s`, `--gain N` (the device gain index, when this client holds control;
the GUI sets it live under Settings → Radio), `--format float|int16`
(default float), `--pfb` (polyphase channelizer; cheaper from a handful of
channels on), `--iq-swap`, `--reanchor-ms MS` (default 500). Modes:
`FT8 FT4 FST4-15 FST4-30 FST4-60 FST4-120 FST4-300 WSPR JT9 JT65 Q65-15A
Q65-30A Q65-60A … Q65-300A JTTY`. JTTY has no slot: a message grows while it is
received, so it is printed (and written to ALL.TXT, as WSJT-X does) when it is over, its Rx
frequency and tolerance are `:rx=` and `:tol=` (1500 and 50 Hz, as upstream), and in the window it
has a panel of its own where a message is updated in place. Its sender and locator go into the
database and the station list; the calls are read by WSJT-X's own spotting rule.

### PSK Reporter (`--psk-reporter`, off by default)

```sh
skimmer --server HOST:5555 --ch FT8@14074000 --mycall CALL --mygrid GRID \
    --psk-reporter --psk-grid RXLOCATOR --psk-ant "3-el yagi"
```

Sends what it decodes as spots to [PSK Reporter](https://pskreporter.info/), by the protocol on
[its developer page](https://pskreporter.info/pskdev.html) (IPFIX over UDP; the packets are
byte-for-byte WSJT-X's, `skimmer_core::pskreporter`). **The datagrams go to PSK Reporter's test listener
(`pskreporter.info:14739`), which analyses them and records nothing**, until PSK Reporter's author has said yes
to production (#655). There is no option for it: `APP_ENDPOINT` in `skimmer_core::pskreporter` is the one
switch, for the CLI and the GUI both. `--psk-to HOST:PORT` sends to a listener of your own, to look at the
datagrams.

- The receiver is `--mycall` at `--psk-grid` (else `--mygrid`). **The locator is the antenna's**: for a
  SpyServer somewhere else it is not where you sit. `--psk-ant` and `--psk-rig` are free text.
- What is a spot is WSJT-X's rule, checked against WSJT-X's own code (`tests/pskreporter_spot_rules.rs`):
  a structured message (not free text: `i3 > 0 or n3 > 0` of the decoded bits, as `stdmsg`), the sender and
  locator by `DecodedText::deCallAndGrid` (so `K1ABC W9XYZ R EN37` is W9XYZ at EN37), a locator or a CQ; not
  low-confidence (`?`), not your own call, not your own transmission heard again; WSPR is not sent (that is
  WSPRnet's). JTTY: a completed message by WSJT-X's JTTY rule (`Radio::is_standard_callsign`, `/P` and `/R`
  included). A spot needs a time, so none are sent before the clock is known.
- A slot that began before the radio was last retuned (a rotation step, a move of the device) may hold the
  old band's samples and is not reported (WSJT-X waits four fifths of a period after a band change; this is
  exact, by the slot's own start).
- One timed datagram at most every five minutes, with a random addition (`--psk-interval SECS` only for the test
  listener or your own), and earlier only when a datagram has become full: PSK Reporter's author, asked in
  #655, says 1200 bytes or more counts as full, so datagrams are packed to 1200 bytes and the one that the
  next spot would overfill is sent at once; a callsign once per band per five minutes; the templates in the first three
  datagrams and every hour; one UDP socket, one source port. Spots pending when the program stops are not
  sent.

In the GUI the same switch is under each server (Settings > Server > PSK Reporter): it reports under the server's
**My call** at its **Grid**, which is the locator of that server's antenna, so each server is its own receiver on PSK
Reporter (PSK Reporter's author: one callsign per antenna location, and the locator is the antenna's, as here). The destination is `APP_ENDPOINT`, as for the CLI; there is no choice to make. The header shows what was sent and the last error; hover for the counts.

## GUI (`gui/`)

The GUI is a Tauri 2 shell (`gui/src-tauri`) around `skimmer-core` with a
Svelte 5 front end (`gui/src`). A server's button in the header holds its
on/off dot and its name; the channel list has band presets; the right side is a
**Waterfall** tab (a thumbnail per channel, one large, the decodes under it) and
an **Analysis** tab (below). The decode list uses one row format for every mode,
highlights CQ and your own call, and can filter by channel, CQ or a search
string. A row whose `<...>` the same period then resolved (a call learned from a later message) is replaced in place and marked ✓; *Detail* adds the sync score and the errors the FEC corrected, and hovering a row gives the message key. Settings are saved and restored. Every decode goes to the database
`skimmer.db`. The presets come from WSJT-X's `default_frequency_list` (all-region and
Region 3 entries), plus FT8 at 7041 kHz for JA.

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
The database (`skimmer.db`) and the health log `STATUS.log` are in `Documents\mfsk-skimmer\`. The database file is chosen in Analysis > Database
(**Open…** an existing one to add to it, **New…** for another), apart from the folder
of `STATUS.log`, which is chosen there too; the tab can also read any other file without
changing where the skimmer records. When Documents is redirected to
OneDrive, they sync there; change the folder in Analysis > Database if that is not wanted
(a database being written does not suit a synced folder well).
`STATUS.log` is capped: past 5 MB it becomes `STATUS.log.1` (one older generation, so at most about 10 MB in all), and a server that keeps failing the same way is one line and a count, not two lines every 10 s retry.

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
- **SDR# keeps its connection after *Stop*.** Once it has played, stopping the
  radio leaves the SpyServer connection open until SDR# is closed or its
  source is switched. It holds one of the server's slots, and control, all that
  time. To let the skimmer write the gain, close SDR# (not just Stop); to adjust
  the gain in SDR#, make sure the skimmer is a guest (its slider then shows
  SDR#'s value).
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

## Several servers, and a rotation of bands

Up to four SpyServers at once, wherever they are. Each has a name, an address,
its own call and locator (the origin of the bearings of what it hears), IQ
format, channelizer, network delay, gain and control; the channels of each are listed under it.
`Settings > Server` edits the one selected in the header; `+` adds another.
One SpyServer is one SDR with one centre and one IQ rate, so the way to cover
more bands is more servers, or a rotation on one.

A **rotation** gives each band of a server's channels its turn: one SDR holds
one band at a time, and the modes of a band (FT8, FT4, WSPR...) are heard
together. Each step is one band for at least four minutes, rounded up to a whole number of
the slots of its modes (WSPR's are two minutes, so five becomes six; a retune
costs a slot or two): `20m for 10 min, 40m for 10 min, 80m for 10 min, round again`. In the
GUI the steps are the bands the server's channels are in; you set the minutes and
the order. Connect begins with the first step (every server's cycle begins
together, on an even minute); "Rotation follows the UTC clock" counts the cycle
from UTC midnight instead, so a restart is at the same step at the same moment. The stream retunes at each step; the last slot of a step is
decoded before leaving, and a channel's decoder (with its callsign table) and
its waterfall history wait for the band's next turn.

Each band can also be given the **UTC hours it takes part in**, hour by hour
(24 dots to click, any set of hours: 7 MHz all day, 14 MHz only by day, 3.5 MHz
only by night, or just 03, 04 and 21). In the GUI, "day" runs from an hour before today's sunrise at the server to an hour after its
sunset, and "night" from an hour before sunset to an hour after sunrise, so the grey line (the
hours of dawn and dusk) is in both; without the server's locator they are 06-18 and 18-06 local time.
The sunrise and sunset are shown in Settings and marked on the hours strip. 
"Follow the calendar" (offered once day or night is chosen, with the hours of grey line either side, 1 by
default) makes the band follow each day's sunrise and sunset instead of today's hours: the rotation works
them out as it goes, from the server's locator (else yours), so the edges move a minute or two a day and
there is no moment at which they are renewed. The cycle goes on among the bands that are in at that
hour and starts again whenever the set changes; when none is in, nothing is heard
until one opens.

**On the screen.** A server's dot in the header switches that server on or off
(off closes its connection and keeps its settings and channels; the dot is hollow
when off). Beside a rotating server's channels the current band and the time left
are shown, `⟳ 20m 3:21`; pressing it **holds** the rotation on that band with no
retune (`⏸ 20m held`), and again lets it go on at the step the clock says. `‹` and
`›` on either side move to the previous or next band at once, even in the middle
of a turn; the band moved to gets its whole turn from that moment (held, it stays
held on the new band). A channel that is not being heard keeps, greyed, the number
of decodes of the last slot it was heard in.

The database records which server heard each decode and where that server is,
so Analysis can filter by server, centre the map on any of them, and measure
bearing and distance from the place that heard the signal.

## Analysis

Each decode is stored with the decoder's own view of it: the message's bits (the key, so the same message heard on two servers can be matched), the sync score, the errors the FEC corrected, and whether a `<...>` was resolved. Files from before have none of these (blank in the CSV).
One query (UTC period, regular expressions on call, locator and text, band, mode,
server, SNR, distance and bearing sector from the server that heard it, CQ kind)
over four views:

- **Map**: a great-circle map centred on home or on any server, or a Mercator one;
  stations coloured by SNR; "Animate" plays the period in windows of 1 to 60
  minutes, repeating. A station opened in Results is ringed.
- **Results**: the bands' openings (hour by band) above a list of stations or of
  decodes; a station opened shows a small map of its great circle and the hours
  it was heard in, by day.
- **Database**: the file read (another one can be opened here), its size and what is
  in it; clearing out decodes older than N days (all servers or one); compacting the file
  (deleting alone does not shrink it); a CSV of the decodes of a period you choose
  (the query's, the last day, week or month, everything, or from – to), with the query's
  filters, bearing and distance; a compact copy of the whole file, made while it records.

On the command line: `--server NAME=HOST:PORT` starts a server (its options
and `--ch` follow); `--step MINUTES` before the `--ch` heard in that step.

```
skimmer --server home=127.0.0.1:5555 --step 10 --ch FT8@14074000 --step 10 --ch FT8@7074000 \
        --server shack=192.168.1.20:5555 --ch FT8@28074000 --ch FT4@28180000
```

## Time

SpyServer sends no timestamps. The slot grid is anchored on the host clock:
the anchor is the minimum `arrival − samples/fs` over 30 s, since delay only
ever makes a message late. After that the device's sample clock keeps time,
and the skimmer re-anchors only when the estimate moves more than 500 ms.
Measured against an Airspy HF+ on the LAN: delivery delay 3–11 ms, drift 7 ms
in 9 min.

The host clock can itself be out (Windows syncs weekly by default), which
moves every station's DT. `--ntp HOST` (GUI: Settings > Clock > NTP, the
default, `pool.ntp.org`) has the skimmer measure its offset to an NTP server
(SNTP, the best of five queries by round trip, repeated every 10 minutes) and
add it to the host clock; the host clock itself is not changed, and if the
server cannot be reached the host clock is used and the health line says so.

The minimum removes the *varying* part of the delivery delay, not the fixed
part (the server's buffer, the path), which on the internet is tens to
hundreds of ms. `--net-delay MS` (GUI: Network delay) is taken off every
arrival time, live: if every station shows the same DT offset, enter it.

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
