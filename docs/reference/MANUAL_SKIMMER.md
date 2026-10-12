# mfsk skimmer — user manual

Decode FT8, FT4, WSPR and the other WSJT-X modes from a SpyServer, on a PC, with no radio on the desk.

This is the manual for the Windows and macOS app (version 1.0). It describes what
you see and what each setting does. Why it works the way it does is in
[`apps/skimmer/README.md`](../../apps/skimmer/README.md); the command line version is there too.

> **A sample app.** It receives only, it has no support promise, and it was written to show how to drive
> `mfsk-core` from a live source. **PSK Reporter reporting is in it, off by default, and sends to the real
> PSK Reporter** (section 10). The installers are not signed with a publisher certificate, so each system warns once
> (section 3).

---

## 1. What it does

The skimmer connects to a [SpyServer](https://airspy.com/download/) (an Airspy HF+, an Airspy or an RTL-SDR behind it,
on your LAN or elsewhere), takes the radio's IQ stream, and decodes every channel you give it from that one stream. A
channel is a mode at a dial frequency (USB), for example `FT8  7 074.0 kHz`.

| Mode | Slot | Notes |
|---|---|---|
| FT8 | 15 s | rows from about 11.8 s into the slot, as WSJT-X shows them; the rest at the end |
| FT4 | 7.5 s | |
| FST4-15 / 30 / 60 / 120 / 300 | 15 to 300 s | |
| WSPR | 2 min | |
| JT9, JT65 | 1 min | |
| Q65 (15A, 30A, 60A to 60E, 120D, 120E, 300A) | 15 to 300 s | the sub-modes the presets and the mode list offer |
| JTTY | none | WSJT-X 3.2's keyboard mode: a message grows while it is received |

What it adds to a plain decoder:

- **Several channels at once**, from one stream: 7 041 kHz FT8, 7 074 kHz FT8 and 7 047.5 kHz FT4 cost one connection.
- **Several servers** (up to four) and a **rotation** of bands on a server, for one SDR that cannot hold every band at once.
- **Sharing the radio with SDR#**: it is the guest, and never moves the radio you are listening to.
- A **waterfall** of each channel, and a **database** of everything heard with a map, statistics and a time-lapse
  (section 9).

It receives; it never transmits.

## 2. What you need

- A **SpyServer** you can reach from the PC: an address like `192.168.1.26:5555`. On a LAN, it is the address and port
  SDR# uses.
- A **PC clock that is right to about a second**: every mode decodes in time slots (section 11).
- Windows 10 or 11 (x64), or macOS on Apple silicon. There is no Intel Mac and no Linux build; on those, build the
  command line version.
- Your callsign and your locator. They are used for the distance and bearing of what you hear, the sunrise and
  sunset, and (when you ask for it) the a-priori decoding of a QSO.

## 3. Install

Download the installer from the newest *mfsk skimmer* release
([all skimmer releases](https://github.com/jl1nie/mfsk-core/releases?q=skimmer)), under *Assets*:

| platform | file |
|---|---|
| Windows 10/11 (x64) | `mfsk-skimmer-vX.Y.Z-windows-x64-setup.exe` |
| macOS (Apple silicon) | `mfsk-skimmer-vX.Y.Z-macos-arm64.dmg` |

Neither is signed with a publisher certificate, so each system warns once.

### Windows

Run the setup. If *Windows protected your PC* appears, choose *More info*, then *Run anyway*. It installs for the
current user (no administrator rights) under `%LOCALAPPDATA%\mfsk skimmer` and adds *mfsk skimmer* to the Start menu.

Your settings and database are not in the install folder (section 12), so uninstalling from Windows' app list keeps
them, and so does installing a newer version over the old one.

### macOS

Open the disk image and drag the app to *Applications*. The app is signed ad hoc, not with a Developer ID, so it is
not notarized and Gatekeeper stops the first launch:

- **macOS 14 and earlier:** right-click the app and choose *Open*.
- **macOS 15 and later:** open the app once (it is refused), then System Settings → Privacy & Security → scroll to
  *"mfsk skimmer" was blocked* → *Open Anyway*.
- From a terminal, either way: `xattr -dr com.apple.quarantine "/Applications/mfsk skimmer.app"`.

**Allow local network access** when the first connection asks for it. If it was refused, turn it on under System
Settings → Privacy & Security → Local Network. A refusal shows in the app as `No route to host (os error 65)`.

### First run

1. Open **Settings** (the ⚙ at the top left). On the *Server* tab give the server an **Address** (`host:5555`), **My
   call** and **Grid** (your locator, or the antenna's if the SpyServer is elsewhere).
2. Close Settings, and under **Channels** pick a band in *Presets* and click the modes you want (`+ FT8 7074.0 kHz`).
3. Press **Connect**.

Decodes appear after the end of the next slot: FT8 every 15 s, WSPR every 2 minutes. Settings are saved and restored on the
next start.

---

## 4. The window

```
┌───────────────────────────────────────────────────────────────────────────────┐
│ ●Server 1  ●Server 2  +   ⚙  [Connect]   status: Decoding · delay · drift      │  header
├──────────────────────┬────────────────────────────────────────────────────────┤
│ Channels             │ [Waterfall] [Analysis]                                   │
│  Server 1            │  ┌thumb┐┌thumb┐┌thumb┐   one small waterfall per channel │
│   FT8 7074.0  ⚙ ✕   │  └─────┘└─────┘└─────┘                                   │
│   FT4 7047.5  ⚙ ✕   │  ┌─────────────────────┐ the chosen channel, large        │
│  ...                 │  │                     │                                  │
│  [FT8 ▾][dial kHz]   │  └─────────────────────┘                                  │
│  Presets [40m ▾]     │  Decodes: UTC  Mode  MHz  dB  DT  Message                 │
│   + FT8 7041.0 (JA)  │                                                          │
└──────────────────────┴────────────────────────────────────────────────────────┘
```

### The header

- **Server chips.** One per server, each with a dot and its name. The dot switches that server on or off (off
  closes its connection and keeps its settings and channels; the dot is hollow when off). Click the name to choose
  the server whose channels, gain and settings are shown. `+` adds a server (up to four).
- **⚙** opens Settings (section 6).
- **Connect / Disconnect** starts or stops every server that is switched on. Settings that change the connection
  (address, IQ format, channelizer, clock, the waterfall) are fixed while connected: *Disconnect to change these*.
- **The status** of the chosen server, in words and then in numbers:

| Phase | Meaning |
|---|---|
| `Connecting to …` | the TCP connection is being made |
| `Connected with control of the radio` / `Connected as a guest` | you may tune the radio and write its gain / SDR# (or another client) holds control (section 7) |
| `Decoding` | IQ is arriving; the line under it says the IQ rate, the centre frequency and the channelizer |
| `No channel fits the band around … MHz; waiting for the radio to move` | the radio is tuned where none of your channels is; it decodes again when the radio comes back |
| `Disconnected (…); retrying` | the connection failed; the reason is in the brackets and it tries again |

  After it, in small type: the **clock** in use (PC clock, or NTP), the stream's **delay** and **drift** against that
  clock, and the longest decode of the last period. A highlighted item means the clock is wrong or the stream
  cannot keep up (section 11). With PSK Reporter on, the count of spots sent is here too.

### Channels (left)

The channels of each server, with, for each channel, its mode and dial frequency, a small pie that fills during its
current slot (hover it for the time to the next boundary), the **number of decodes in the latest slot it was heard in** (grey when it is not heard
now: outside the radio's band, or out of its turn in a rotation), **⚙** (its options, section 5) and **✕** (remove).

Adding or removing a channel while connected **restarts the session** (it disconnects and connects again with the new
set), so the slot being received is lost. A channel's *options* do not: they apply at its next slot.

### The waterfall

With the waterfall on (Settings → *All servers*), the *Waterfall* tab shows a small waterfall for every channel of the
chosen server and one large one. Click a thumbnail to show that channel large; the decodes under it follow. Drag the
bar between them to change a height, double-click it to restore the usual. *Fine* (Settings) gives the large one 1.5 Hz
per bin instead of 2.9 Hz. The decodes are marked on the waterfall.

With the waterfall off, the tab is called *Decodes* and is the table alone.

### The decode table

One row format for every mode:

| Column | Meaning |
|---|---|
| UTC | start of the slot |
| Server | the server that heard it (shown only with more than one) |
| Mode | the mode of the channel |
| MHz | the signal's RF frequency, the dial plus its audio frequency |
| dB | SNR in 2 500 Hz, as WSJT-X reports it |
| DT | the signal's time offset from the slot, seconds |
| Message | the text |

**CQ** rows and rows with **your call** are highlighted. A ✓ at the end of a row says that a `<...>` in it was
resolved from a callsign learned in a later message of the same period (the row is replaced in place). *Detail* adds
**Sync** (the sync score) and **Err** (the errors the FEC corrected) and marks a row decoded early (~11.8 s) with ⏱.

Above the table: a channel selector (*All channels* or one), a *Search call or text* box, and *Clear*. Clear empties
the table; **the database keeps every decode** (section 9).

### JTTY

A JTTY channel has no slot: a message grows while it is received. Its list (selected the same way) has one row per
message, **replaced in place** as the message grows, completes, or is given up on. The sender and, when there is one,
its locator are put in the database and the station list.

---

## 5. Channels

### Adding

- **Presets.** Choose a band; the list is WSJT-X's default frequency list (all-region and Region 3 entries), plus
  FT8 at 7 041 kHz (JA) and JTTY's frequencies. Click `+ …` to add one; one already added is greyed.
- **By hand.** Choose the mode, type the **dial** in kHz (the USB dial, not the signal's frequency), press *Add*.

A channel is added to the server chosen in the header.

### Options (the ⚙ of a channel)

The options are WSJT-X's decoder parameters. They apply to a running skimmer **at the channel's next slot**, without
a reconnect. Leave a field empty for the mode's default.

| Group | Field | What it does |
|---|---|---|
| Search | Audio band (low–high) | the audio frequencies searched (`nfa`, `nfb`) |
| | Rx frequency, Tolerance | where the single-signal modes (JT9, Q65, JTTY) search, and how far around it |
| | Tx frequency | the Tx frequency (`nftx`); FT8 tries partner hypotheses near it |
| | Depth | `fast` / `normal` / `deep` (WSJT-X's `ndepth`; default deep) |
| A-priori | AP | off / CQ only / full. Default: FT8 and JT65 off, the others full |
| | DX call | hunt one station: its call is given to the decoder (FT8, FT4, FST4, Q65) |
| | My call, grid | this channel's own, when it differs from its server's |
| QSO in progress | His call, grid, progress | with *My call*, FT8, FT4 and FST4 derive WSJT-X's QSO-context a-priori decoding |
| | Contest | NA VHF / WW Digi / ARRL Digi / Q65 pileup, EU VHF, Field Day, RTTY Roundup, Fox, Hound |
| Other | Average periods | JT65 and Q65: sum the same station over periods |
| | Decode at 52 s | EME delay |

The ⚙ turns blue when a channel has options set; hover it for a summary.

---

## 6. Settings

Open with ⚙. The first tabs are **the chosen server's own** (they carry its name), the last applies to every server.

### Server

| Field | What it does |
|---|---|
| Name | the name on the chip; for you and for the database |
| Address | `host:5555` |
| My call | your callsign at this server; used for the a-priori decoding and for PSK Reporter |
| Grid | the locator **of this server's antenna**: bearings and distances, the sunrise and sunset, and the maps are measured from it. For a SpyServer somewhere else it is not where you sit |
| Sun today | sunrise and sunset at that locator |
| Network delay (ms) | the fixed delay from the SDR to this PC, taken off every arrival time. Zero on a LAN. If every station shows the same DT offset, enter it here |
| IQ format | float32, or int16 (half the bytes, for a slow link) |
| Channelizer | Auto, Direct, or Filter bank. Auto uses the filter bank from a handful of channels on |
| PSK Reporter | section 10 |
| Give control back… | section 7 |
| Gain | the radio's gain index; the slider writes it when you have control (section 7) |

### Rotation

Section 8.

### All servers

| Field | What it does |
|---|---|
| Clock | **PC clock**, or **NTP** with a server name. The PC clock is not changed: the offset to the NTP server is added to the stream's UTC |
| Rotation follows the UTC clock | section 8 |
| Decode threads | per channel; the next slot is decoded on another thread while the last is still being decoded (default 4) |
| Early decode | FT8 rows from ~11.8 s into the slot (default on) |
| Time budget | the share of a slot's period a decode may run before it stops and reports what it has; 0, the default, decodes every slot to the end |
| Waterfall, Fine | section 4 |

---

## 7. Sharing the radio with SDR#

The SpyServer gives **control** of the radio to one client at a time, and **SDR# takes it when it connects**, whichever
was first. Closing SDR# gives it back, with no reconnect.

- **Alone**, the skimmer has control, tunes the radio to its IQ centre, and can write the gain (Settings → Gain).
- **Beside SDR#**, it is the **guest**: it never tunes the radio or writes the gain (the slider shows what SDR# set).
  It places only its own IQ centre, which a guest may put anywhere in the radio's band, and it pauses the channels
  that fall outside that band (grey, `0`). You can start or close SDR# at any time.
- **SDR# keeps its connection after *Stop***, and with it a slot of the server and control, until SDR# is closed or its
  source is switched. To let the skimmer write the gain, close SDR#.
- To change band while the skimmer is a guest, retune in SDR#: the skimmer plans again around the new centre.
- **Give control back…** (Settings → Server) is the older behaviour: leave and retry 10 s later when given control.
  It is no longer needed to let SDR# in, and is off by default.

The server counts a client until it hears the connection close. A laptop that sleeps with the skimmer still connected
can keep its slot. **Disconnect before sleeping the machine.** A connection the server closes at once reads
`closed by the server (client limit?)`.

## 8. Several servers, and a rotation of bands

**Several servers.** `+` in the header adds a server (up to four), each with its own address, call, locator, format,
channelizer and gain; the database records which one heard what. One SpyServer is one SDR with one centre and one IQ
rate, so more bands at once means more servers, or a rotation.

**Rotation** (Settings → Rotation → *Rotate through the bands of its channels*). It gives each band of the server's
channels its turn: `20m for 10 min, 40m for 10 min, 80m for 10 min, round again`. The modes of a band are heard
together.

- Each step is at least **4 minutes**, rounded up to a whole number of the slots of its modes (WSPR's are 2 minutes,
  so 5 becomes 6). A retune costs a slot or two. The last slot of a step is decoded before leaving.
- Reorder the bands with ▲ ▼. *Rotation follows the UTC clock* counts the cycle from UTC midnight, so a restart is at
  the same step at the same moment.
- **Hours.** Click the UTC hours a band takes part in (24 dots): 7 MHz all day, 14 MHz by day, 3.5 MHz by night, or
  any set. *day* and *night* follow the sun at the server's locator (an hour before sunrise to an hour after sunset,
  and the reverse), so the grey line is in both. *Follow the calendar* makes the band follow each day's sunrise and
  sunset instead of today's hours.
- **On the screen**, beside the channels: `⟳ 20m 3:21`, the band and the time left. Click it to **hold** on that band
  (`⏸ 20m held`) and again to let it go on. `‹` and `›` move to the previous or next band now; the new band gets its
  whole turn from that moment.

---

## 9. Analysis

Every decode is recorded in a database file (`skimmer.db`) and the **Analysis** tab reads it, so it covers everything
heard, not only what is in the table.

**The query bar** narrows what the three views show: a **time range** (presets, or from–to), **call**, **grid** and
**message text** as regular expressions (`^JA` for calls starting JA; `^(JA|JH)\d`), the kind of **CQ** (any, every CQ,
plain, DX, POTA, SOTA, TEST), the **bands** and **modes**, an **SNR** range, a **distance** from your grid, a
**bearing** (click a direction on the compass), and the **server**. *Search* applies it, *Reset* clears it.

- **Map.** The stations heard, on a world map centred on your locator (or on a server), with the great-circle paths,
  coloured by band when two or more are chosen. *grey line* shows day and night. **Play** runs the period as a
  time-lapse in windows of 1 to 60 minutes, at 1 to 16 windows a second.
- **Results.** A list of **stations** (call, grid, bands, how many times heard, best SNR, distance, first and last
  heard) and, for a station you open, each of its decodes and the hours of the day it is on the air.
- **Database.** The file: *Open another…* to read a different one without changing where the skimmer records; whether to
  keep decodes at all; the folder of `STATUS.log`; **Clear out old decodes** and **Compact the file**; and **Export** (the
  query, the last 24 hours, 7 or 30 days, or everything).

The file and the folder are chosen here, while disconnected. When Documents is synced to OneDrive the database syncs
there; a database being written does not suit a synced folder well, so choose another folder if that is not wanted.

## 10. PSK Reporter

The skimmer can send what it decodes to [PSK Reporter](https://pskreporter.info/) as spots, with the packets WSJT-X
sends (IPFIX over UDP). **Since 1.0.0 they go to PSK Reporter itself**: its author has said yes to the skimmer going to
production. There is no setting for the destination; it is fixed in the program. It is **off by default**. (The 0.9.x
beta sent to PSK Reporter's test listener, which records nothing, and the window said *Test only*; that wording is gone.)

If you turn it on (Settings → Server → *PSK Reporter*) your spots appear on PSK Reporter under the call and locator you
set, so check them first; the header shows how many spots were sent and the last error, and hovering it gives the counts.

- It needs **My call** and a **Grid** of at least four characters, and reports under them: **the locator is the
  antenna's**, so for a SpyServer somewhere else it is not where you sit. Each server is its own receiver on PSK
  Reporter (one callsign per antenna location).
- *Antenna* is free text in the receiver record.
- What is reported follows WSJT-X's rule for a spot (FT8, FT4, FST4, JT9, JT65, Q65 and JTTY; **not WSPR**). One
  datagram at most every five minutes, earlier only when one is full, a callsign once per band per five minutes.
  Spots still waiting when you disconnect are not sent.

## 11. Time, and why the PC clock matters

Every slotted mode decodes in time slots, so the slots must line up with UTC: a PC clock off by more than about a
second loses the start of signals or shifts every DT. The skimmer stamps the IQ with UTC.

- Use *Clock → NTP* (Settings → *All servers*) if the PC clock is not kept right. The PC clock itself is not changed.
- The status line shows the clock in use, the stream's **delay**, and its **drift** against the clock. If **every**
  station shows the same DT offset, the fixed delay from the radio to the PC is not zero: enter it as *Network delay*
  in the server's settings.
- Connect does not wait on an NTP server that does not answer: it uses the PC clock, and the status line says `NTP … failed … using the PC clock`.
- A highlighted drift, or a longest decode longer than the slot, means the PC cannot keep up: lower the number of
  channels, the *Decode threads*, or the Depth of the heaviest channels.

## 12. Where things are

| What | Windows | macOS |
|---|---|---|
| Settings | `%APPDATA%\io.github.jl1nie.mfsk-skimmer\settings.json` | `~/Library/Application Support/io.github.jl1nie.mfsk-skimmer/` |
| Database `skimmer.db`, health log `STATUS.log` | `Documents\mfsk-skimmer\` | `~/Documents/mfsk-skimmer/` |

Both are changed in Analysis → Database. `STATUS.log` is a line per health event for a long run; past 5 MB it becomes
`STATUS.log.1` (one older generation), and a server that keeps failing the same way is one line and a count. The GUI
does not write an `ALL.TXT`; the command line version does (`--log`).

## 13. Troubleshooting

| What you see | What it means |
|---|---|
| `No route to host (os error 65)` on macOS | local network access was refused: System Settings → Privacy & Security → Local Network |
| `closed by the server (client limit?)` | the server's client slots are taken (SDR# open, or a skimmer that slept while connected); close one |
| It never connects | the address and port must be the ones SDR# uses; try from the PC with SDR# first |
| `No channel fits the band around … MHz` | the radio was tuned (by SDR#) where none of your channels is; retune it, or add a channel on that band |
| Connected, but no decodes | the clock (section 11); the dial is not the signal's frequency; the band is closed; the gain is too low |
| Every station has the same DT | enter the *Network delay* (section 6) |
| Channels greyed with `0` | outside the radio's band, or out of its turn in a rotation |
| macOS blocks the app | section 3 |
| Windows warns *protected your PC* | section 3 |

## 14. The limits

- Windows 10/11 x64 and macOS on Apple silicon only; unsigned installers.
- It is a sample app. Reports and questions are welcome as GitHub issues, but there is no support promise.
- The macOS build is made by CI and has not been run on every Mac.
- PSK Reporter reporting sends to the real service, under the call and locator you set (section 10).
- The command line version (`skimmer`) has every decoder option and writes an `ALL.TXT`-style log; see
  [`apps/skimmer/README.md`](../../apps/skimmer/README.md).
