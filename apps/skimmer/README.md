# SpyServer skimmer — sample app

Decodes FT8, FT4, FST4, WSPR, JT9, JT65 and Q65 from a
[SpyServer](https://airspy.com/download/) (Airspy HF+, Airspy, RTL-SDR)
through `mfsk_core::iq::IqReceiver`. Every channel is a mode at a USB dial
frequency, and all channels in the IQ span are decoded from one stream.

| path | what | builds in |
|---|---|---|
| `core/` | `skimmer-core`: SpyServer client, stream planner, time anchor, event loop | the workspace (CI lint + tests) |
| `cli/` | `skimmer`: prints decodes, appends an `ALL.TXT`-style log | the workspace (CI lint) |
| `gui/` | Tauri + Svelte front end over `skimmer-core` (v1: connection, channels, decodes, settings, log) | outside the workspace (needs a webview and Node) |

This is sample code. It shows how to drive the library from a live source.
It is not a supported application, and `mfsk-core` itself has no network
code.

## Run

```sh
cargo run --release -p skimmer -- --server 192.168.1.26:5555 \
    --ch FT8@7041000 --ch FT8@7074000 --ch FT4@7047500 --log ALL.TXT
```

Options: `--tune` (take control of the device and tune it), `--center HZ`,
`--rate S/s`, `--gain N` (with `--tune`), `--format float|int16`
(default float), `--pfb` (polyphase channelizer; cheaper from a handful of
channels on), `--iq-swap`, `--reanchor-ms MS` (default 500). Modes:
`FT8 FT4 FST4-15 FST4-30 FST4-60 FST4-120 FST4-300 WSPR JT9 JT65 Q65-15A
Q65-30A Q65-60A … Q65-300A`.

## Sharing the radio with SDR#

Observed with SDR# and an Airspy HF+ (2026-10-03):

- Control goes to the first client that connects. **Start SDR# first.** If
  the skimmer gets control without `--tune`, it leaves and reconnects 10 s
  later, which hands control to SDR#.
- While a second client is connected, the controlling client cannot move the
  device outside its band (about 780 kHz on the HF+). SDR# can still tune
  anywhere inside that band, for example all of 40 m. **To change band, stop
  the skimmer, retune in SDR#, then start the skimmer again.**
- The skimmer never tunes the device. It sets only its own IQ (DDC) centre,
  which a client without control may place anywhere in the device's band.
  It uses the lowest rate that holds the most channels, and pauses the
  channels that fall outside the band.

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
