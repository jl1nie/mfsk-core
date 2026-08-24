# ハムフェア2026 ブース資料（mfsk-core）

印刷を前提にした2種類の配布・説明資料。どちらも 1ファイル完結の HTML で、
ブラウザから直接印刷しても、同梱の PDF をそのまま印刷しても同じ結果になる。

| ファイル | 判型 | 用途 |
|---|---|---|
| `flipchart.html` / `.pdf` | A4 横 · 10ページ | ブースで手元に置いてめくる説明資料。1ページ1トピックで、各ページ下端に「この1枚のまとめ」の帯がある |
| `leaflet.html` / `.pdf`   | A4 縦 · 2ページ（両面1枚） | 来場者の持ち帰り用リーフレット。表＝概要・デモ・対応モード・感度、裏＝マルチプラットフォーム・組み込み・設計・参加のしかた |
| `qr-github.svg`           | — | 両資料に埋め込んである `https://github.com/jl1nie/mfsk-core` の QR コード |

想定：ブースでの随時説明（5〜10分）、デモ機は M5Stack CoreS3 + IC-705（USB Audio 直結）。

## 印刷

- **フリップ資料** — A4 横・片面・10枚。等倍（「ページに合わせる」ではなく100%）で印刷する。
- **リーフレット** — A4 縦・**両面**・短辺綴じでも長辺綴じでもよい（裏面は独立した1ページとして読める）。

PDF は Noto Sans CJK JP をサブセット埋め込みしてあるので、印刷環境に日本語フォントがなくても崩れない。
HTML から直接印刷する場合は、ブラウザの印刷ダイアログで **余白なし** / **背景グラフィックを印刷する** を指定する
（`@page` で余白 0 を指定しているが、既定を上書きしてくるブラウザがある）。

## PDF の再生成

HTML を編集したら、Chromium のヘッドレス印刷でそのまま焼き直せる。

```sh
cd docs/presentations/hamfair2026
chromium --headless --disable-gpu --no-pdf-header-footer \
         --print-to-pdf=flipchart.pdf file://$PWD/flipchart.html
chromium --headless --disable-gpu --no-pdf-header-footer \
         --print-to-pdf=leaflet.pdf   file://$PWD/leaflet.html
```

判型・ページ数・余白はすべて HTML 側の `@page` と `.page` で決まるので、コマンドラインで用紙を指定する必要はない。

**ページからはみ出していないかは、目視ではなく測ること。** どちらの資料も
`.page` に `overflow:hidden` が掛かっていて、溢れた分は警告なしに切り捨てられる。
文言を足したあとは、印刷メディアで各ページの `scrollHeight` と `clientHeight` を比べる:

```js
[...document.querySelectorAll('.page')].map(el => el.scrollHeight - el.clientHeight)  // 全部 0 なら収まっている
```

フリップ資料は情報量の多い4ページ（04・05・06・08）だけ `class="page dense"` で
一段小さい文字寸法を当てている。文言を足して溢れたら、まず `dense` を付けるか外すかで調整する。

## QR コードを差し替える

```sh
python3 -c "import segno; segno.make('https://example.com', error='m').save('qr-github.svg', scale=10, border=0, dark='#12233b')"
```

生成した SVG に `viewBox=\"0 0 290 290\"` を足してから HTML に貼り込むこと。
`viewBox` がないと、CSS で `width` を指定したときに縮小ではなく**左上の切り抜き**になり、
読み取れない QR が刷り上がる。

## 数字の出どころ

資料に載っている測定値はすべてリポジトリ内の記録から取っている。更新するときは元の文書も確認すること。

| 資料上の記述 | 出典 |
|---|---|
| 各モードの再現率・誤デコード・AWGN 感度 | `docs/notes/BENCHMARKS.md` |
| ホストの復調時間（Ryzen 9 9900X） | 同上「Decode speed」節 |
| ESP32-S3 1.19 s / Core2 2.8 s | `docs/reference/EMBEDDED.md`「vs host wide-band on the WSJT-X reference」 |
| 必要 RAM 約150 KB / フラッシュ 150–200 KB / 内部 DRAM スクラッチ 0 | `docs/reference/EMBEDDED.md` 冒頭・「Binary footprint」節 |
| WSPR 1214 s → 101.6 s | `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md` |
| CoreS3 実機 6〜8局/スロット、+8〜−24 dB（2026-08-23） | `docs/reference/MANUAL_M5STACK_CORES3.md`、`docs/notes/ROADMAP.md` Phase B-Core |
| 4受信機を1イメージ・タッチ切替・USB ホストの電源規則 | `docs/reference/MANUAL_M5STACK_CORES3.md` |
| `src/` の 18.7k / 58.9k 行がジェネリック | `README.md`「Why a `Protocol` trait」 |

crates.io の最新リリースは資料本文に版数を書いてある。リリースを打ったら両ファイルの
「最新リリース v0.9.1、0.10.0 準備中」の記述を更新すること。
