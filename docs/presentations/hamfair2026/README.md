# ハムフェア2026 ブース資料（mfsk-core）

印刷を前提にした2種類の配布・説明資料。文言は `content.md` に外出ししてあり、
そこを直して `build.py` を走らせると HTML と PDF が作り直される。

| ファイル | 判型 | 用途 |
|---|---|---|
| `content.md` | — | **文言の正本。** 直すのは基本ここだけ |
| `build.py`   | — | `content.md` → HTML → PDF。はみ出し検査つき |
| `flipchart.html` | A4 横 · 10ページ | ブースで手元に置いてめくる説明資料。1ページ1トピックで、各ページ下端に「この1枚のまとめ」の帯がある |
| `leaflet.html`   | A4 縦 · 2ページ（両面1枚） | 来場者の持ち帰り用リーフレット。表＝概要・デモ・対応モード・感度、裏＝マルチプラットフォーム・組み込み・設計・参加のしかた |
| `qr-github.svg` | — | 両資料に埋め込んである `https://github.com/jl1nie/mfsk-core` の QR コード |

**PDF はリポジトリに入っていない。** 印刷する前に `build` を走らせて手元で焼く。
2026-08-25 まではコミットしていたが、文言を直しても同じマシンで焼き直せないと
古い PDF がそのまま残り、しかもそれが印刷される側だという状態になっていた。
成果物なので `.gitignore` に入れてある。

想定：ブースでの随時説明（5〜10分）、デモ機は M5Stack CoreS3 + IC-705（USB Audio 直結）。

## 文言を直す

```sh
cd docs/presentations/hamfair2026
$EDITOR content.md          # 好きなエディタでまとめて直す
python3 build.py build      # HTML と PDF を作り直す + はみ出し検査
```

`content.md` は「ID 見出し + 本文」の並びになっている。

```markdown
### F02-02 · 右肩の補足

M5Stack CoreS3
```

- `### F02-02 · 右肩の補足` の**次の行から次の見出しまで**が、その箇所の文言。
  **見出しの行は変えないこと** — `F02-02` が HTML 側の `data-t="F02-02"` と対応していて、
  これを見失うと差し込み先が分からなくなる。
- ID の読み方：`F` がフリップ資料、`L` がリーフレット。続く2桁がページ、末尾がページ内の順番。
- 装飾は `**太字**` / `` `等幅` `` / `==マーカー==` / `++アクセント色++` / `^小さめ^`（数値の単位）。
- **改行したいところで改行する。** そのまま改行として出る。ただし表と箇条書きの中だけは
  行が壊れるので `<br>` を使う。
- 表は Markdown の表のまま編集でき、行の増減もできる（列は増やせない）。
- 箇条書きは `- ` で始まる行。項目の増減もできる。
- ここに書いていない HTML タグはそのまま素通しする。分からないものは触らないのが安全。

**レイアウト（判型・段組・色・列幅・文字寸法）は HTML 側が持っている。** 文言では足りない
調整はそちらを直す。HTML を直したあとで文言側に反映したいときは:

```sh
python3 build.py extract    # HTML の現在の文言で content.md を作り直す
```

`build` は `content.md` を正、`extract` は HTML を正として上書きする。逆向きに走らせると
直したほうが消えるので、どちらを走らせるかは意識すること。

## はみ出しは目視でなく機械で見る

どちらの資料も `.page` に `overflow:hidden` が掛かっていて、溢れた分は**警告なしに切り捨てられる**。
`build` は毎回チェックして、次のどちらかを報告する。

```
  ✓ flipchart.html: 全ページ収まっています
  ⚠ flipchart.html p3: 「versus 44mm」分が箱に収まらず欠けています
```

ページ全体の溢れだけでなく、**中の箱が潰れて欠けた場合も検出する**（`.body` が flex なので、
文言を足したときの被害はページではなく内側のカードや表に出ることが多い）。⚠ が出たら、
その分だけ文言を削るか、HTML 側でそのページに `class="page dense"` を付けて寸法を一段落とす。
フリップ資料は情報量の多い4ページ（04・05・06・08）が既に `dense`。

検査には Playwright が要る（`pip install playwright`）。入っていなければ検査だけ飛ばして
PDF は作られるので、その場合は自分で PDF を見て確認すること。

## PDF について

`build.py` が Chromium のヘッドレス印刷で焼く。判型・ページ送り・余白はすべて HTML 側の
`@page` と `.page` で決まるので、コマンドラインで用紙を指定する必要はない。

**必要なもの**（`build` はどれが欠けても、その工程だけ飛ばして先に進む）:

```sh
pip install beautifulsoup4          # 必須。これが無いと import で落ちる
pip install playwright && playwright install chromium   # PDF 生成 + はみ出し検査
brew install poppler                # フォントのサブセット化（Linux なら apt install poppler-utils）
```

Chromium は `$CHROME` → Playwright のブラウザキャッシュ → `PATH` → macOS/Windows の
アプリバンドル、の順に探すので、どのマシンでも置き場所を書き換える必要はない。

**フォントを先に入れること。** 本文は `Noto Sans JP` で組んである。CSS は Hiragino /
Yu Gothic / Meiryo へフォールバックするので画面で読む分には困らないが、PDF を焼くなら
Noto が要る。無ければ `build` が警告する。macOS なら:

```sh
brew install --cask font-noto-sans-cjk-jp
```

加えて **静的**な `Noto Sans JP`（`notofonts/noto-cjk` リリースの `16_NotoSansJP.zip`）を
`~/Library/Fonts` に入れる。Homebrew の `font-noto-sans-jp` は可変ウェイト1ファイルで、
これだと Chromium が上手く扱えない。

**PDF のサイズは環境で大きく変わる。** Chromium の PDF 出力は、Linux では日本語グリフを
パス化して flipchart が約 1.5 MB になり、macOS では 2.19 MB のフォントを7本そのまま
埋め込んで約 11.6 MB になる。描画結果は同じで、格納方法が違うだけ。`build` は
`pdftocairo` があれば自動でサブセット化して 2.8 MB 程度まで落とす。

手で焼くなら:

```sh
chromium --headless --disable-gpu --no-pdf-header-footer \
         --print-to-pdf=flipchart.pdf file://$PWD/flipchart.html
```

焼いた PDF はどちらのやり方でも日本語を自前で持っている（Linux ではグリフがパスとして、
macOS ではフォントが埋め込まれて入る）ので、印刷環境に日本語フォントが無くても崩れない。

### 印刷

- **フリップ資料** — A4 横・片面・10枚。等倍（「ページに合わせる」ではなく100%）で印刷する。
- **リーフレット** — A4 縦・**両面**・短辺綴じでも長辺綴じでもよい（裏面は独立した1ページとして読める）。

HTML から直接印刷する場合は、ブラウザの印刷ダイアログで **余白なし** / **背景グラフィックを印刷する**
を指定する（`@page` で余白 0 を指定しているが、既定を上書きしてくるブラウザがある）。

## QR コードを差し替える

```sh
python3 -c "import segno; segno.make('https://example.com', error='m').save('qr-github.svg', scale=10, border=0, dark='#12233b')"
```

生成した SVG に `viewBox="0 0 290 290"` を足してから HTML に貼り込むこと。
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

crates.io の最新リリースは資料本文に版数を書いてある。リリースを打ったら `content.md` の
「最新リリース …」の2箇所（`F10-09` と `L02-28`）を更新すること。現在は v0.10.0。
