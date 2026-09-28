# mfsk-core

[![CI](https://github.com/jl1nie/mfsk-core/actions/workflows/ci.yml/badge.svg)](https://github.com/jl1nie/mfsk-core/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/mfsk-core.svg)](https://crates.io/crates/mfsk-core)
[![docs.rs](https://img.shields.io/docsrs/mfsk-core)](https://docs.rs/mfsk-core)
[![License](https://img.shields.io/badge/license-GPL--3.0--or--later-blue.svg)](LICENSE)

English: [README.md](README.md)

## これは何か

`mfsk-core` は、**WSJT-X のデジタルモードを、ポータブルかつ高速な純 Rust
実装として提供し、本家のリファレンスデコーダーとの比較で検証するライブラリ**です。
FT8、FT4、FST4、WSPR、JT9、JT65、Q65(全10サブモード)の復調・符号化・波形合成を、
共通部品(DSP、同期相関、LLR、LDPC / 畳み込み / Reed-Solomon / QRA の各 FEC、
メッセージコーデック)の上に1つのクレートとして実装しています。
デスクトップ、ブラウザ上の WASM、Android/iOS、`no_std` の組み込み MCU など、
Rust が動く場所ならどこでも動きます。

アプリケーションではなくライブラリで、GUI は持ちません。狙いは、WSJT-X 品質のデコーダーを
自分のシステムに組み込めるようにすることです。このプロジェクトが継続して保っているのは
コードそのものよりも、その周りの検証です。

- WSJT-X 由来の golden 録音(全 PR がこれでゲートされ、recall と誤検出を同時に確認)
- リリース前に WSJT-X 自身のシミュレータを使って行う感度スイープ
- 数値の記録: [`docs/notes/BENCHMARKS.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/BENCHMARKS.md)

個人プロジェクトであり、本家 WSJT-X への追従はベストエフォートです。時期の約束はしません。

## 組み込みでの動作

リポジトリには、Xtensa LX7 上でこのライブラリを動かす受信機が2つ含まれます。

- [`embedded-poc/m5stack-cores3-app`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/m5stack-cores3-app/):
  主ターゲット。FT8(無線機からの USB Audio)、WSPR、FST4、無線機不要のデモの4受信機を
  1イメージに収め、タッチパネルで切り替えます。2026-08-23 に IC-705 の 40 m で確認し、
  1スロットあたり FT8 を 6〜8 局デコードしました。
- [`embedded-poc/m5stack-s3-app`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/m5stack-s3-app/):
  M5StickS3 のデモ(LCD UI、IC-705 への BLE CI-V、マイク経由の音響入力、QSO FSM)。

## 詳細

対応プロトコル、機能フラグ、設計方針、使い方は [README.md](README.md)(英語)と
`docs/reference/`(`.ja.md` の日本語版あり)を参照してください。

すべてのアルゴリズムは [WSJT-X](https://sourceforge.net/projects/wsjt/)(Joe Taylor K1JT
ほか)の Rust による再実装であり、WSJT-X が引き続きリファレンス実装です。
ライセンスは GPL-3.0-or-later です。
