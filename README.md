# AE-RP2040 Rust 開発用テンプレート

AE-RP2040 で動作するキーボードファームウェア検証用リポジトリ

## usage

### 環境準備

```power shell
rustup target install thumbv6m-none-eabi
cargo install flip-link probe-run
cargo install --git https://github.com/rspeir/elf2uf2-rs --branch branch2
```

### ビルド

```power shell
cargo build
```

### 書き込み

```power shell
cargo run
```

## todo

- [ ] [keyberon](https://github.com/TeXitoi/keyberon)クレートの使用方法の確認
- [ ] AE-RP2040 への対応
- [ ] JIS キーボードへの対応
- [ ] キーマップ変更の手順確立

### [keyberon](https://github.com/TeXitoi/keyberon)クレートの使用方法の確認

[TeXitoi/keyberon-f4: A handwired unsplitted ergo keyboard](https://github.com/TeXitoi/keyberon-f4)

[camrbuss/pinci: Super thin split PCB keyboard using rp2040 chips running Rust](https://github.com/camrbuss/pinci)

[DrewTChrist/nibble-rp2040-rs: This is a Keyberon based firmware for the Nibble keyboard paired with different RP2040 boards.](https://github.com/DrewTChrist/nibble-rp2040-rs)

[mryndzionek/keyberon-atreus: Keyboard firmware for my Atreus-like keyboard written in Rust and using Keyberon](https://github.com/mryndzionek/keyberon-atreus)

機能が少な目

[camrbuss/lily58-rp2040-keyberon: Keyboard firmware for Lily58 Pro with Sparkfun Pro Micro RP2040](https://github.com/camrbuss/lily58-rp2040-keyberon)

[rustberry-pi-pico/src/examples/keyberon-one-key-keyboard at main · radlinskii/rustberry-pi-pico · GitHub](https://github.com/radlinskii/rustberry-pi-pico/tree/main/src/examples/keyberon-one-key-keyboard)

## memo

### key code

- [USB デバイスの定義](https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf)
- [キー配列比較表 | Electronic Information Research Laboratory](https://www.minagi.jp/2020/09/10/keyboardlayout_jis_us/)

## rtic::app
