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

## TODO

- [ ] [keyberon](https://github.com/TeXitoi/keyberon)クレートの使用方法の習得
- [ ] AE-RP2040 への対応
- [ ] JIS キーボードへの対応
- [ ] キーマップ変更の手順確立
