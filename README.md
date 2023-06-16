# AE-RP2040 Rust 開発用テンプレート

[rp-rs/rp2040-project-template](https://github.com/rp-rs/rp2040-project-template)を自分が使いやすいように調整

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
