#![no_std]
#![no_main]

// mod demux_matrix;
// mod encoder;
// mod layout;

// ブートローダー機能はいらんかなと
// #[link_section = ".boot2"]
// #[used]
// pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
use ae_rp2040 as bsp;

// #[rtic::app(device = bsp::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1, PIO1_IRQ_0])]
#[rtic::app(device = bsp::hal::pac, peripherals = true)]
mod app {
    use cortex_m::prelude::{
        _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogEnable,
    };
    use defmt_rtt as _;
    use embedded_time::duration::Extensions;
    use embedded_time::rate::Extensions as RateExtensions;
    use panic_probe as _;

    // AE-2040用に変更
    use ae_rp2040 as bsp;
    use bsp::hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{bank0::*, dynpin::DynPin},
        pac,
        pac::{I2C0, PIO0},
        pio::{PIOExt, SM0, SM1},
        sio::Sio,
        timer::{Alarm0, Timer},
        usb::UsbBus,
        watchdog::Watchdog,
    };

    use core::iter::once;

    // 作成の必要があるモジュール
    // use crate::demux_matrix::DemuxMatrix;
    // use crate::encoder::Encoder;
    // use crate::layout as kb_layout;

    // ディスプレイ関係なので使わない
    // use display_interface_i2c::I2CInterface;
    // use embedded_graphics::{
    //     image::{Image, ImageRaw},
    //     mono_font::{ascii::FONT_7X14_BOLD, MonoTextStyleBuilder},
    //     pixelcolor::BinaryColor,
    //     prelude::*,
    //     text::{Baseline, Text},
    // };

    use keyberon::debounce::Debouncer;
    use keyberon::key_code;
    use keyberon::layout::{CustomEvent, Event, Layout};
    use keyberon::matrix::Matrix;

    // use ssd1306::mode::BufferedGraphicsMode;
    // use ssd1306::{prelude::*, Ssd1306};

    // LEDの制御、不要
    // use smart_leds::{brightness, SmartLedsWrite, RGB8};

    use usb_device::class::UsbClass;
    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::device::UsbDeviceState;

    // LEDを扱うためのpioのラッパー、多分不要
    // use ws2812_pio::Ws2812Direct as Ws2812Pio;

    const SCAN_TIME_US: u32 = 1000;
    // const EXTERNAL_XTAL_FREQ_HZ: u32 = 12_000_000u32;
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

    // pub struct Leds {
    //     caps_lock: Ws2812Pio<PIO0, SM0, Gpio17>,
    // }

    // impl keyberon::keyboard::Leds for Leds {
    //     fn caps_lock(&mut self, status: bool) {
    //         if status {
    //             let mut onboard_data: [RGB8; 1] = [RGB8::default(); 1];
    //             onboard_data[0] = RGB8 {
    //                 r: 0x40,
    //                 g: 0x00,
    //                 b: 0x00,
    //             };
    //             self.caps_lock
    //                 .write(brightness(once(onboard_data[0]), 32))
    //                 .unwrap();
    //         } else {
    //             let mut onboard_data: [RGB8; 1] = [RGB8::default(); 1];
    //             onboard_data[0] = RGB8 {
    //                 r: 0x00,
    //                 g: 0x40,
    //                 b: 0x00,
    //             };
    //             self.caps_lock
    //                 .write(brightness(once(onboard_data[0]), 32))
    //                 .unwrap();
    //         }
    //     }
    // }

    // type Sda = rp2040_hal::gpio::Pin<
    //     rp2040_hal::gpio::bank0::Gpio12,
    //     rp2040_hal::gpio::Function<rp2040_hal::gpio::I2C>,
    // >;

    // type Scl = rp2040_hal::gpio::Pin<
    //     rp2040_hal::gpio::bank0::Gpio13,
    //     rp2040_hal::gpio::Function<rp2040_hal::gpio::I2C>,
    // >;

    // type DisplayI2C = I2CInterface<rp2040_hal::I2C<I2C0, (Sda, Scl)>>;

    #[shared]
    struct Shared {
        #[lock_free]
        underglow: Ws2812Pio<PIO0, SM1, Gpio7>,
        underglow_state: bool,
        encoder: Encoder,
        #[lock_free]
        // display: Ssd1306<DisplayI2C, DisplaySize128x32, BufferedGraphicsMode<DisplaySize128x32>>,
        // display_state: bool,
        usb_dev: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_class: keyberon::Class<'static, UsbBus, Leds>,
        timer: Timer,
        alarm: Alarm0,
        #[lock_free]
        matrix: DemuxMatrix<DynPin, DynPin, 16, 5>,
        layout: Layout<16, 5, 1, kb_layout::CustomActions>,
        #[lock_free]
        debouncer: Debouncer<[[bool; 16]; 5]>,
        #[lock_free]
        watchdog: Watchdog,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        watchdog.pause_on_debug(false);

        let clocks = init_clocks_and_plls(
            EXTERNAL_XTAL_FREQ_HZ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);

        let pins = bsp::Pins::new(c.IO_BANK0, c.PADS_BANK0, sio.gpio_bank0, &mut c.RESETS);

        let mut timer = Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        alarm.enable_interrupt();

        // let (mut pio, sm0, sm1, _, _) = c.device.PIO0.split(&mut resets);

        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        unsafe {
            USB_BUS = Some(usb_bus);
        }
        let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });

        watchdog.start(10_000.microseconds());

        let matrix: Matrix<DynPin, DynPin, 14, 5> = Matrix::new(
            [
                pins.gpio6.into_push_pull_output().into(),
                pins.gpio7.into_push_pull_output().into(),
                pins.gpio8.into_push_pull_output().into(),
                pins.gpio9.into_push_pull_output().into(),
                pins.gpio10.into_push_pull_output().into(),
                pins.gpio11.into_push_pull_output().into(),
                pins.gpio12.into_push_pull_output().into(),
                pins.gpio18.into_push_pull_output().into(),
                pins.gpio19.into_push_pull_output().into(),
                pins.gpio20.into_push_pull_output().into(),
                pins.gpio21.into_push_pull_output().into(),
                pins.gpio22.into_push_pull_output().into(),
                pins.gpio23.into_push_pull_output().into(),
                pins.gpio24.into_push_pull_output().into(),
            ],
            [
                pins.gpio13.into_pull_up_input().into(),
                pins.gpio14.into_pull_up_input().into(),
                pins.gpio15.into_pull_up_input().into(),
                pins.gpio17.into_pull_up_input().into(),
                pins.gpio16.into_pull_up_input().into(),
            ],
        );

        (
            Shared {
                usb_dev,
                usb_class,
                timer,
                alarm,
                matrix: matrix.unwrap(),
                // デバウンサーの定義、ノイズ除去について
                debouncer: Debouncer::new([[false; 16]; 5], [[false; 16]; 5], 10),
                layout: Layout::new(&kb_layout::LAYERS),
                watchdog,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let usb = c.shared.usb_dev;
        let kb = c.shared.usb_class;
        (usb, kb).lock(|usb, kb| {
            if usb.poll(&mut [kb]) {
                kb.poll();
            }
        });
    }

    #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class, layout])]
    fn handle_event(mut c: handle_event::Context, event: Option<Event>) {
        let mut layout = c.shared.layout;
        // eventはないような気がする
        match event {
            None => {
                if let CustomEvent::Press(event) = layout.lock(|l| l.tick()) {
                    match event {
                        kb_layout::CustomActions::Underglow => {
                            handle_underglow::spawn().unwrap();
                        }
                        kb_layout::CustomActions::Bootloader => {
                            rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
                        }
                        kb_layout::CustomActions::Display => {
                            handle_display::spawn().unwrap();
                        }
                    };
                }
            }
            Some(e) => {
                layout.lock(|l| l.event(e));
                return;
            }
        }

        let report: key_code::KbHidReport = layout.lock(|l| l.keycodes().collect());
        if !c
            .shared
            .usb_class
            .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
        {
            return;
        }
        if c.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
            return;
        }
        while let Ok(0) = c.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
    }

    #[task(binds = TIMER_IRQ_0, priority = 1, shared = [encoder, matrix, debouncer, timer, alarm, watchdog, usb_dev, usb_class])]
    fn scan_timer_irq(mut c: scan_timer_irq::Context) {
        let mut alarm = c.shared.alarm;

        alarm.lock(|a| {
            a.clear_interrupt();
            let _ = a.schedule(SCAN_TIME_US.microseconds());
        });

        c.shared.watchdog.feed();

        for event in c.shared.debouncer.events(c.shared.matrix.get().unwrap()) {
            handle_event::spawn(Some(event)).unwrap();
        }

        c.shared.encoder.lock(|e| {
            if let Some(events) = e.read_events() {
                for event in events {
                    handle_event::spawn(Some(event)).unwrap();
                }
            }
        });

        handle_event::spawn(None).unwrap();
    }
}
