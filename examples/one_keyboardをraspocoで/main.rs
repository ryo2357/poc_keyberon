//! One key keyboard example using keyberon crate.
//! Based on https://github.com/camrbuss/pinci implementation.
// [rustberry-pi-pico/src/examples/keyberon-one-key-keyboard/example.rs at main · radlinskii/rustberry-pi-pico · GitHub](https://github.com/radlinskii/rustberry-pi-pico/blob/main/src/examples/keyberon-one-key-keyboard/example.rs)
// 入力されない
// raspicoにブートしても動かない

// Cargo.toml
// # one keyboard
// # blink on-board led
// embedded-hal = "0.2.7"
// rp-pico = "0.6.0"
// cortex-m = "0.7.6"
// panic-halt = "0.2.0"
// cortex-m-rt = "0.7.2"

// # ☝️ + usb-serial-communicator
// usb-device = "0.2.9"
// usbd-serial = "0.1.1"
// heapless = "0.7.16"

// # rtic
// cortex-m-rtic = "1.1.3"
// fugit = "0.3.6"

// # keyberon
// keyberon = { git = "https://github.com/TeXitoi/keyberon" }
// rp2040-hal = "0.7.0"
// rp2040-monotonic = "1.2.0"

// # flip-link: the native linker failed to link the program normally; please check your project configuration and linker scripts
// # 上記エラーでコンパイルできない
// defmt = "0.3.2"
// defmt-rtt = "0.4.0"
// panic-probe = { version = "0.3.0", features = ["print-defmt"] }

#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
    use cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable;
    use embedded_hal::digital::v2::OutputPin;
    use keyberon::debounce::Debouncer;
    use keyberon::key_code;
    use keyberon::layout::{self, Layout};
    use keyberon::matrix::Matrix;
    use rp_pico::{
        hal::{
            self, clocks::init_clocks_and_plls, gpio::DynPin, sio::Sio, timer::Alarm, usb::UsbBus,
            watchdog::Watchdog,
        },
        XOSC_CRYSTAL_FREQ,
    };
    use usb_device::class_prelude::*;
    use usb_device::device::UsbDeviceState;

    use fugit::MicrosDurationU32;

    const SCAN_TIME_US: MicrosDurationU32 = MicrosDurationU32::micros(1000);

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<rp2040_hal::usb::UsbBus>> = None;

    pub static LAYERS: keyberon::layout::Layers<1, 1, 1> = keyberon::layout::layout! {
        {
            [
                A
            ]
        }
    };

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, rp2040_hal::usb::UsbBus>,
        usb_class: keyberon::hid::HidClass<
            'static,
            rp2040_hal::usb::UsbBus,
            keyberon::keyboard::Keyboard<()>,
        >,
        layout: Layout<1, 1, 1>,
    }

    #[local]
    struct Local {
        watchdog: hal::watchdog::Watchdog,
        matrix: Matrix<DynPin, DynPin, 1, 1>,
        debouncer: Debouncer<[[bool; 1]; 1]>,
        alarm: hal::timer::Alarm0,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
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
        let pins = hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut led = pins.gpio11.into_push_pull_output();
        led.set_high().unwrap();

        // delay for power on
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }

        let matrix: Matrix<DynPin, DynPin, 1, 1> = Matrix::new(
            [pins.gpio28.into_pull_up_input().into()],
            [pins.gpio27.into_push_pull_output().into()],
        )
        .unwrap();

        let layout = Layout::new(&LAYERS);
        let debouncer = Debouncer::new([[false; 1]; 1], [[false; 1]; 1], 20);

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US);
        alarm.enable_interrupt();

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

        // Start watchdog and feed it with the lowest priority task at 1000hz
        watchdog.start(MicrosDurationU32::micros(10000));

        (
            Shared {
                usb_dev,
                usb_class,
                layout,
            },
            Local {
                alarm,
                watchdog,
                matrix,
                debouncer,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let mut usb_d = c.shared.usb_dev;
        let mut usb_c = c.shared.usb_class;
        usb_d.lock(|d| {
            usb_c.lock(|c| {
                if d.poll(&mut [c]) {
                    c.poll();
                }
            })
        });
    }

    #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class, layout])]
    fn handle_event(mut c: handle_event::Context, event: Option<layout::Event>) {
        match event {
            None => (),
            Some(e) => {
                c.shared.layout.lock(|l| l.event(e));
                return;
            }
        };

        let report: key_code::KbHidReport = c.shared.layout.lock(|l| l.keycodes().collect());
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

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        local = [matrix, debouncer, watchdog, alarm],
    )]
    fn scan_timer_irq(c: scan_timer_irq::Context) {
        let alarm = c.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_TIME_US);

        c.local.watchdog.feed();
        let keys_pressed = c.local.matrix.get().unwrap();
        let deb_events = c.local.debouncer.events(keys_pressed);

        for event in deb_events {
            handle_event::spawn(Some(event)).unwrap();
        }
        handle_event::spawn(None).unwrap();
    }
}
