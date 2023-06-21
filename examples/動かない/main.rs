//! One key keyboard example using keyberon crate.
//! Based on https://github.com/camrbuss/pinci implementation.

#![no_std]
#![no_main]

use panic_halt as _;
mod key_setting;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use cortex_m::prelude::{
        _embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogEnable,
    };
    use rp2040_hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{bank0::*, dynpin::DynPin},
        pac::{CorePeripherals, I2C0, PIO0, RESETS, SPI0},
        pio::{PIOExt, SM0, SM1},
        sio::Sio,
        timer::{Alarm, Alarm2, Alarm3, Timer},
        usb::UsbBus,
        watchdog::Watchdog,
    };
    use rp_pico::{hal, XOSC_CRYSTAL_FREQ};

    use embedded_hal::{prelude::*, watchdog};
    use keyberon::debounce::Debouncer;
    use keyberon::key_code;
    use keyberon::layout::{self, Layout};
    use keyberon::matrix::Matrix;
    // use rp_pico::{
    //     hal::{
    //         self, clocks::init_clocks_and_plls, gpio::DynPin, sio::Sio, timer::Alarm, usb::UsbBus,
    //         watchdog::Watchdog,
    //     },
    //     XOSC_CRYSTAL_FREQ,
    // };
    use usb_device::class_prelude::*;
    use usb_device::device::UsbDeviceState;

    use rp2040_monotonic::fugit::MicrosDurationU32;

    const SCAN_TIME_US: MicrosDurationU32 = MicrosDurationU32::micros(1000);
    // MicrosDurationU32::micros(1000);
    // const SCAN_TIME_US: u32 = 2000;

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<rp2040_hal::usb::UsbBus>> = None;

    use crate::key_setting;

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, rp2040_hal::usb::UsbBus>,
        usb_class: keyberon::hid::HidClass<
            'static,
            rp2040_hal::usb::UsbBus,
            keyberon::keyboard::Keyboard<()>,
        >,
        layout: Layout<2, 2, 3>,
    }
    #[local]
    struct Local {
        watchdog: hal::watchdog::Watchdog,
        matrix: Matrix<DynPin, DynPin, 2, 2>,
        debouncer: Debouncer<[[bool; 2]; 2]>,
        // debouncer: Debouncer::new([[false; 13]; 4], [[false; 13]; 4], 5),
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
        // 動かない
        // let mut led = pins.gpio12.into_push_pull_output();
        // led.set_high().unwrap();

        // delay for power on
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }

        let matrix: Matrix<DynPin, DynPin, 2, 2> = Matrix::new(
            [
                pins.gpio19.into_pull_up_input().into(),
                pins.gpio10.into_pull_up_input().into(),
            ],
            [
                pins.gpio20.into_push_pull_output().into(),
                pins.gpio11.into_push_pull_output().into(),
            ],
        )
        .unwrap();

        let layout = Layout::new(&key_setting::LAYERS);
        let debouncer = Debouncer::new([[false; 2]; 2], [[false; 2]; 2], 5);

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

    #[task(binds = USBCTRL_IRQ, priority = 4, shared = [usb_dev, usb_class])]
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
    fn handle_event(mut c: handle_event::Context, event: Option<layout::Event>) {
        let mut layout = c.shared.layout;
        match event {
            None => (),
            Some(e) => {
                layout.lock(|l| l.event(e));
                return;
            }
        };

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

    #[task(
        binds = TIMER_IRQ_3,
        priority = 2,
        local = [matrix, debouncer, watchdog, alarm],
    )]
    fn scan_timer_irq(c: scan_timer_irq::Context) {
        c.local.watchdog.feed();

        for event in c.local.debouncer.events(c.local.matrix.get().unwrap()) {
            handle_event::spawn(Some(event)).unwrap();
        }

        handle_event::spawn(None).unwrap();

        let alarm = c.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_TIME_US);
    }
}
