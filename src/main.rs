#![no_std]
#![no_main]
#![allow(unused, non_camel_case_types)]

use arrayvec::ArrayString;
use arrayvec::ArrayVec;
use core::fmt::Write;
use core::mem::MaybeUninit;
use core::ptr::read;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Extensions;
use embedded_time::rate::*;
use hal::{adc::Adc, clocks::*, pac::interrupt, timer::Alarm, watchdog::Watchdog, Sio};
use heapless::{
    pool,
    pool::singleton::{Box, Pool},
    spsc::{Consumer, Producer, Queue},
};
use panic_halt as _;
use pimoroni_pico_explorer::entry;
use pimoroni_pico_explorer::{hal, pac, Button, PicoExplorer, XOSC_CRYSTAL_FREQ};
use systick_monotonic::Systick;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

use crate::pac::{Peripherals, RESETS, USBCTRL_DPRAM, USBCTRL_REGS};
use x328_proto::node::NodeState;

// See 4.9.5 from RP2040 datasheet
fn calc_temp(adc_value: f32, refv: f64) -> f64 {
    let vbe: f64 = f64::from(adc_value) * refv;
    27f64 - (vbe - 0.706) / 0.001721
}

const X328_NODE_ADDR: x328_proto::Address = x328_proto::addr(10);

#[rtic::app(device = hal::pac, peripherals = true, dispatchers = [RTC_IRQ])]
mod app {
    use super::*;
    use crate::hal::gpio;
    use crate::pac::CorePeripherals;
    use embedded_graphics::text::renderer::TextRenderer;
    use embedded_hal::digital::v2::StatefulOutputPin;
    use embedded_hal::prelude::_embedded_hal_blocking_serial_Write;
    use rp2040_monotonic::Rp2040Monotonic;

    type DispLine = ArrayString<25>;
    pool!(DISP_TEXT: DispLine);

    const SCAN_TIME_US: u32 = 10000;

    #[monotonic(binds = SysTick, default = true)]
    //type MyMono = rp2040_monotonic::Rp2040Monotonic;
    type MyMono = Systick<100>;

    #[shared]
    struct Shared {
        //    timer: hal::Timer,
        //  alarm: hal::timer::Alarm0,
        led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
        usb_serial: SerialPort<'static, hal::usb::UsbBus>,
    }

    #[local]
    struct Local {
        gpio_0: gpio::Pin<gpio::bank0::Gpio0, gpio::Output<gpio::PushPull>>,
        gpio_1: gpio::Pin<gpio::bank0::Gpio1, gpio::Output<gpio::PushPull>>,
        gpio_2: gpio::Pin<gpio::bank0::Gpio2, gpio::Output<gpio::PushPull>>,
        screen: pimoroni_pico_explorer::Screen,
        x328_node: x328_proto::node::Node,
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        disp_text_tx: Producer<'static, Box<DISP_TEXT>, 2>,
        disp_text_rx: Consumer<'static, Box<DISP_TEXT>, 2>,
    }

    #[init(
        local = [
            usb_bus_uninit: MaybeUninit<usb_device::bus::UsbBusAllocator<hal::usb::UsbBus>> = MaybeUninit::uninit(),
            uart_mem_pool: [u8; 128] = [0; 128],
            usb_tx_queue: Queue<Box<DISP_TEXT>, 2> = Queue::new(),
        ]
    )]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let init::LocalResources {
            usb_bus_uninit,
            uart_mem_pool,
            usb_tx_queue,
        } = ctx.local;

        DISP_TEXT::grow(uart_mem_pool);

        let mut pac: Peripherals = ctx.device;
        let cp: CorePeripherals = ctx.core;

        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
        let mut temp_sense = adc.enable_temp_sensor();

        let sio = Sio::new(pac.SIO);

        let mut delay =
            cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.get_freq().integer());

        let (mut explorer, pins) = PicoExplorer::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            pac.SPI0,
            adc,
            &mut pac.RESETS,
            &mut delay,
        );
        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        // Set up the USB driver
        usb_bus_uninit.write(UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )));
        // SAFETY: This is ok because we just wrote a valid value above.
        let usb_bus = unsafe { usb_bus_uninit.assume_init_ref() };

        // Set up the USB Communications Class Device driver
        let mut usb_serial = SerialPort::new(usb_bus);

        // Create a USB device with a fake VID and PID
        let mut usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();

        // The queue for sending USB serial data
        let (disp_text_tx, disp_text_rx) = usb_tx_queue.split();

        let x328_node = x328_proto::node::Node::new(X328_NODE_ADDR);

        x328_task::spawn();
        (
            Shared {
                // timer,
                //  alarm,
                led,
                usb_serial,
            },
            Local {
                gpio_0: pins.gpio0.into_push_pull_output(),
                gpio_1: pins.gpio1.into_push_pull_output(),
                gpio_2: pins.gpio2.into_push_pull_output(),
                screen: explorer.screen,
                x328_node,
                usb_device,
                disp_text_tx,
                disp_text_rx,
            },
            init::Monotonics(Systick::new(
                delay.free(),
                clocks.system_clock.get_freq().integer(),
            )),
        )
    }

    // Task with least priority that only runs when nothing else is running.
    #[idle(local = [gpio_0, screen, disp_text_rx])]
    fn idle(cx: idle::Context) -> ! {
        let idle::LocalResources {
            gpio_0,
            screen,
            disp_text_rx,
        } = cx.local;
        let screen: &mut pimoroni_pico_explorer::Screen = screen;

        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::GREEN)
            .background_color(Rgb565::BLACK)
            .build();

        let mut pos = Point::new(0, style.line_height() as i32 * 5);
        let start_pos = pos;

        loop {
            if Ok(true) == gpio_0.is_set_high() {
                gpio_0.set_low();
            } else {
                gpio_0.set_high();
            } // Create a fixed buffer to store screen contents
            let mut buf = ArrayString::<100>::new();

            // Write to buffer
            writeln!(&mut buf, "Hello World").unwrap();
            writeln!(&mut buf, "Temp: {}", gpio_0.is_set_high().unwrap()).unwrap();
            // Draw buffer on screen
            Text::with_alignment(
                &buf,
                Point::new(0, style.line_height() as i32),
                style,
                Alignment::Left,
            )
            .draw(screen)
            .unwrap();

            while let Some(line) = disp_text_rx.dequeue() {
                Text::new("  ", pos, style).draw(screen).unwrap();
                pos.y += style.line_height() as i32;
                if pos.y >= screen.bounding_box().size.height as i32 {
                    pos = start_pos;
                }
                pos = Text::new("> ", pos, style).draw(screen).unwrap();
                Text::with_alignment(line.as_ref(), pos, style, Alignment::Left)
                    .draw(screen)
                    .unwrap();
                pos.x = 0;
            }
        }
    }

    #[task(
        priority = 1,
        capacity = 1,
        local = [x328_node, disp_text_tx, gpio_2],
        shared = [usb_serial, led]
    )]
    fn x328_task(mut ctx: x328_task::Context) {
        let x328_task::LocalResources {
            x328_node,
            disp_text_tx,
            gpio_2,
        } = ctx.local;
        let x328_node: &mut x328_proto::node::Node = x328_node;
        let mut data_in = ArrayVec::from([0; 20]);
        let mut serial = ctx.shared.usb_serial;
        let mut led = ctx.shared.led;
        led.lock(|led| {
            if !led.is_set_high().unwrap() {
                led.set_high();
            } else {
                led.set_low();
            }
        });

        loop {
            match x328_node.state() {
                NodeState::ReceiveData(recv) => {
                    serial.lock(|serial| {
                        gpio_2.set_high();
                        let len = serial.read(data_in.as_mut()).unwrap_or(0);
                        data_in.truncate(len);
                        gpio_2.set_low();
                    });
                    if data_in.is_empty() {
                        break;
                    }
                    recv.receive_data(&data_in);
                }
                NodeState::SendData(send) => serial.lock(|serial| {
                    serial.write(send.get_data());
                }),
                NodeState::ReadParameter(read) => {
                    if let Some(mut line) =
                        DISP_TEXT::alloc().map(|uninit| uninit.init(DispLine::new()))
                    {
                        write!(&mut line, "r {} {}", *read.address(), *read.parameter());
                        disp_text_tx.enqueue(line);
                    }
                    match *read.parameter() {
                        1 => read.send_reply_ok(4u16.into()),
                        _ => read.send_invalid_parameter(),
                    }
                }
                NodeState::WriteParameter(write) => match *write.parameter() {
                    _ => write.write_ok(),
                },
            }
        }
    }

    #[task(
        binds = USBCTRL_IRQ,
        priority=2,
        local = [usb_device, gpio_1],
        shared = [usb_serial],
    )]
    fn usb_irq(mut ctx: usb_irq::Context) {
        use core::sync::atomic::{AtomicBool, Ordering};

        let usb_irq::LocalResources { usb_device, gpio_1 } = ctx.local;

        let mut serial = ctx.shared.usb_serial;
        // Poll the USB driver with all of our supported USB Classes
        let mut ready = false;
        serial.lock(|serial: &mut SerialPort<_>| {
            gpio_1.set_high();
            ready = usb_device.poll(&mut [serial]);
            if ready {
                let mut buf = [0u8; 0];
                serial.read(&mut buf);
                x328_task::spawn();
            }
            gpio_1.set_low();
        });
    }
}
