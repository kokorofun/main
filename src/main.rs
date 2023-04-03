#![no_std]
#![no_main]

// extern crate alloc;

// use cortex_m_rt::entry;
// use embedded_alloc::Heap;

// #[global_allocator]
// static HEAP: Heap = Heap::empty();

const REVERSE_LOOKUP: [u8; 256] = [
    0, 128, 64, 192, 32, 160, 96, 224, 16, 144, 80, 208, 48, 176, 112, 240, 8, 136, 72, 200, 40,
    168, 104, 232, 24, 152, 88, 216, 56, 184, 120, 248, 4, 132, 68, 196, 36, 164, 100, 228, 20,
    148, 84, 212, 52, 180, 116, 244, 12, 140, 76, 204, 44, 172, 108, 236, 28, 156, 92, 220, 60,
    188, 124, 252, 2, 130, 66, 194, 34, 162, 98, 226, 18, 146, 82, 210, 50, 178, 114, 242, 10, 138,
    74, 202, 42, 170, 106, 234, 26, 154, 90, 218, 58, 186, 122, 250, 6, 134, 70, 198, 38, 166, 102,
    230, 22, 150, 86, 214, 54, 182, 118, 246, 14, 142, 78, 206, 46, 174, 110, 238, 30, 158, 94,
    222, 62, 190, 126, 254, 1, 129, 65, 193, 33, 161, 97, 225, 17, 145, 81, 209, 49, 177, 113, 241,
    9, 137, 73, 201, 41, 169, 105, 233, 25, 153, 89, 217, 57, 185, 121, 249, 5, 133, 69, 197, 37,
    165, 101, 229, 21, 149, 85, 213, 53, 181, 117, 245, 13, 141, 77, 205, 45, 173, 109, 237, 29,
    157, 93, 221, 61, 189, 125, 253, 3, 131, 67, 195, 35, 163, 99, 227, 19, 147, 83, 211, 51, 179,
    115, 243, 11, 139, 75, 203, 43, 171, 107, 235, 27, 155, 91, 219, 59, 187, 123, 251, 7, 135, 71,
    199, 39, 167, 103, 231, 23, 151, 87, 215, 55, 183, 119, 247, 15, 143, 79, 207, 47, 175, 111,
    239, 31, 159, 95, 223, 63, 191, 127, 255,
];

mod assets;
mod coso;
mod display;
mod utils;

use coso::*;

use core::cell::RefCell;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;

use cortex_m::interrupt::Mutex;
use cortex_m::prelude::_embedded_hal_blocking_spi_Write;
use cortex_m::prelude::_embedded_hal_spi_FullDuplex;
use embedded_hal::digital::v2::OutputPin;
use fugit::Duration;
use fugit::ExtU32;
use fugit::MicrosDurationU32;
// The macro for our start-up function
use rp_pico::entry;

// info!() and error!() macros for printing information to the debug output
use defmt::*;
use defmt_rtt as _;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use rp_pico::hal::gpio::bank0::Gpio19;
use rp_pico::hal::gpio::bank0::Gpio25;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::Pin;
use rp_pico::hal::gpio::PushPull;
// Pull in any important traits
use rp_pico::hal::prelude::*;

// Embed the `Hz` function/trait:
use fugit::RateExtU32;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac::{self, interrupt};

// Import the SPI abstraction:
use rp_pico::hal::spi;

// Import the GPIO abstraction:
use rp_pico::hal::gpio;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use rp_pico::hal::timer::Alarm;
use rp_pico::hal::timer::Alarm0;

/// A dummy timesource, which is mostly important for creating files.
#[derive(Default)]
pub struct DummyTimesource();

// const SHARPMEM_BIT_WRITECMD: u8 = 0x01; // 0x80 in LSB format
// const SHARPMEM_BIT_VCOM: u8 = 0x02; // 0x40 in LSB format
// const SHARPMEM_BIT_CLEAR: u8 = 0x04; // 0x20 in LSB format

const SHARPMEM_BIT_WRITECMD: u8 = 0x80; // 0x80 in LSB format
const SHARPMEM_BIT_VCOM: u8 = 0x40; // 0x40 in LSB format
const SHARPMEM_BIT_CLEAR: u8 = 0x20; // 0x20 in LSB format

const VRES: usize = 240;
const HRES: usize = 400;

type VCOMPinType = Pin<Gpio19, Output<PushPull>>;
type LEDPinType = Pin<Gpio25, Output<PushPull>>;

const VCOM_PERIOD_MS: u32 = 67;

static G_ALARM0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static G_VCOM_PIN: Mutex<RefCell<Option<VCOMPinType>>> = Mutex::new(RefCell::new(None));
static G_LED_PIN: Mutex<RefCell<Option<LEDPinType>>> = Mutex::new(RefCell::new(None));
static G_IS_VCOM_HIGH: AtomicBool = AtomicBool::new(false);
static G_TIMER: Mutex<RefCell<Option<hal::Timer>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    info!("Program start");

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut vcom_pin = pins.gpio19.into_push_pull_output();
    let mut led_pin = pins.led.into_push_pull_output();

    // Setup a delay for the LED blink signals:
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<gpio::FunctionSpi>();
    let mut spi_cs = pins.gpio5.into_push_pull_output();
    let mut on = pins.gpio20.into_push_pull_output();
    // let mut extcomin = pins.gpio19.into_push_pull_output();

    delay.delay_us(500);
    on.set_high().unwrap();

    // Create an SPI driver instance for the SPI0 device
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        2_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let alarm0 = timer.alarm_0().unwrap();

    cortex_m::interrupt::free(|cs| {
        // use rp_pico::hal::timer::Alarm;

        G_ALARM0.borrow(cs).replace(Some(alarm0));
        G_VCOM_PIN.borrow(cs).replace(Some(vcom_pin));
        G_LED_PIN.borrow(cs).replace(Some(led_pin));
        G_TIMER.borrow(cs).replace(Some(timer));

        let ref mut g_alarm0 = G_ALARM0.borrow(cs).borrow_mut();
        if let Some(alarm0) = g_alarm0.as_mut() {
            alarm0.schedule(VCOM_PERIOD_MS.millis()).unwrap();
            alarm0.enable_interrupt();
        }

        let ref mut g_vcom_pin = G_VCOM_PIN.borrow(cs).borrow_mut();
        let ref mut g_led_pin = G_LED_PIN.borrow(cs).borrow_mut();
        if let Some(led_pin) = g_vcom_pin.as_mut() {
            // set led on for start
            led_pin.set_high().unwrap();
            if let Some(led_pin) = g_led_pin.as_mut() {
                led_pin.set_high().unwrap();
            }
            G_IS_VCOM_HIGH.store(true, Ordering::Release);
        }
    });

    #[allow(unsafe_code)]
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    // // Clear the screen

    // spi_cs.set_high().unwrap();
    // delay.delay_us(20);

    // spi.write(&[0b00100000, 0x00]).unwrap();
    // // spi.write(&[0b00000100, 0x00]).unwrap();

    // delay.delay_us(20);
    // spi_cs.set_low().11101110unwrap();

    // let mut vcom = 0b01000000;
    let mut vcom = 0b00000000;

    // {
    //     spi_cs.set_high().unwrap();
    //     delay.delay_us(20);

    //     // spi.write(&[0b00000100 | vcom, 0x00]).unwrap();
    //     spi.write(&[0b00100000 | vcom, 0x00]).unwrap();

    //     delay.delay_us(20);
    //     spi_cs.set_low().unwrap();
    // }

    // vcom = !vcom & 0b01000000;

    let mut display = display::Display::new(spi, spi_cs, delay);

    display.draw_texture(
        display::Position { x: 0, y: 0 },
        &assets::textures::Background
    );
    display.draw_texture(
        display::Position { x: 0, y: 0 },
        &assets::textures::uru::idle_front
    );

    // display.draw_texture(
    //     display::Position { x: 0, y: 0 },
    //     &assets::texture::furniture::lava_lamp.base,
    //     assets::texture::furniture::lava_lamp.width as usize,
    //     assets::texture::furniture::lava_lamp.height as usize,
    // );
    // display.update();

    let mut col: usize = 0;
    let max_x: i32 = 400 - 1 - assets::textures::uru::idle_front_smol.width as i32;
    let max_y: i32 = 240 - 1 - assets::textures::uru::idle_front_smol.height as i32;

    let v_x = 2;
    let v_y = 2;

    let mut x: i32 = 0;
    let mut y: i32 = 0;
    let mut x_dir: i32 = 1;
    let mut y_dir = 1;

    loop {
        display.draw_texture(
            display::Position { x: 0, y: 0 },
            &assets::textures::Background
        );
        display.draw_texture(
            display::Position { x: x as usize, y: y as usize },
            &assets::textures::uru::idle_front_smol
        );
        display.update();


        x += v_x * x_dir;
        y += v_y * y_dir;

        if x > max_x  || x < 0 {
            x_dir = -1 * x_dir;
            if x < 0 {
                x = 0;
            } else {
                x = max_x;
            }
        }
        if y > max_y || y < 0 {
            y_dir = -1 * y_dir;
            if y < 0 {
                y = 0;
            } else {
                y = max_y;
            }
        }

        // delay.delay_ms(1000);
    }
}

// 8   1000 0000 <- 0000 0001 1
// 4   0100 0000 <- 0000 0010 2
// 12  1100 0000 <- 0000 0011 3
// 2   0010 0000 <- 0000 0100 4
// 10  1010 0000 <- 0000 0101 5
// 6   0110 0000 <- 0000 0110 6

#[interrupt]
fn TIMER_IRQ_0() {
    static mut VCOM: Option<VCOMPinType> = None;
    static mut LED: Option<LEDPinType> = None;

    if VCOM.is_none() {
        cortex_m::interrupt::free(|cs| {
            *VCOM = G_VCOM_PIN.borrow(&cs).take();
        });
    }

    if LED.is_none() {
        cortex_m::interrupt::free(|cs| {
            *LED = G_LED_PIN.borrow(&cs).take();
        });
    }

    // let mut set_high = false;

    // switch led
    if let Some(vcom) = VCOM {
        if let Some(led) = LED {
            let is_high = G_IS_VCOM_HIGH.load(Ordering::Acquire);
            // led.set_low().unwrap();

            if is_high {
                vcom.set_low().unwrap();
                led.set_low().unwrap();
            } else {
                // set_high = true;
                vcom.set_high().unwrap();
                led.set_high().unwrap();
            }
            G_IS_VCOM_HIGH.store(!is_high, Ordering::Release);
        }
    }

    cortex_m::interrupt::free(|cs| {
        let ref mut g_alarm0 = G_ALARM0.borrow(cs).borrow_mut();
        if let Some(alarm0) = g_alarm0.as_mut() {
            // let ref mut g_timer = G_TIMER.borrow(cs).borrow_mut();
            alarm0.clear_interrupt();

            // if false {
            //     alarm0.schedule(500u32.micros()).unwrap();
            //     alarm0.enable_interrupt();
            // } else {
            alarm0.schedule(VCOM_PERIOD_MS.millis()).unwrap();
            // alarm0.schedule(100u32.micros()).unwrap();

            alarm0.enable_interrupt();
            // }

            // if let Some(timer) = g_timer.as_mut() {
            // }
        }
    });
}
