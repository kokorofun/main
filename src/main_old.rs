#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;

use cortex_m::interrupt::Mutex;
use cortex_m::prelude::_embedded_hal_blocking_spi_Write;
use cortex_m::prelude::_embedded_hal_spi_FullDuplex;
use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
// The macro for our start-up function
use rp_pico::entry;

// info!() and error!() macros for printing information to the debug output
use defmt::*;
use defmt_rtt as _;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

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

type LEDPinType = Pin<Gpio25, Output<PushPull>>;

static G_ALARM0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static G_LED_PIN: Mutex<RefCell<Option<LEDPinType>>> = Mutex::new(RefCell::new(None));
static G_IS_LED_HIGH: AtomicBool = AtomicBool::new(false);
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
    let mut led_pin = pins.led.into_push_pull_output();

    // Setup a delay for the LED blink signals:
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<gpio::FunctionSpi>();
    let mut spi_cs = pins.gpio5.into_push_pull_output();
    let mut on = pins.gpio20.into_push_pull_output();
    on.set_high().unwrap();
    // let mut extcomin = pins.gpio19.into_push_pull_output();

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
        G_LED_PIN.borrow(cs).replace(Some(led_pin));
        G_TIMER.borrow(cs).replace(Some(timer));

        let ref mut g_alarm0 = G_ALARM0.borrow(cs).borrow_mut();
        if let Some(alarm0) = g_alarm0.as_mut() {
            alarm0.schedule(500u32.millis()).unwrap();
            alarm0.enable_interrupt();
        }

        // let ref mut g_led_pin = G_LED_PIN.borrow(cs).borrow_mut();
        // if let Some(led_pin) = g_led_pin.as_mut() {
        //     // set led on for start
        //     led_pin.set_high().unwrap();
        //     G_IS_LED_HIGH.store(true, Ordering::Release);
        // }
    });

    #[allow(unsafe_code)]
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    loop {
        // cortex_m::asm::wfi();
        delay.delay_ms(200);
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    static mut LED: Option<LEDPinType> = None;

    if LED.is_none() {
        cortex_m::interrupt::free(|cs| {
            *LED = G_LED_PIN.borrow(&cs).take();
        });
    }

    // switch led
    if let Some(led) = LED {
        let is_high = G_IS_LED_HIGH.load(Ordering::Acquire);
        // led.set_low().unwrap();

        if is_high {
            led.set_low().unwrap();
        } else {
            led.set_high().unwrap();
        }
        G_IS_LED_HIGH.store(!is_high, Ordering::Release);
    }

    cortex_m::interrupt::free(|cs| {
        let ref mut g_alarm0 = G_ALARM0.borrow(cs).borrow_mut();
        if let Some(alarm0) = g_alarm0.as_mut() {
            // let ref mut g_timer = G_TIMER.borrow(cs).borrow_mut();
            alarm0.clear_interrupt();

            alarm0.schedule(500u32.millis()).unwrap();
            alarm0.enable_interrupt();
            // if let Some(timer) = g_timer.as_mut() {
            // }
        }
    });
}
