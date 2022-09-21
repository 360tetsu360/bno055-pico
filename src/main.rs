//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{gpio, uart, I2C},
};
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use core::fmt::Write;

use fugit::RateExtU32;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut uart_on = pins.gpio22.into_push_pull_output();

    let mut led_pin = pins.led.into_push_pull_output();
    for _ in 0..3 {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }

    // Initialize uart for debug
    let uart_pins = (
        pins.gpio0.into_mode::<gpio::FunctionUart>(),
        pins.gpio1.into_mode::<gpio::FunctionUart>(),
    );

    let mut uart = uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            uart::common_configs::_115200_8_N_1,
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart_on.set_high().unwrap();
    uart.write_str("\n").unwrap();
    uart.write_str("UART start\n").unwrap();

    // Initialize bno055
    let sda_pin = pins.gpio16.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio17.into_mode::<gpio::FunctionI2C>();
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400_u32.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let mut imu = bno055::Bno055::new(i2c).with_alternative_address();
    imu.init(&mut delay).unwrap_or_else(|_| {
        uart.write_full_blocking(b"An error occurred while building the IMU\n")
    });
    imu.set_external_crystal(true, &mut delay).unwrap();
    imu.set_mode(bno055::BNO055OperationMode::IMU, &mut delay)
        .unwrap_or_else(|_| {
            uart.write_full_blocking(b"An error occurred while building the IMU\n")
        });

    uart.write_str("BNO055 initialized!\n").unwrap();

    loop {
        if let Ok(is_calibrated) = imu.is_fully_calibrated() {
            if is_calibrated {
                uart.write_str("calibrated.\n").unwrap();
                if let Ok(calib_profile) = imu.calibration_profile(&mut delay) {
                    let _ = imu.set_calibration_profile(calib_profile, &mut delay);
                }
            }
        }
        delay.delay_ms(1000);
    }
}

// End of file
