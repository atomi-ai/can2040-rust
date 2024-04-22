//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

extern crate alloc;

use alloc_cortex_m::CortexMHeap;
use defmt::*;
use defmt_rtt as _;
use embedded_can::nb::Can;
use embedded_can::{ExtendedId, Frame, StandardId};
use embedded_hal::digital::v2::ToggleableOutputPin;
use panic_probe as _;
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::gpio::Pins;
use rp2040_hal::{entry, pac, Sio, Watchdog};
use rp_pico::XOSC_CRYSTAL_FREQ;

use can2040::{Can2040, CanFrame};

const CONFIG_CANBUS_FREQUENCY: u32 = 10_000;
const CONFIG_RP2040_CANBUS_GPIO_RX: u32 = 26;
const CONFIG_RP2040_CANBUS_GPIO_TX: u32 = 27;

#[global_allocator]
pub static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

pub fn init_allocator() {
    // Please set the correct heap size.
    const HEAP_SIZE: usize = 0x8000;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP.len()) }
}

#[entry]
fn main() -> ! {
    init_allocator();
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let _clocks = init_clocks_and_plls(
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

    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let mut led_pin = pins.gpio25.into_push_pull_output();

    let mut can_bus = can2040::initialize_cbus(
        &mut core,
        CONFIG_CANBUS_FREQUENCY,
        CONFIG_RP2040_CANBUS_GPIO_RX,
        CONFIG_RP2040_CANBUS_GPIO_TX,
    );

    let mut count = 0u64;
    let mut packet_num = 0u64;
    loop {
        count += 1;
        if count % 1_000_000 == 13 {
            packet_num += 1;
            info!("before generate frame");
            let f = CanFrame::new(
                ExtendedId::new(0x123).expect("error in create standard id"),
                &[1, 2, 3, (packet_num & 0xff) as u8],
            )
            .expect("error in create_frame");

            info!("To transmit frame: {}", Debug2Format(&f));

            // let f = create_frame(5, packet_num);
            match <Can2040 as embedded_can::blocking::Can>::transmit(&mut can_bus, &f) {
                Ok(_) => {}
                Err(err) => {
                    error!("Transmit error: {}", err);
                }
            }
            info!("Transmitted package: {:?}", f);
            led_pin.toggle().expect("TODO: led toggle");
        }
        match can_bus.receive() {
            Ok(f) => {
                info!("Received packet: {:?}, is_extended: {}", f, f.is_extended());
            }
            Err(nb::Error::Other(err)) => {
                error!("Errors in reading CAN frame, {:?}", err);
            }
            _ => {} // ignore
        }
    }
}

pub fn create_frame(cob_id: u16, data: u64) -> CanFrame {
    CanFrame::new(
        StandardId::new(cob_id).expect("error in create standard id"),
        &[1, 2, 3, (data & 0xff) as u8],
    )
    .expect("error in create_frame")
}
