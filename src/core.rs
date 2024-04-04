extern crate alloc;

use alloc::fmt;
use alloc::vec::Vec;
use core::cell::RefCell;

use rp2040_hal::pac;
use cortex_m::asm::wfi;
use cortex_m::interrupt::Mutex;
use defmt::{debug, Format};
use embedded_can::{ErrorKind, Id, StandardId};
use rp2040_hal::pac::{interrupt, Interrupt};

use crate::core::can2040_lib::{
    can2040, can2040_bitunstuffer, can2040_callback_config, can2040_check_transmit, can2040_msg,
    can2040_msg__bindgen_ty_1, can2040_pio_irq_handler, can2040_setup, can2040_start,
    can2040_transmit, CAN2040_NOTIFY_RX,
};

#[allow(warnings)]
mod can2040_lib {
    include!(concat!(env!("OUT_DIR"), "/can2040_lib.rs"));
}

impl can2040_bitunstuffer {
    pub fn new() -> Self {
        Self { stuffed_bits: 0, count_stuff: 0, unstuffed_bits: 0, count_unstuff: 0 }
    }
}

impl can2040_msg__bindgen_ty_1 {
    pub fn new() -> Self {
        Self {
            data: [0; 8], // 这里使用data字段初始化
        }
    }
}

impl can2040_msg {
    pub fn new() -> Self {
        Self { id: 0, dlc: 0, __bindgen_anon_1: can2040_msg__bindgen_ty_1::new() }
    }
}

impl can2040_transmit {
    pub fn new() -> Self {
        Self { msg: can2040_msg::new(), crc: 0, stuffed_words: 0, stuffed_data: [0; 5] }
    }
}

impl can2040 {
    pub fn new() -> Self {
        Self {
            pio_num: 0,
            pio_hw: core::ptr::null_mut(),
            gpio_rx: 0,
            gpio_tx: 0,
            rx_cb: None,
            unstuf: can2040_bitunstuffer::new(),
            raw_bit_count: 0,
            parse_state: 0,
            parse_crc: 0,
            parse_crc_bits: 0,
            parse_crc_pos: 0,
            parse_msg: can2040_msg::new(),
            report_state: 0,
            tx_state: 0,
            tx_pull_pos: 0,
            tx_push_pos: 0,
            tx_queue: [can2040_transmit::new(); 4],
        }
    }
}

pub struct Can2040 {}

#[derive(Debug, defmt::Format)]
pub enum CanError {
    TransmissionError,
    ReceptionError,
}

impl embedded_can::Error for CanError {
    fn kind(&self) -> ErrorKind {
        todo!()
    }
}

impl fmt::Debug for can2040_msg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        unsafe {
            write!(
                f,
                "can2040_msg(D) {{ id: {:x?}, dlc: {:x?}, data: {:x?} }}",
                self.id,
                self.dlc,
                &self.__bindgen_anon_1.data[..self.dlc as usize]
            )
        }
    }
}

impl defmt::Format for can2040_msg {
    fn format(&self, f: defmt::Formatter) {
        unsafe {
            defmt::write!(
                f,
                "can2040_msg(F) {{ id: {:x}, dlc: {:x}, data: {:x} }}",
                self.id,
                self.dlc,
                &self.__bindgen_anon_1.data[..self.dlc as usize]
            )
        }
    }
}

#[derive(Clone, Debug, Format)]
pub struct CanFrame(can2040_msg);

impl embedded_can::Frame for CanFrame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }

        let mut data_arr = [0u8; 8];
        data_arr[..data.len()].copy_from_slice(data);

        Some(CanFrame {
            0: can2040_msg {
                id: match id.into() {
                    Id::Standard(sid) => sid.as_raw() as u32,
                    Id::Extended(_) => return None,
                },
                dlc: data.len() as u32,
                __bindgen_anon_1: can2040_msg__bindgen_ty_1 { data: data_arr },
            },
        })
    }

    fn new_remote(_id: impl Into<Id>, _dlc: usize) -> Option<Self> {
        todo!()
    }

    fn is_extended(&self) -> bool {
        false
    }

    fn is_remote_frame(&self) -> bool {
        false
    }

    fn id(&self) -> embedded_can::Id {
        embedded_can::Id::Standard(StandardId::new(self.0.id as u16).unwrap())
    }

    fn dlc(&self) -> usize {
        self.0.dlc as usize
    }

    fn data(&self) -> &[u8] {
        // 假设您可以从 __bindgen_anon_1 字段获取数据的byte slice
        unsafe { &self.0.__bindgen_anon_1.data[0..self.0.dlc as usize] }
    }
}

static RECEIVE_QUEUE: Mutex<RefCell<Vec<CanFrame>>> = Mutex::new(RefCell::new(Vec::new()));

unsafe extern "C" fn can2040_cb(_cd: *mut can2040, notify: u32, msg: *mut can2040_msg) {
    debug!("xfguo: can2040_cb 0, notify = {:x}, msg = {:?}", notify, *msg);
    if notify == CAN2040_NOTIFY_RX {
        let msg_copy = *msg.clone();
        cortex_m::interrupt::free(|cs| {
            RECEIVE_QUEUE.borrow(cs).borrow_mut().push(CanFrame(msg_copy));
        });
    }
    // TODO(zephyr): More code for TX / Err
}

impl embedded_can::nb::Can for Can2040 {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        unsafe {
            if let Some(cbus) = CBUS.as_mut() {
                let cbus_ptr = &mut *cbus as *mut _;
                if can2040_check_transmit(cbus_ptr) == 0 {
                    Err(nb::Error::WouldBlock)
                } else {
                    // critical path
                    can2040_transmit(cbus_ptr, frame as *const _ as *mut _);
                    Ok(None)
                }
            } else {
                Err(nb::Error::Other(CanError::TransmissionError))
            }
        }
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        cortex_m::interrupt::free(|cs| {
            let mut queue = RECEIVE_QUEUE.borrow(cs).borrow_mut();
            if queue.is_empty() {
                Err(nb::Error::WouldBlock)
            } else {
                return Ok(queue.remove(0));
            }
        })
    }
}

// TODO(zephyr): Remove blocking::Can, if we need blocking, we can use nb::block!()
impl embedded_can::blocking::Can for Can2040 {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        unsafe {
            if let Some(cbus) = CBUS.as_mut() {
                let cbus_ptr = &mut *cbus as *mut _;

                // Wait for CAN to be ready to transmit
                while can2040_check_transmit(cbus_ptr) == 0 {}
                can2040_transmit(cbus_ptr, frame as *const _ as *mut _);
            }
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        loop {
            if let Some(received_msg) = cortex_m::interrupt::free(|cs| {
                let mut queue = RECEIVE_QUEUE.borrow(cs).borrow_mut();
                if !queue.is_empty() {
                    Some(queue.remove(0))
                } else {
                    None
                }
            }) {
                return Ok(received_msg);
            }

            // Wait for interrupt (this will put the core to sleep until the next interrupt, i.e., the next message)
            wfi();
        }
    }
}

pub const RP2040_SYS_FREQ: u32 = 125_000_000;

static mut CBUS: Option<can2040> = None;

#[interrupt]
fn PIO0_IRQ_0() {
    unsafe {
        if let Some(cbus) = CBUS.as_mut() {
            can2040_pio_irq_handler(&mut *cbus as *mut _);
        }
    }
}

/// PIO0_IRQ_0 is hard-coded by the implementation of the Can2040 C library.
/// Therefore, if you need to enable can-bus on RP2040, you must reserve PIO0
/// for Can2040. Additionally, when enabling Can2040, we forcibly set the
/// priority of Can2040 to 0 to ensure that communication interrupts can
/// receive the most real-time response.
pub fn initialize_cbus(
    core: &mut cortex_m::Peripherals,
    baud_rate: u32,
    can_rx_id: u32,
    can_tx_id: u32,
) -> Can2040 {
    unsafe {
        assert!(CBUS.is_none());
        CBUS = Some(can2040::new());
        let cbus = CBUS.as_mut().unwrap();
        let cbus_ptr = &mut *cbus as *mut _;
        can2040_setup(cbus_ptr, 0);
        can2040_callback_config(cbus_ptr, Some(can2040_cb));
        can2040_start(cbus_ptr, RP2040_SYS_FREQ, baud_rate, can_rx_id, can_tx_id);

        // Enable interrupts and set priority for it.
        core.NVIC.set_priority(pac::Interrupt::PIO0_IRQ_0, 0);
        pac::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
        Can2040 {}
    }
}
