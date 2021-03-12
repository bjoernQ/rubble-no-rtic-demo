#![no_std]
#![no_main]
#![warn(rust_2018_idioms)]

use core::mem::MaybeUninit;

use cortex_m_rt::entry;
use hal::prelude::OutputPin;
// We need to import this crate explicitly so we have a panic handler
use panic_rtt_target as _;

use nrf52832_hal::pac::interrupt;
use nrf52832_hal::pac::Interrupt;

use nrf52832_hal as hal;
use nrf52832_hal::gpio::Level;
use rtt_target::{rprintln, rtt_init_print};
use rubble::{
    att::{Attribute, AttributeProvider, Handle},
    config::Config,
    l2cap::{BleChannelMap, L2CAPState},
    link::{
        ad_structure::AdStructure,
        queue::{PacketQueue, SimpleQueue},
        LinkLayer, Responder, MIN_PDU_BUF,
    },
    security::NoSecurity,
    time::{Duration, Timer},
    uuid::{Uuid128, Uuid16},
};
use rubble_nrf5x::{radio::BleRadio, timer::BleTimer, utils::get_device_address};

pub enum AppConfig {}

impl Config for AppConfig {
    type Timer = BleTimer<hal::pac::TIMER0>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<DemoAttrs, NoSecurity>;
    type PacketQueue = &'static mut SimpleQueue;
}

static LOGGER: SimpleLogger = SimpleLogger;

static mut BLE_TX_BUF: [u8; MIN_PDU_BUF] = [0u8; MIN_PDU_BUF];
static mut BLE_RX_BUF: [u8; MIN_PDU_BUF] = [0u8; MIN_PDU_BUF];
static mut TX_QUEUE: SimpleQueue = SimpleQueue::new();
static mut RX_QUEUE: SimpleQueue = SimpleQueue::new();

static mut BLE_LL: MaybeUninit<LinkLayer<AppConfig>> = MaybeUninit::uninit();
static mut RADIO: MaybeUninit<BleRadio> = MaybeUninit::uninit();
static mut BLE_R: MaybeUninit<Responder<AppConfig>> = MaybeUninit::uninit();

#[entry]
unsafe fn main() -> ! {
    rtt_init_print!();
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(log::LevelFilter::Trace);

    let p = hal::pac::Peripherals::take().unwrap();

    cortex_m::peripheral::NVIC::unmask(Interrupt::RADIO);
    cortex_m::peripheral::NVIC::unmask(Interrupt::TIMER0);

    // On reset, the internal high frequency clock is already used, but we
    // also need to switch to the external HF oscillator. This is needed
    // for Bluetooth to work.
    let _clocks = hal::clocks::Clocks::new(p.CLOCK).enable_ext_hfosc();

    let ble_timer = BleTimer::init(p.TIMER0);

    // Determine device address
    let device_address = get_device_address();

    *RADIO.as_mut_ptr() = BleRadio::new(p.RADIO, &p.FICR, &mut BLE_TX_BUF, &mut BLE_RX_BUF);
    let mut radio = RADIO.as_mut_ptr().as_mut().unwrap();

    // let log_sink = logger::init(ble_timer.create_stamp_source());

    // Create TX/RX queues
    let (tx, tx_cons) = TX_QUEUE.split();
    let (rx_prod, rx) = RX_QUEUE.split();

    // Create the actual BLE stack objects
    *BLE_LL.as_mut_ptr() = LinkLayer::<AppConfig>::new(device_address, ble_timer);

    let ble_ll = BLE_LL.as_mut_ptr().as_mut().unwrap();

    *BLE_R.as_mut_ptr() = Responder::new(
        tx,
        rx,
        L2CAPState::new(BleChannelMap::with_attributes(DemoAttrs::default())),
    );

    // Send advertisement and set up regular interrupt
    let next_update = ble_ll
        .start_advertise(
            Duration::from_millis(20), // in order to make Android see the device we need a fast interval
            &[AdStructure::ShortenedLocalName("Rubble")],
            &mut radio,
            tx_cons,
            rx_prod,
        )
        .unwrap();

    ble_ll.timer().configure_interrupt(next_update);

    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let mut led = port0.p0_17.into_push_pull_output(Level::Low);

    rprintln!("READY.");
    loop {
        if ble_ll.is_advertising() || ble_ll.is_connected() {
            led.set_low().unwrap();
        } else {
            led.set_high().unwrap();
        }
    }
}

#[interrupt]
fn TIMER0() {
    let ble_ll = unsafe { BLE_LL.as_mut_ptr().as_mut() }.unwrap();

    let timer = ble_ll.timer();
    if !timer.is_interrupt_pending() {
        return;
    }
    timer.clear_interrupt();

    let mut radio = unsafe { RADIO.as_mut_ptr().as_mut() }.unwrap();

    let cmd = ble_ll.update_timer(&mut radio);
    radio.configure_receiver(cmd.radio);

    ble_ll.timer().configure_interrupt(cmd.next_update);

    if cmd.queued_work {
        // Fully drain the packet queue
        let ble_r = unsafe { BLE_R.as_mut_ptr().as_mut() }.unwrap();
        while ble_r.has_work() {
            ble_r.process_one().unwrap();
        }
    }
}

#[interrupt]
fn RADIO() {
    let ble_ll = unsafe { BLE_LL.as_mut_ptr().as_mut() }.unwrap();
    let radio = unsafe { RADIO.as_mut_ptr().as_mut() }.unwrap();

    let timer = ble_ll.timer();

    if let Some(cmd) = radio.recv_interrupt(timer.now(), ble_ll) {
        radio.configure_receiver(cmd.radio);
        ble_ll.timer().configure_interrupt(cmd.next_update);

        if cmd.queued_work {
            // Fully drain the packet queue
            let ble_r = unsafe { BLE_R.as_mut_ptr().as_mut() }.unwrap();
            while ble_r.has_work() {
                ble_r.process_one().unwrap();
            }
        }
    }
}

const PRIMARY_SERVICE_UUID16: Uuid16 = Uuid16(0x2800);
const CHARACTERISTIC_UUID16: Uuid16 = Uuid16(0x2803);

// Randomly generated
// a86a62f0-5d26-4538-b364-565496151500
const MY_SERVICE_UUID128: [u8; 16] = [
    0x00, 0x15, 0x15, 0x96, 0x54, 0x56, 0x64, 0xB3, 0x38, 0x45, 0x26, 0x5D, 0xF0, 0x62, 0x6A, 0xA8,
];

const MY_CHARACTERISTIC_UUID128: [u8; 16] = [
    0x01, 0x15, 0x15, 0x96, 0x54, 0x56, 0x64, 0xB3, 0x38, 0x45, 0x26, 0x5D, 0xF0, 0x62, 0x6A, 0xA8,
];

#[rustfmt::skip]
const MY_CHARACTERISTIC_DECL_VALUE: [u8; 19] = [
    0x02, // 0x02 = read
    0x03, 0x00,  // 2 byte handle pointing to characteristic value

    // 128-bit UUID of characteristic value (copied from above constant)
    0x01, 0x15, 0x15, 0x96, 0x54, 0x56, 0x64, 0xB3, 0x38, 0x45, 0x26, 0x5D, 0xF0, 0x62, 0x6A,0xA8,
];

pub struct DemoAttrs {
    attribs: [Attribute<&'static [u8]>; 3],
}

impl Default for DemoAttrs {
    fn default() -> Self {
        Self {
            attribs: [
                Attribute::new(
                    PRIMARY_SERVICE_UUID16.into(),
                    Handle::from_raw(0x0001),
                    &MY_SERVICE_UUID128,
                ),
                Attribute::new(
                    CHARACTERISTIC_UUID16.into(),
                    Handle::from_raw(0x0002),
                    &MY_CHARACTERISTIC_DECL_VALUE,
                ),
                Attribute::new(
                    Uuid128::from_bytes(MY_CHARACTERISTIC_UUID128).into(),
                    Handle::from_raw(0x0003),
                    b"Hello World",
                ),
            ],
        }
    }
}

impl AttributeProvider for DemoAttrs {
    /// Applies a function to all attributes with handles within the specified range
    fn for_attrs_in_range(
        &mut self,
        range: rubble::att::HandleRange,
        mut f: impl FnMut(&Self, &Attribute<dyn AsRef<[u8]>>) -> Result<(), rubble::Error>,
    ) -> Result<(), rubble::Error> {
        let start = range.start().as_u16();
        let end = range.end().as_u16();
        let range_u16 = start..=end;

        for attr in &self.attribs {
            if range_u16.contains(&attr.handle.as_u16()) {
                f(self, attr)?;
            }
        }

        Ok(())
    }

    fn is_grouping_attr(&self, uuid: rubble::att::AttUuid) -> bool {
        uuid == PRIMARY_SERVICE_UUID16 || uuid == CHARACTERISTIC_UUID16
    }

    fn group_end(
        &self,
        handle: rubble::att::Handle,
    ) -> Option<&rubble::att::Attribute<dyn AsRef<[u8]>>> {
        match handle.as_u16() {
            0x0001 => Some(&self.attribs[2]),
            0x0002 => Some(&self.attribs[2]),
            _ => None,
        }
    }
}

struct SimpleLogger;

impl log::Log for SimpleLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.level() <= log::Level::Trace
    }

    fn log(&self, record: &log::Record<'_>) {
        if self.enabled(record.metadata()) {
            rprintln!("{} - {}", record.level(), record.args());
        }
    }

    fn flush(&self) {}
}
