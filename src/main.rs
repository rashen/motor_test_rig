//! Bench tool: sends motor CAN commands while the ack button is held.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embedded_can::{Frame, Id, StandardId};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{BusTimeout, Config as I2cConfig, FsmTimeout, I2c, SoftwareTimeout},
    interrupt::{self, software::SoftwareInterruptControl},
    peripherals,
    time::Rate,
    timer::timg::TimerGroup,
    twai::{self, EspTwaiFrame, TwaiRx, TwaiTx},
};

esp_bootloader_esp_idf::esp_app_desc!(
    env!("CARGO_PKG_VERSION"),
    env!("CARGO_PKG_NAME"),
    esp_bootloader_esp_idf::BUILD_TIME,
    esp_bootloader_esp_idf::BUILD_DATE,
    esp_bootloader_esp_idf::ESP_IDF_COMPATIBLE_VERSION,
    esp_bootloader_esp_idf::MMU_PAGE_SIZE,
    0,
    u16::MAX,
    esp_bootloader_esp_idf::SECURE_VERSION
);

const CAN_BAUDRATE: twai::BaudRate = twai::BaudRate::B250K;

mod can_ids {
    // To the wheel motors.
    pub const SET_MOTOR_SPEED: u32 = 0x167;
    pub const SET_MOTOR_MODE: u32 = 0x168;
    // From the wheel motors.
    pub const LEFT_MOTOR_STATE: u32 = 0x141;
    pub const LEFT_MOTOR_STATE_2: u32 = 0x142;
    pub const RIGHT_MOTOR_STATE: u32 = 0x143;
    pub const RIGHT_MOTOR_STATE_2: u32 = 0x144;
    // To the lift motor.
    pub const LIFT_MOTOR_INPUT: u32 = 0x300;
    pub const LIFT_MOTOR_CONFIG: u32 = 0x301;
    // From the lift motor.
    pub const LIFT_MOTOR_STATUS: u32 = 0x310;
    pub const LIFT_MOTOR_POSITION: u32 = 0x311;
}

// IO expander with the front panel buttons. Same chip as main_control's `mcp_inputs`.
const MCP_INPUTS_ADDRESS: u8 = 0x21;
const MCP_REG_IODIRA: u8 = 0x00;
const MCP_REG_GPIOA: u8 = 0x12;
const ACK_BUTTON_BIT: u8 = 1;

// Command sent while the button is held. Mode 33 is speed mode (see main_control).
const MOTOR_MODE_RUN: u16 = 33;
const MOTOR_MODE_STOP: u16 = 0;
const RAMP_ACC: i16 = 800;
const RAMP_DEC: i16 = 900;
const TORQUE: i16 = 200;
const RUN_SPEED: i32 = 1000;

// Lift motor: speed ramp mode (33), speed given in `target_input`.
const LIFT_MODE_RUN: u16 = 33;
const LIFT_MODE_STOP: u16 = 0;
const LIFT_RAMP_SPEED: i16 = 10_000;
const LIFT_TORQUE: u16 = 200;
const LIFT_RUN_SPEED: i32 = 1000;

const LOOP_TIME: Duration = Duration::from_millis(20);
const RX_LOG_INTERVAL_MS: u64 = 500;

fn std_frame(id: u32, data: &[u8; 8]) -> EspTwaiFrame {
    let id = Id::Standard(StandardId::new(id as u16).expect("invalid CAN id"));
    EspTwaiFrame::new(id, data).expect("invalid CAN frame")
}

fn motor_mode_frame(mode: u16, acc: i16, dec: i16, torque: i16) -> EspTwaiFrame {
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&mode.to_le_bytes());
    data[2..4].copy_from_slice(&acc.to_le_bytes());
    data[4..6].copy_from_slice(&dec.to_le_bytes());
    data[6..8].copy_from_slice(&torque.to_le_bytes());
    std_frame(can_ids::SET_MOTOR_MODE, &data)
}

fn motor_speed_frame(left: i32, right: i32) -> EspTwaiFrame {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&left.to_le_bytes());
    data[4..8].copy_from_slice(&right.to_le_bytes());
    std_frame(can_ids::SET_MOTOR_SPEED, &data)
}

#[embassy_executor::task]
async fn can_receiver(mut rx: TwaiRx<'static, esp_hal::Async>) -> ! {
    // Last log time per id: wheel motor states 0x141..=0x144, then lift status/position.
    let mut last_logged = [0u64; 6];
    loop {
        match rx.receive_async().await {
            Ok(frame) => {
                let Id::Standard(id) = Frame::id(&frame) else {
                    continue;
                };
                let id = id.as_raw() as u32;
                let slot = match id {
                    can_ids::LEFT_MOTOR_STATE
                    | can_ids::LEFT_MOTOR_STATE_2
                    | can_ids::RIGHT_MOTOR_STATE
                    | can_ids::RIGHT_MOTOR_STATE_2 => (id - can_ids::LEFT_MOTOR_STATE) as usize,
                    can_ids::LIFT_MOTOR_STATUS => 4,
                    can_ids::LIFT_MOTOR_POSITION => 5,
                    _ => continue,
                };
                let now = Instant::now().as_millis();
                if now.saturating_sub(last_logged[slot]) < RX_LOG_INTERVAL_MS {
                    continue;
                }
                last_logged[slot] = now;
                esp_println::println!("RX {:#05x} {:02x?}", id, frame.data());
            }
            Err(e) => {
                esp_println::println!("CAN receive error: {e:?}");
                Timer::after(Duration::from_millis(50)).await;
            }
        }
    }
}

fn lift_config_frame(max_torque: u16) -> EspTwaiFrame {
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&max_torque.to_le_bytes());
    std_frame(can_ids::LIFT_MOTOR_CONFIG, &data)
}

fn lift_input_frame(mode: u16, ramp_speed_max: i16, target: i32) -> EspTwaiFrame {
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&mode.to_le_bytes());
    data[2..4].copy_from_slice(&ramp_speed_max.to_le_bytes());
    data[4..8].copy_from_slice(&target.to_le_bytes());
    std_frame(can_ids::LIFT_MOTOR_INPUT, &data)
}

/// Read MCP23017 port A. Returns 0 (nothing pressed) on bus errors.
async fn read_buttons(i2c: &mut I2c<'static, esp_hal::Async>) -> u8 {
    let mut port_a = [0u8];
    match i2c
        .write_read_async(MCP_INPUTS_ADDRESS, &[MCP_REG_GPIOA], &mut port_a)
        .await
    {
        Ok(()) => port_a[0],
        Err(e) => {
            esp_println::println!("I2C read error: {e:?}");
            0
        }
    }
}

async fn send(tx: &mut TwaiTx<'static, esp_hal::Async>, frame: &EspTwaiFrame) {
    if let Err(e) = tx.transmit_async(frame).await {
        esp_println::println!("CAN transmit error: {e:?}");
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

    esp_println::println!("motor_test_rig {} starting", env!("CARGO_PKG_VERSION"));

    let mut can_config = twai::TwaiConfiguration::new(
        peripherals.TWAI0,
        peripherals.GPIO25,
        peripherals.GPIO24,
        CAN_BAUDRATE,
        twai::TwaiMode {
            self_test: false,
            listen_only: false,
            loopback: false,
        },
    )
    .into_async();
    can_config.set_filter(twai::filter::SingleStandardFilter::new(
        b"xxxxxxxxxxx",
        b"x",
        [b"xxxxxxxx", b"xxxxxxxx"],
    ));
    let can = can_config.start();
    interrupt::enable(
        peripherals::Interrupt::TWAI0,
        interrupt::Priority::Priority1,
    );
    let (rx, mut tx) = can.split();

    // Take the CAN transceiver out of standby.
    let _stb = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());

    // Reset the IO expanders, then give them time to come up.
    let mut reset_pin = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
    Timer::after(Duration::from_millis(10)).await;
    reset_pin.set_high();
    Timer::after(Duration::from_millis(20)).await;

    // Max out all I2C timeouts, same workaround as main_control.
    let fsm_timeout = FsmTimeout::new_const::<23>();
    let i2c_config = I2cConfig::default()
        .with_frequency(Rate::from_khz(100))
        .with_software_timeout(SoftwareTimeout::None)
        .with_timeout(BusTimeout::Maximum)
        .with_scl_main_st_timeout(fsm_timeout)
        .with_scl_st_timeout(fsm_timeout);
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO2)
        .with_scl(peripherals.GPIO3)
        .into_async();

    // All pins as inputs (the reset default, but make it explicit).
    if let Err(e) = i2c
        .write_async(MCP_INPUTS_ADDRESS, &[MCP_REG_IODIRA, 0xff, 0xff])
        .await
    {
        esp_println::println!("IO expander init error: {e:?}");
    }

    spawner.spawn(can_receiver(rx).expect("Failed spawning CAN receiver"));

    let stop_mode = motor_mode_frame(MOTOR_MODE_STOP, RAMP_ACC, RAMP_DEC, TORQUE);
    let stop_speed = motor_speed_frame(0, 0);
    let run_mode = motor_mode_frame(MOTOR_MODE_RUN, RAMP_ACC, RAMP_DEC, TORQUE);
    let run_speed = motor_speed_frame(RUN_SPEED, RUN_SPEED);

    let lift_stop = lift_input_frame(LIFT_MODE_STOP, LIFT_RAMP_SPEED, 0);
    let lift_run = lift_input_frame(LIFT_MODE_RUN, LIFT_RAMP_SPEED, LIFT_RUN_SPEED);

    let mut was_pressed = false;
    let mut lift_config_sent = false;
    loop {
        let buttons = read_buttons(&mut i2c).await;
        let pressed = (buttons >> ACK_BUTTON_BIT) & 1 == 1;
        if pressed != was_pressed {
            esp_println::println!("Button {}", if pressed { "pressed" } else { "released" });
            was_pressed = pressed;
        }

        if !lift_config_sent {
            send(&mut tx, &lift_config_frame(LIFT_TORQUE)).await;
            lift_config_sent = true;
        }
        send(&mut tx, if pressed { &lift_run } else { &lift_stop }).await;

        if pressed {
            send(&mut tx, &run_mode).await;
            send(&mut tx, &run_speed).await;
        } else {
            send(&mut tx, &stop_mode).await;
            send(&mut tx, &stop_speed).await;
        }

        Timer::after(LOOP_TIME).await;
    }
}
