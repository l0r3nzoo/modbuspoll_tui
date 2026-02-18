mod arguments;

use arguments::{Cli, ConnectionInfo, Mode};
use clap::Parser;
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use ratatui::{prelude::*, widgets::*};
use std::{
    io,
    sync::mpsc,
    time::{Duration, Instant},
};
use tokio::runtime::Runtime;
use tokio_modbus::prelude::*;
use tokio_modbus::slave::SlaveContext;

// ── Modbus worker messages ────────────────────────────────────────────────────

/// Commands sent from TUI → worker
#[derive(Debug)]
enum PollCommand {
    Poll {
        slave_id: u8,
        reg_type: RegTypeKind,
        address: u16,
        count: u16,
    },
    Shutdown,
}

/// Results sent from worker → TUI
enum PollResult {
    Data {
        registers: Vec<u16>,
        tx_frame: Vec<u8>,
        rx_frame: Vec<u8>,
    },
    Error(String),
}

/// Which Modbus function to call
#[derive(Debug, Clone, Copy)]
enum RegTypeKind {
    Coil,
    Discrete,
    Holding,
    Input,
}

// ── CRC16 Modbus ──────────────────────────────────────────────────────────────

/// Compute Modbus RTU CRC16.
/// Returns CRC as u16 — low byte first per Modbus RTU spec.
fn crc16_modbus(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 0x0001 != 0 {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

/// Build a Modbus RTU frame: [slave_id] + pdu + [crc_lo, crc_hi]
fn build_rtu_frame(slave_id: u8, pdu: &[u8]) -> Vec<u8> {
    let mut frame = vec![slave_id];
    frame.extend_from_slice(pdu);
    let crc = crc16_modbus(&frame);
    frame.push((crc & 0xFF) as u8); // CRC low byte first
    frame.push((crc >> 8) as u8); // CRC high byte second
    frame
}

/// Build a Modbus TCP frame: MBAP header + PDU
/// MBAP: TransactionID(2) + ProtocolID(2) + Length(2) + UnitID(1)
fn build_tcp_frame(slave_id: u8, pdu: &[u8], transaction_id: u16) -> Vec<u8> {
    let length = (1 + pdu.len()) as u16; // UnitID(1) + PDU
    let mut frame = vec![
        (transaction_id >> 8) as u8,
        (transaction_id & 0xFF) as u8,
        0x00,
        0x00, // Protocol ID always 0
        (length >> 8) as u8,
        (length & 0xFF) as u8,
        slave_id, // Unit ID
    ];
    frame.extend_from_slice(pdu);
    frame
}

// ── Modbus worker (runs inside tokio) ────────────────────────────────────────

async fn run_modbus_tcp(
    ip: String,
    port: u16,
    cmd_rx: mpsc::Receiver<PollCommand>,
    res_tx: mpsc::Sender<PollResult>,
) {
    let addr = format!("{}:{}", ip, port);
    let socket_addr: std::net::SocketAddr = match addr.parse() {
        Ok(a) => a,
        Err(e) => {
            let _ = res_tx.send(PollResult::Error(format!("Bad address: {}", e)));
            return;
        }
    };

    let mut ctx = match tcp::connect(socket_addr).await {
        Ok(c) => c,
        Err(e) => {
            let _ = res_tx.send(PollResult::Error(format!("TCP connect failed: {}", e)));
            return;
        }
    };

    let mut transaction_id: u16 = 0;

    for cmd in cmd_rx {
        match cmd {
            PollCommand::Shutdown => break,
            PollCommand::Poll {
                slave_id,
                reg_type,
                address,
                count,
            } => {
                ctx.set_slave(Slave(slave_id));
                transaction_id = transaction_id.wrapping_add(1);
                let result = do_poll(
                    &mut ctx,
                    reg_type,
                    address,
                    count,
                    slave_id,
                    false,
                    transaction_id,
                )
                .await;
                let _ = res_tx.send(result);
            }
        }
    }
}

async fn run_modbus_rtu(
    comport: String,
    baudrate: u32,
    parity: String,
    stopbits: u8,
    databits: u8,
    cmd_rx: mpsc::Receiver<PollCommand>,
    res_tx: mpsc::Sender<PollResult>,
) {
    use tokio_serial::SerialStream;

    let parity = match parity.to_lowercase().as_str() {
        "even" => tokio_serial::Parity::Even,
        "odd" => tokio_serial::Parity::Odd,
        _ => tokio_serial::Parity::None,
    };
    let stop_bits = match stopbits {
        2 => tokio_serial::StopBits::Two,
        _ => tokio_serial::StopBits::One,
    };
    let data_bits = match databits {
        5 => tokio_serial::DataBits::Five,
        6 => tokio_serial::DataBits::Six,
        7 => tokio_serial::DataBits::Seven,
        _ => tokio_serial::DataBits::Eight,
    };

    let builder = tokio_serial::new(&comport, baudrate)
        .parity(parity)
        .stop_bits(stop_bits)
        .data_bits(data_bits)
        .timeout(Duration::from_millis(500));

    let port = match SerialStream::open(&builder) {
        Ok(p) => p,
        Err(e) => {
            let _ = res_tx.send(PollResult::Error(format!("Serial open failed: {}", e)));
            return;
        }
    };

    let mut ctx = rtu::attach_slave(port, Slave(1));

    for cmd in cmd_rx {
        match cmd {
            PollCommand::Shutdown => break,
            PollCommand::Poll {
                slave_id,
                reg_type,
                address,
                count,
            } => {
                ctx.set_slave(Slave(slave_id));
                let result = do_poll(
                    &mut ctx, reg_type, address, count, slave_id, true,
                    0, // transaction_id unused for RTU
                )
                .await;
                let _ = res_tx.send(result);
            }
        }
    }
}

async fn do_poll(
    ctx: &mut impl Reader,
    reg_type: RegTypeKind,
    address: u16,
    count: u16,
    slave_id: u8,
    use_rtu: bool,
    transaction_id: u16,
) -> PollResult {
    let fc: u8 = match reg_type {
        RegTypeKind::Coil => 0x01,
        RegTypeKind::Discrete => 0x02,
        RegTypeKind::Holding => 0x03,
        RegTypeKind::Input => 0x04,
    };

    // ── Build real TX frame ──────────────────────────────────────────────────
    // PDU = FC + StartAddr(2) + Quantity(2)
    let req_pdu = vec![
        fc,
        (address >> 8) as u8,
        address as u8,
        (count >> 8) as u8,
        count as u8,
    ];

    let tx_frame = if use_rtu {
        build_rtu_frame(slave_id, &req_pdu)
    } else {
        build_tcp_frame(slave_id, &req_pdu, transaction_id)
    };

    // ── Execute Modbus request ───────────────────────────────────────────────
    let result: Result<Vec<u16>, Box<dyn std::error::Error>> = match reg_type {
        RegTypeKind::Coil => match ctx.read_coils(address, count).await {
            Ok(Ok(v)) => Ok(v.iter().map(|b| *b as u16).collect()),
            Ok(Err(e)) => Err(format!("Modbus exception: {}", u8::from(e)).into()),
            Err(e) => Err(e.into()),
        },
        RegTypeKind::Discrete => match ctx.read_discrete_inputs(address, count).await {
            Ok(Ok(v)) => Ok(v.iter().map(|b| *b as u16).collect()),
            Ok(Err(e)) => Err(format!("Modbus exception: {}", u8::from(e)).into()),
            Err(e) => Err(e.into()),
        },
        RegTypeKind::Holding => match ctx.read_holding_registers(address, count).await {
            Ok(Ok(v)) => Ok(v),
            Ok(Err(e)) => Err(format!("Modbus exception: {}", u8::from(e)).into()),
            Err(e) => Err(e.into()),
        },
        RegTypeKind::Input => match ctx.read_input_registers(address, count).await {
            Ok(Ok(v)) => Ok(v),
            Ok(Err(e)) => Err(format!("Modbus exception: {}", u8::from(e)).into()),
            Err(e) => Err(e.into()),
        },
    };

    match result {
        Ok(registers) => {
            // ── Build real RX frame ──────────────────────────────────────────
            // Response PDU = FC + ByteCount + Data
            let byte_count = (registers.len() * 2) as u8;
            let mut resp_pdu = vec![fc, byte_count];
            for r in &registers {
                resp_pdu.push((r >> 8) as u8);
                resp_pdu.push(*r as u8);
            }

            let rx_frame = if use_rtu {
                build_rtu_frame(slave_id, &resp_pdu)
            } else {
                build_tcp_frame(slave_id, &resp_pdu, transaction_id)
            };

            PollResult::Data {
                registers,
                tx_frame,
                rx_frame,
            }
        }
        Err(e) => PollResult::Error(e.to_string()),
    }
}

// ── Data Types ────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq)]
enum DataType {
    Int16,
    Int32Msb,
    Int32Lsb,
    Int64Msb,
    Int64Lsb,
    UInt16,
    UInt32Msb,
    UInt32Lsb,
    UInt64Msb,
    UInt64Lsb,
    FloatMsb,
    FloatLsb,
    DoubleMsb,
    DoubleLsb,
}

impl DataType {
    const ALL: &'static [DataType] = &[
        DataType::Int16,
        DataType::Int32Msb,
        DataType::Int32Lsb,
        DataType::Int64Msb,
        DataType::Int64Lsb,
        DataType::UInt16,
        DataType::UInt32Msb,
        DataType::UInt32Lsb,
        DataType::UInt64Msb,
        DataType::UInt64Lsb,
        DataType::FloatMsb,
        DataType::FloatLsb,
        DataType::DoubleMsb,
        DataType::DoubleLsb,
    ];

    fn label(self) -> &'static str {
        match self {
            DataType::Int16 => "Int16",
            DataType::Int32Msb => "Int32 MSB",
            DataType::Int32Lsb => "Int32 LSB",
            DataType::Int64Msb => "Int64 MSB",
            DataType::Int64Lsb => "Int64 LSB",
            DataType::UInt16 => "UInt16",
            DataType::UInt32Msb => "UInt32 MSB",
            DataType::UInt32Lsb => "UInt32 LSB",
            DataType::UInt64Msb => "UInt64 MSB",
            DataType::UInt64Lsb => "UInt64 LSB",
            DataType::FloatMsb => "Float MSB",
            DataType::FloatLsb => "Float LSB",
            DataType::DoubleMsb => "Double MSB",
            DataType::DoubleLsb => "Double LSB",
        }
    }

    fn regs_per_value(self) -> usize {
        match self {
            DataType::Int16 | DataType::UInt16 => 1,
            DataType::Int32Msb
            | DataType::Int32Lsb
            | DataType::UInt32Msb
            | DataType::UInt32Lsb
            | DataType::FloatMsb
            | DataType::FloatLsb => 2,
            _ => 4,
        }
    }
}

// ── Register Type ─────────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq)]
enum RegType {
    Coil,
    Discrete,
    Holding,
    Input,
}

impl RegType {
    const ALL: &'static [RegType] = &[
        RegType::Coil,
        RegType::Discrete,
        RegType::Holding,
        RegType::Input,
    ];

    fn label(self) -> &'static str {
        match self {
            RegType::Coil => "Coil (0x)",
            RegType::Discrete => "Discrete (1x)",
            RegType::Holding => "Holding (4x)",
            RegType::Input => "Input (3x)",
        }
    }

    fn prefix(self) -> u32 {
        match self {
            RegType::Coil => 0,
            RegType::Discrete => 1,
            RegType::Holding => 4,
            RegType::Input => 3,
        }
    }

    fn display_address(self, suffix: u16) -> u32 {
        self.prefix() * 10_000 + suffix as u32 + 1
    }

    fn protocol_address(suffix: u16) -> u16 {
        suffix
    }

    fn kind(self) -> RegTypeKind {
        match self {
            RegType::Coil => RegTypeKind::Coil,
            RegType::Discrete => RegTypeKind::Discrete,
            RegType::Holding => RegTypeKind::Holding,
            RegType::Input => RegTypeKind::Input,
        }
    }
}

// ── Focus ─────────────────────────────────────────────────────────────────────

#[derive(PartialEq, Clone, Copy)]
enum Focus {
    RegType,
    Address,
    Length,
    Slave,
    Interval,
}

impl Focus {
    fn next(self) -> Self {
        match self {
            Focus::RegType => Focus::Address,
            Focus::Address => Focus::Length,
            Focus::Length => Focus::Slave,
            Focus::Slave => Focus::Interval,
            Focus::Interval => Focus::RegType,
        }
    }
}

// ── Traffic ───────────────────────────────────────────────────────────────────

struct TrafficFrame {
    tx: bool,
    bytes: Vec<u8>,
    seq: u64,
}

// ── App ───────────────────────────────────────────────────────────────────────

struct App {
    conn: String,
    is_rtu: bool, // true = RTU, false = TCP — drives frame formatting labels

    reg_type_idx: usize,
    address_suffix: String,
    length: String,
    slave: String,
    interval_ms: String,

    dt_idx: usize,
    focus: Focus,

    registers: Vec<u16>,
    traffic: Vec<TrafficFrame>,
    traffic_scroll: usize,
    poll_seq: u64,
    last_poll: Instant,
    status: String,

    cmd_tx: mpsc::SyncSender<PollCommand>,
    res_rx: mpsc::Receiver<PollResult>,
}

impl App {
    fn new(
        conn: ConnectionInfo,
        is_rtu: bool,
        cmd_tx: mpsc::SyncSender<PollCommand>,
        res_rx: mpsc::Receiver<PollResult>,
    ) -> Self {
        Self {
            conn: conn.mode_label,
            is_rtu,
            reg_type_idx: 2,
            address_suffix: "0".into(),
            length: "10".into(),
            slave: "1".into(),
            interval_ms: "1000".into(),
            dt_idx: 0,
            focus: Focus::Address,
            registers: vec![0u16; 64],
            traffic: Vec::new(),
            traffic_scroll: 0,
            poll_seq: 0,
            last_poll: Instant::now(),
            status: "Connecting...".into(),
            cmd_tx,
            res_rx,
        }
    }

    fn reg_type(&self) -> RegType {
        RegType::ALL[self.reg_type_idx]
    }

    fn suffix(&self) -> u16 {
        self.address_suffix.parse::<u16>().unwrap_or(0)
    }

    fn display_address(&self) -> u32 {
        self.reg_type().display_address(self.suffix())
    }

    fn protocol_address(&self) -> u16 {
        RegType::protocol_address(self.suffix())
    }

    fn interval(&self) -> Duration {
        Duration::from_millis(self.interval_ms.parse().unwrap_or(1000))
    }

    fn send_poll(&self) {
        let _ = self.cmd_tx.try_send(PollCommand::Poll {
            slave_id: self.slave.parse().unwrap_or(1),
            reg_type: self.reg_type().kind(),
            address: self.protocol_address(),
            count: self.length.parse::<u16>().unwrap_or(10).min(125),
        });
    }

    fn drain_results(&mut self) {
        while let Ok(res) = self.res_rx.try_recv() {
            match res {
                PollResult::Data {
                    registers,
                    tx_frame,
                    rx_frame,
                } => {
                    self.poll_seq += 1;
                    let len = registers.len().min(64);
                    for i in 0..len {
                        self.registers[i] = registers[i];
                    }
                    self.push_frame(true, tx_frame);
                    self.push_frame(false, rx_frame);
                    self.status = format!("Poll #{} OK", self.poll_seq);
                }
                PollResult::Error(e) => {
                    self.status = format!("Error: {}", e);
                }
            }
        }
    }

    fn push_frame(&mut self, tx: bool, bytes: Vec<u8>) {
        self.traffic.push(TrafficFrame {
            tx,
            bytes,
            seq: self.poll_seq,
        });
        if self.traffic.len() > 500 {
            self.traffic.remove(0);
        }
        self.traffic_scroll = self.traffic.len().saturating_sub(1);
    }

    fn format_value(&self, start: usize) -> String {
        let dt = DataType::ALL[self.dt_idx];
        let r = &self.registers;
        let len = self.length.parse::<usize>().unwrap_or(10).min(64);
        if start >= len {
            return "—".into();
        }

        match dt {
            DataType::Int16 => format!("{}", r[start] as i16),
            DataType::UInt16 => format!("{}", r[start]),

            DataType::Int32Msb | DataType::Int32Lsb => {
                if start + 1 >= len {
                    return "—".into();
                }
                let (h, l) = if dt == DataType::Int32Msb {
                    (r[start], r[start + 1])
                } else {
                    (r[start + 1], r[start])
                };
                format!("{}", (((h as u32) << 16) | l as u32) as i32)
            }
            DataType::UInt32Msb | DataType::UInt32Lsb => {
                if start + 1 >= len {
                    return "—".into();
                }
                let (h, l) = if dt == DataType::UInt32Msb {
                    (r[start], r[start + 1])
                } else {
                    (r[start + 1], r[start])
                };
                format!("{}", ((h as u32) << 16) | l as u32)
            }
            DataType::FloatMsb | DataType::FloatLsb => {
                if start + 1 >= len {
                    return "—".into();
                }
                let (h, l) = if dt == DataType::FloatMsb {
                    (r[start], r[start + 1])
                } else {
                    (r[start + 1], r[start])
                };
                format!("{:.4}", f32::from_bits(((h as u32) << 16) | l as u32))
            }
            DataType::Int64Msb | DataType::Int64Lsb => {
                if start + 3 >= len {
                    return "—".into();
                }
                let w: Vec<u16> = if dt == DataType::Int64Msb {
                    r[start..start + 4].into()
                } else {
                    r[start..start + 4].iter().cloned().rev().collect()
                };
                let v = ((w[0] as u64) << 48)
                    | ((w[1] as u64) << 32)
                    | ((w[2] as u64) << 16)
                    | w[3] as u64;
                format!("{}", v as i64)
            }
            DataType::UInt64Msb | DataType::UInt64Lsb => {
                if start + 3 >= len {
                    return "—".into();
                }
                let w: Vec<u16> = if dt == DataType::UInt64Msb {
                    r[start..start + 4].into()
                } else {
                    r[start..start + 4].iter().cloned().rev().collect()
                };
                let v = ((w[0] as u64) << 48)
                    | ((w[1] as u64) << 32)
                    | ((w[2] as u64) << 16)
                    | w[3] as u64;
                format!("{}", v)
            }
            DataType::DoubleMsb | DataType::DoubleLsb => {
                if start + 3 >= len {
                    return "—".into();
                }
                let w: Vec<u16> = if dt == DataType::DoubleMsb {
                    r[start..start + 4].into()
                } else {
                    r[start..start + 4].iter().cloned().rev().collect()
                };
                let bits = ((w[0] as u64) << 48)
                    | ((w[1] as u64) << 32)
                    | ((w[2] as u64) << 16)
                    | w[3] as u64;
                format!("{:.6}", f64::from_bits(bits))
            }
        }
    }

    fn type_input(&mut self, c: char) {
        if !c.is_ascii_digit() {
            return;
        }
        match self.focus {
            Focus::Address => {
                if self.address_suffix.len() < 4 {
                    self.address_suffix.push(c);
                }
            }
            Focus::Length => self.length.push(c),
            Focus::Slave => self.slave.push(c),
            Focus::Interval => self.interval_ms.push(c),
            _ => {}
        }
    }

    fn backspace(&mut self) {
        match self.focus {
            Focus::Address => {
                self.address_suffix.pop();
                if self.address_suffix.is_empty() {
                    self.address_suffix = "0".into();
                }
            }
            Focus::Length => {
                self.length.pop();
            }
            Focus::Slave => {
                self.slave.pop();
            }
            Focus::Interval => {
                self.interval_ms.pop();
            }
            _ => {}
        }
    }
}

// ── UI ────────────────────────────────────────────────────────────────────────

fn draw(f: &mut ratatui::Frame, app: &App) {
    let root = Layout::vertical([
        Constraint::Length(3),
        Constraint::Length(3),
        Constraint::Length(3),
        Constraint::Min(0),
        Constraint::Length(1),
    ])
    .split(f.area());

    draw_conn_bar(f, app, root[0]);
    draw_settings(f, app, root[1]);
    draw_dtype_bar(f, app, root[2]);
    draw_panels(f, app, root[3]);
    draw_help(f, root[4]);
}

fn focused_block(title: &str, focused: bool) -> Block {
    Block::bordered()
        .title(title.to_owned())
        .border_type(BorderType::Rounded)
        .border_style(if focused {
            Style::new().cyan()
        } else {
            Style::new().dark_gray()
        })
}

fn draw_conn_bar(f: &mut ratatui::Frame, app: &App, area: Rect) {
    let status_color = if app.status.starts_with("Error") {
        Color::Red
    } else if app.status.starts_with("Connecting") {
        Color::Yellow
    } else {
        Color::Green
    };
    f.render_widget(
        Paragraph::new(app.conn.as_str())
            .style(Style::new().white())
            .block(
                Block::bordered()
                    .title(Line::from(vec![
                        Span::raw(" ModbusPoll  ─  "),
                        Span::styled(&app.status, Style::new().fg(status_color).bold()),
                        Span::raw(" "),
                    ]))
                    .border_type(BorderType::Rounded)
                    .border_style(Style::new().cyan()),
            ),
        area,
    );
}

fn draw_settings(f: &mut ratatui::Frame, app: &App, area: Rect) {
    let cols = Layout::horizontal([
        Constraint::Percentage(24),
        Constraint::Percentage(19),
        Constraint::Percentage(14),
        Constraint::Percentage(14),
        Constraint::Percentage(19),
        Constraint::Min(0),
    ])
    .split(area);

    // Register type
    let rt = RegType::ALL[app.reg_type_idx];
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::raw("◄ "),
            Span::styled(rt.label(), Style::new().white().bold()),
            Span::raw(" ►"),
        ]))
        .block(focused_block(" Type ", app.focus == Focus::RegType)),
        cols[0],
    );

    // Address
    let addr_focused = app.focus == Focus::Address;
    let display_addr = app.display_address();
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(
                format!("{}", display_addr),
                if addr_focused {
                    Style::new().black().on_cyan().bold()
                } else {
                    Style::new().white()
                },
            ),
            Span::styled(
                format!("  [{}]", app.address_suffix),
                if addr_focused {
                    Style::new().black().on_cyan()
                } else {
                    Style::new().dark_gray()
                },
            ),
        ]))
        .block(focused_block(" Address (suffix) ", addr_focused)),
        cols[1],
    );

    // Length, Slave
    let fields: &[(&str, &str, Focus)] = &[
        (" Length ", &app.length, Focus::Length),
        (" Slave ", &app.slave, Focus::Slave),
    ];
    for (i, (title, val, foc)) in fields.iter().enumerate() {
        let focused = app.focus == *foc;
        f.render_widget(
            Paragraph::new(*val)
                .style(if focused {
                    Style::new().black().on_cyan().bold()
                } else {
                    Style::new().white()
                })
                .block(focused_block(title, focused)),
            cols[i + 2],
        );
    }

    // Interval
    let int_focused = app.focus == Focus::Interval;
    f.render_widget(
        Paragraph::new(format!("{} ms", app.interval_ms))
            .style(if int_focused {
                Style::new().black().on_cyan().bold()
            } else {
                Style::new().white()
            })
            .block(focused_block(" Interval ", int_focused)),
        cols[4],
    );
}

fn draw_dtype_bar(f: &mut ratatui::Frame, app: &App, area: Rect) {
    let spans: Vec<Span> = DataType::ALL
        .iter()
        .enumerate()
        .flat_map(|(i, dt)| {
            let s = if i == app.dt_idx {
                Span::styled(
                    format!(" {} ", dt.label()),
                    Style::new().black().on_cyan().bold(),
                )
            } else {
                Span::styled(format!(" {} ", dt.label()), Style::new().dark_gray())
            };
            vec![s, Span::raw(" ")]
        })
        .collect();

    f.render_widget(
        Paragraph::new(Line::from(spans)).block(
            Block::bordered()
                .title(" Data Type  ◄ ► ")
                .border_type(BorderType::Rounded)
                .border_style(Style::new().yellow()),
        ),
        area,
    );
}

fn draw_panels(f: &mut ratatui::Frame, app: &App, area: Rect) {
    let [left, right] =
        Layout::horizontal([Constraint::Percentage(58), Constraint::Percentage(42)]).areas(area);

    let len = app.length.parse::<usize>().unwrap_or(10).min(64);
    let base = app.display_address();
    let rpv = DataType::ALL[app.dt_idx].regs_per_value();
    let dt_label = DataType::ALL[app.dt_idx].label();
    let rt_label = RegType::ALL[app.reg_type_idx].label();

    let header = Row::new(vec![
        "Address".to_string(),
        "Raw Hex".to_string(),
        format!("Value ({})", dt_label),
    ])
    .style(Style::new().cyan().bold())
    .bottom_margin(1);

    let mut rows: Vec<Row> = Vec::new();
    let mut i = 0usize;
    while i < len {
        let addr = base + i as u32;
        let hex: String = (0..rpv.min(len - i))
            .map(|j| format!("{:04X}", app.registers.get(i + j).unwrap_or(&0)))
            .collect::<Vec<_>>()
            .join(" ");
        let val = app.format_value(i);
        let style = if i % 2 == 0 {
            Style::new().white()
        } else {
            Style::new().gray()
        };
        rows.push(Row::new(vec![format!("{}", addr), hex, val]).style(style));
        i += rpv;
    }

    f.render_widget(
        Table::new(
            rows,
            [
                Constraint::Length(8),
                Constraint::Length(20),
                Constraint::Min(0),
            ],
        )
        .header(header)
        .block(
            Block::bordered()
                .title(format!(" Registers  ─  {} ", rt_label))
                .border_type(BorderType::Rounded)
                .border_style(Style::new().green()),
        ),
        left,
    );

    // ── Traffic panel ────────────────────────────────────────────────────────
    // Label the last two bytes as CRC (RTU) or show MBAP breakdown (TCP)
    let items: Vec<ListItem> = app
        .traffic
        .iter()
        .map(|fr| {
            let (color, dir) = if fr.tx {
                (Color::Yellow, "TX ▶")
            } else {
                (Color::Green, "RX ◀")
            };

            let bytes = &fr.bytes;
            let hex_parts: Vec<Span> = if app.is_rtu {
                // RTU: [SlaveID] [PDU bytes...] [CRC_LO CRC_HI]
                annotate_rtu_frame(bytes, color)
            } else {
                // TCP: [TxnID(2)] [Proto(2)] [Len(2)] [UnitID] [PDU...]
                annotate_tcp_frame(bytes, color)
            };

            let mut spans = vec![
                Span::styled(format!("#{:<4} ", fr.seq), Style::new().dark_gray()),
                Span::styled(format!("{} ", dir), Style::new().fg(color).bold()),
            ];
            spans.extend(hex_parts);

            ListItem::new(Line::from(spans))
        })
        .collect();

    let total = items.len();
    let mut state = ListState::default();
    if total > 0 {
        state.select(Some(app.traffic_scroll.min(total - 1)));
    }

    let proto_label = if app.is_rtu { "RTU" } else { "TCP" };

    f.render_stateful_widget(
        List::new(items).block(
            Block::bordered()
                .title(format!(" Traffic [{}]  ─  {} frames ", proto_label, total))
                .border_type(BorderType::Rounded)
                .border_style(Style::new().magenta()),
        ),
        right,
        &mut state,
    );
}

fn annotate_rtu_frame(bytes: &[u8], color: Color) -> Vec<Span<'static>> {
    let hex = bytes
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ");
    vec![Span::styled(hex, Style::new().fg(color))]
}

/// Plain hex dump for TCP: all bytes space-separated.
fn annotate_tcp_frame(bytes: &[u8], color: Color) -> Vec<Span<'static>> {
    let hex = bytes
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ");
    vec![Span::styled(hex, Style::new().fg(color))]
}

fn draw_help(f: &mut ratatui::Frame, area: Rect) {
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(" Tab ", Style::new().black().on_dark_gray()),
            Span::raw(" field  "),
            Span::styled(" ◄ ► ", Style::new().black().on_dark_gray()),
            Span::raw(" cycle  "),
            Span::styled(" ↑ ↓ ", Style::new().black().on_dark_gray()),
            Span::raw(" scroll  "),
            Span::styled(" Space ", Style::new().black().on_dark_gray()),
            Span::raw(" poll now  "),
            Span::styled(" q ", Style::new().black().on_red()),
            Span::raw(" quit  "),
            Span::styled(" RTU: SlaveID|PDU|CRC✓ ", Style::new().dark_gray()),
            Span::styled(
                " TCP: TxnID|Proto|Len|UnitID|PDU ",
                Style::new().dark_gray(),
            ),
        ]))
        .alignment(Alignment::Center),
        area,
    );
}

// ── Event handling ────────────────────────────────────────────────────────────

fn handle_key(app: &mut App, key: crossterm::event::KeyEvent) -> bool {
    if key.kind != KeyEventKind::Press {
        return false;
    }
    match key.code {
        KeyCode::Char('q') | KeyCode::Char('Q') => return true,
        KeyCode::Tab => app.focus = app.focus.next(),

        KeyCode::Left => match app.focus {
            Focus::RegType => {
                let n = RegType::ALL.len();
                app.reg_type_idx = (app.reg_type_idx + n - 1) % n;
            }
            _ => {
                let n = DataType::ALL.len();
                app.dt_idx = (app.dt_idx + n - 1) % n;
            }
        },
        KeyCode::Right => match app.focus {
            Focus::RegType => app.reg_type_idx = (app.reg_type_idx + 1) % RegType::ALL.len(),
            _ => app.dt_idx = (app.dt_idx + 1) % DataType::ALL.len(),
        },

        KeyCode::Up => {
            if app.traffic_scroll > 0 {
                app.traffic_scroll -= 1;
            }
        }
        KeyCode::Down => {
            app.traffic_scroll = (app.traffic_scroll + 1).min(app.traffic.len().saturating_sub(1));
        }

        KeyCode::Char(' ') => app.send_poll(),
        KeyCode::Backspace => app.backspace(),
        KeyCode::Char(c) => app.type_input(c),
        _ => {}
    }
    false
}

// ── Main ──────────────────────────────────────────────────────────────────────

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let conn = ConnectionInfo::from_mode(&cli.mode);

    let is_rtu = matches!(cli.mode, Mode::Rtu { .. });

    // Channels: bounded so we don't queue stale polls
    let (cmd_tx, cmd_rx) = mpsc::sync_channel::<PollCommand>(1);
    let (res_tx, res_rx) = mpsc::channel::<PollResult>();

    // Spawn tokio runtime + modbus worker in background thread
    let mode = cli.mode;
    std::thread::spawn(move || {
        let rt = Runtime::new().expect("tokio runtime");
        rt.block_on(async move {
            match mode {
                Mode::Tcpip { ip, port } => {
                    run_modbus_tcp(ip, port, cmd_rx, res_tx).await;
                }
                Mode::Rtu {
                    comport,
                    baudrate,
                    parity,
                    stopbits,
                    databits,
                } => {
                    run_modbus_rtu(
                        comport, baudrate, parity, stopbits, databits, cmd_rx, res_tx,
                    )
                    .await;
                }
            }
        });
    });

    // TUI setup
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let mut terminal = Terminal::new(CrosstermBackend::new(stdout))?;

    let mut app = App::new(conn, is_rtu, cmd_tx.clone(), res_rx);

    loop {
        app.drain_results();
        terminal.draw(|f| draw(f, &app))?;

        if app.last_poll.elapsed() >= app.interval() {
            app.send_poll();
            app.last_poll = Instant::now();
        }

        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(k) = event::read()? {
                if handle_key(&mut app, k) {
                    break;
                }
            }
        }
    }

    let _ = cmd_tx.try_send(PollCommand::Shutdown);
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    Ok(())
}
