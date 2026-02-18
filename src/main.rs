mod arguments;

use arguments::{Cli, ConnectionInfo};
use clap::Parser;
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use ratatui::{prelude::*, widgets::*};
use std::{
    io,
    time::{Duration, Instant},
};

// ── Data Types ────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq)]
enum DataType {
    Int16,
    Int32Msb,
    Int32Lsb,
    Int64Msb,
    Int64Lsb,
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
            DataType::FloatMsb => "Float MSB",
            DataType::FloatLsb => "Float LSB",
            DataType::DoubleMsb => "Double MSB",
            DataType::DoubleLsb => "Double LSB",
        }
    }

    fn regs_per_value(self) -> usize {
        match self {
            DataType::Int16 => 1,
            DataType::Int32Msb | DataType::Int32Lsb | DataType::FloatMsb | DataType::FloatLsb => 2,
            _ => 4,
        }
    }
}

// ── Register Types ────────────────────────────────────────────────────────────

#[derive(Clone, Copy)]
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
    reg_type_idx: usize,
    address: String,
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
}

impl App {
    fn new(conn: ConnectionInfo) -> Self {
        Self {
            conn: conn.mode_label,
            reg_type_idx: 2,
            address: "40001".into(),
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
            status: "Running".into(),
        }
    }

    fn interval(&self) -> Duration {
        Duration::from_millis(self.interval_ms.parse().unwrap_or(1000))
    }

    fn poll(&mut self) {
        self.poll_seq += 1;
        let len = self.length.parse::<usize>().unwrap_or(10).min(64);
        for i in 0..len {
            self.registers[i] = ((self.poll_seq * 17 + i as u64 * 11) % 65535) as u16;
        }
        let slave = self.slave.parse::<u8>().unwrap_or(1);
        let addr = self.address.parse::<u16>().unwrap_or(0) % 10000;
        let tx = vec![slave, 0x03, (addr >> 8) as u8, addr as u8, 0x00, len as u8];
        let mut rx = vec![slave, 0x03, (len * 2) as u8];
        for i in 0..len {
            rx.push((self.registers[i] >> 8) as u8);
            rx.push(self.registers[i] as u8);
        }
        self.push_frame(true, tx);
        self.push_frame(false, rx);
        self.status = format!("Poll #{}", self.poll_seq);
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
            Focus::Address => self.address.push(c),
            Focus::Length => self.length.push(c),
            Focus::Slave => self.slave.push(c),
            Focus::Interval => self.interval_ms.push(c),
            _ => {}
        }
    }

    fn backspace(&mut self) {
        match self.focus {
            Focus::Address => {
                self.address.pop();
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
    f.render_widget(
        Paragraph::new(app.conn.as_str())
            .style(Style::new().white())
            .block(
                Block::bordered()
                    .title(format!(" ModbusPoll  ─  {} ", app.status))
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

    let fields: &[(&str, &str, Focus)] = &[
        (" Address ", &app.address, Focus::Address),
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
            cols[i + 1],
        );
    }

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

    // ── Register table ──
    let len = app.length.parse::<usize>().unwrap_or(10).min(64);
    let base = app.address.parse::<u32>().unwrap_or(40001);
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

    // ── Traffic panel ──
    let items: Vec<ListItem> = app
        .traffic
        .iter()
        .map(|fr| {
            let (color, dir) = if fr.tx {
                (Color::Yellow, "TX ▶")
            } else {
                (Color::Green, "RX ◀")
            };
            let hex: String = fr
                .bytes
                .iter()
                .map(|b| format!("{:02X}", b))
                .collect::<Vec<_>>()
                .join(" ");
            ListItem::new(Line::from(vec![
                Span::styled(format!("#{:<4} ", fr.seq), Style::new().dark_gray()),
                Span::styled(format!("{} ", dir), Style::new().fg(color).bold()),
                Span::styled(hex, Style::new().fg(color)),
            ]))
        })
        .collect();

    let total = items.len();
    let mut state = ListState::default();
    if total > 0 {
        state.select(Some(app.traffic_scroll.min(total - 1)));
    }

    f.render_stateful_widget(
        List::new(items).block(
            Block::bordered()
                .title(format!(" Traffic  ─  {} frames ", total))
                .border_type(BorderType::Rounded)
                .border_style(Style::new().magenta()),
        ),
        right,
        &mut state,
    );
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
            Span::raw(" poll  "),
            Span::styled(" q ", Style::new().black().on_red()),
            Span::raw(" quit"),
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

        KeyCode::Char(' ') => app.poll(),
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

    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let mut terminal = Terminal::new(CrosstermBackend::new(stdout))?;

    let mut app = App::new(conn);

    loop {
        terminal.draw(|f| draw(f, &app))?;

        if app.last_poll.elapsed() >= app.interval() {
            app.poll();
            app.last_poll = Instant::now();
        }

        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(k) = event::read()? {
                if handle_key(&mut app, k) {
                    break;
                }
            }
        }
    }

    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    Ok(())
}
