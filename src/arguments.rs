use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "ModbusPoll")]
#[command(about = "Modbus polling tool")]
pub struct Cli {
    #[command(subcommand)]
    pub mode: Mode,
}

#[derive(Subcommand)]
pub enum Mode {
    /// Serial RTU mode
    Rtu {
        #[arg(short, long)]
        comport: String,
        #[arg(short, long, default_value_t = 9600)]
        baudrate: u32,
        #[arg(short, long, default_value = "None")]
        parity: String,
        #[arg(short, long, default_value_t = 1)]
        stopbits: u8,
        #[arg(short, long, default_value_t = 8)]
        databits: u8,
    },
    /// TCP/IP mode
    Tcpip {
        #[arg(short, long)]
        ip: String,
        #[arg(short, long, default_value_t = 502)]
        port: u16,
    },
}

/// Flattened connection info for the TUI title bar
pub struct ConnectionInfo {
    pub mode_label: String,
}

impl ConnectionInfo {
    pub fn from_mode(mode: &Mode) -> Self {
        let mode_label = match mode {
            Mode::Rtu {
                comport,
                baudrate,
                parity,
                stopbits,
                databits,
            } => format!(
                "RTU  {}  {} baud  {}  {} stop  {} data",
                comport, baudrate, parity, stopbits, databits
            ),
            Mode::Tcpip { ip, port } => format!("TCP  {}:{}", ip, port),
        };
        Self { mode_label }
    }
}
