//! Xbox Wireless Controller HID input-report decoding.
//!
//! Layout follows the Xbox Series controller (model 1914) BLE HID report
//! descriptor, which recent firmware also uses on the Xbox One S (1708) — all
//! current controllers share this unified descriptor over BLE. The 16-byte
//! report body is:
//!
//! | bytes | field                                    |
//! |-------|------------------------------------------|
//! | 0..7  | LX, LY, RX, RY — `u16` LE, 0..65535, centre 32768 |
//! | 8..11 | LT, RT — 10-bit (0..1023), low bits of a `u16` |
//! | 12    | D-pad hat: low nibble, 1..8 clockwise from up, 0 = neutral |
//! | 13    | buttons 1..8                             |
//! | 14    | buttons 9..15                            |
//! | 15    | Record/Share (bit 1)                     |
//!
//! Button *names* aren't in the descriptor (it only declares Button1..15); the
//! mapping below follows the common SDL/xpadneo convention, with the
//! characteristic gaps at buttons 3, 6, 9, 10.

/// Number of bytes in the report body (excluding any leading report-id byte).
const BODY_LEN: usize = 16;

/// D-pad (hat switch) direction.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Dpad {
    None,
    Up,
    UpRight,
    Right,
    DownRight,
    Down,
    DownLeft,
    Left,
    UpLeft,
}

impl Dpad {
    fn from_hat(v: u8) -> Self {
        match v {
            1 => Dpad::Up,
            2 => Dpad::UpRight,
            3 => Dpad::Right,
            4 => Dpad::DownRight,
            5 => Dpad::Down,
            6 => Dpad::DownLeft,
            7 => Dpad::Left,
            8 => Dpad::UpLeft,
            _ => Dpad::None,
        }
    }

    /// Short label for display, e.g. "NE", or "-" when neutral.
    pub fn label(self) -> &'static str {
        match self {
            Dpad::None => "-",
            Dpad::Up => "N",
            Dpad::UpRight => "NE",
            Dpad::Right => "E",
            Dpad::DownRight => "SE",
            Dpad::Down => "S",
            Dpad::DownLeft => "SW",
            Dpad::Left => "W",
            Dpad::UpLeft => "NW",
        }
    }
}

/// A decoded controller state snapshot.
#[derive(Clone, Copy, Debug)]
pub struct XboxReport {
    /// Stick axes, 0..65535, centre 32768. Y grows downward.
    pub lx: u16,
    pub ly: u16,
    pub rx: u16,
    pub ry: u16,
    /// Triggers, 0..1023.
    pub lt: u16,
    pub rt: u16,
    pub dpad: Dpad,
    pub a: bool,
    pub b: bool,
    pub x: bool,
    pub y: bool,
    pub lb: bool,
    pub rb: bool,
    pub view: bool,
    pub menu: bool,
    pub xbox: bool,
    pub ls: bool,
    pub rs: bool,
    pub share: bool,
}

impl XboxReport {
    /// Decode a HID input report. Tolerates an optional leading report-id byte
    /// (the report body is taken as the last [`BODY_LEN`] bytes). Returns `None`
    /// if the slice is too short to be a gamepad report.
    pub fn parse(data: &[u8]) -> Option<Self> {
        let start = data.len().checked_sub(BODY_LEN)?;
        let d = &data[start..];
        let u16le = |i: usize| u16::from_le_bytes([d[i], d[i + 1]]);

        let lo = d[13]; // buttons 1..8
        let hi = d[14]; // buttons 9..15

        Some(Self {
            lx: u16le(0),
            ly: u16le(2),
            rx: u16le(4),
            ry: u16le(6),
            lt: u16le(8) & 0x03ff,
            rt: u16le(10) & 0x03ff,
            dpad: Dpad::from_hat(d[12] & 0x0f),
            a: lo & 0x01 != 0,    // button 1
            b: lo & 0x02 != 0,    // button 2
            x: lo & 0x08 != 0,    // button 4
            y: lo & 0x10 != 0,    // button 5
            lb: lo & 0x40 != 0,   // button 7
            rb: lo & 0x80 != 0,   // button 8
            view: hi & 0x04 != 0, // button 11
            menu: hi & 0x08 != 0, // button 12
            xbox: hi & 0x10 != 0, // button 13 (guide)
            ls: hi & 0x20 != 0,   // button 14 (left stick click)
            rs: hi & 0x40 != 0,   // button 15 (right stick click)
            share: d[15] & 0x02 != 0,
        })
    }

    /// Left/right stick axes as a signed percentage, -100..100 (centre = 0).
    pub fn lx_pct(&self) -> i32 {
        axis_pct(self.lx)
    }
    pub fn ly_pct(&self) -> i32 {
        axis_pct(self.ly)
    }
    pub fn rx_pct(&self) -> i32 {
        axis_pct(self.rx)
    }
    pub fn ry_pct(&self) -> i32 {
        axis_pct(self.ry)
    }

    /// Triggers as a percentage, 0..100.
    pub fn lt_pct(&self) -> u32 {
        self.lt as u32 * 100 / 1023
    }
    pub fn rt_pct(&self) -> u32 {
        self.rt as u32 * 100 / 1023
    }
}

/// Map a 0..65535 axis (centre 32768) to a signed -100..100 percentage.
fn axis_pct(v: u16) -> i32 {
    (v as i32 - 32768) * 100 / 32768
}
