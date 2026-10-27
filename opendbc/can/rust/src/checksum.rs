// Native ports of the checksum functions in opendbc/car/*/*can.py.
// Types not handled here fall back to calling the Python function.

const fn gen_crc8_table(poly: u8) -> [u8; 256] {
  let mut table = [0u8; 256];
  let mut i = 0;
  while i < 256 {
    let mut crc = i as u8;
    let mut j = 0;
    while j < 8 {
      crc = if crc & 0x80 != 0 { (crc << 1) ^ poly } else { crc << 1 };
      j += 1;
    }
    table[i] = crc;
    i += 1;
  }
  table
}

const fn gen_crc16_table(poly: u16) -> [u16; 256] {
  let mut table = [0u16; 256];
  let mut i = 0;
  while i < 256 {
    let mut crc = (i as u16) << 8;
    let mut j = 0;
    while j < 8 {
      crc = if crc & 0x8000 != 0 { (crc << 1) ^ poly } else { crc << 1 };
      j += 1;
    }
    table[i] = crc;
    i += 1;
  }
  table
}

static CRC8J1850: [u8; 256] = gen_crc8_table(0x1D);
static CRC8BODY: [u8; 256] = gen_crc8_table(0xD5);
static CRC16_XMODEM: [u16; 256] = gen_crc16_table(0x1021);

// SignalType values from opendbc/can/dbc.py
const HONDA: u32 = 2;
const TOYOTA: u32 = 3;
const BODY: u32 = 4;
const XOR: u32 = 6;
const SUBARU: u32 = 7;
const CHRYSLER: u32 = 8;
const HKG_CAN_FD: u32 = 9;
const FCA_GIORGIO: u32 = 10;
const TESLA: u32 = 11;
const PSA: u32 = 12;
const BYD: u32 = 14;
const RIVIAN: u32 = 15;
const MG: u32 = 17;

pub fn is_native(sig_type: u32) -> bool {
  matches!(sig_type, HONDA | TOYOTA | BODY | XOR | SUBARU | CHRYSLER | HKG_CAN_FD | FCA_GIORGIO | TESLA | PSA | BYD | RIVIAN | MG)
}

fn byte(d: &[u8], i: usize) -> u32 {
  d.get(i).copied().unwrap_or(0) as u32
}

fn all_but_last(d: &[u8]) -> &[u8] {
  &d[..d.len().saturating_sub(1)]
}

fn crc8(table: &[u8; 256], init: u8, data: &[u8]) -> u8 {
  data.iter().fold(init, |crc, &b| table[(crc ^ b) as usize])
}

pub fn calc(sig_type: u32, address: u32, start_bit: u32, d: &[u8]) -> i64 {
  let checksum_byte = (start_bit / 8) as usize;
  match sig_type {
    HONDA => {
      let mut s: i64 = 0;
      let mut addr = address;
      while addr != 0 {
        s += (addr & 0xF) as i64;
        addr >>= 4;
      }
      for (i, &b) in d.iter().enumerate() {
        let x = if i == d.len() - 1 { b >> 4 } else { b };
        s += ((x & 0xF) + (x >> 4)) as i64;
      }
      s = 8 - s;
      if address > 0x7FF {
        s += 3;
      }
      s & 0xF
    }
    TOYOTA => {
      let mut s = d.len() as u32;
      let mut addr = address;
      while addr != 0 {
        s += addr & 0xFF;
        addr >>= 8;
      }
      s += all_but_last(d).iter().map(|&b| b as u32).sum::<u32>();
      (s & 0xFF) as i64
    }
    BODY => {
      let n = d.len().saturating_sub(1);
      d[..n].iter().rev().fold(0xFFu8, |crc, &b| CRC8BODY[(crc ^ b) as usize]) as i64
    }
    XOR => {
      d.iter().enumerate().filter(|&(i, _)| i != checksum_byte).fold(0u8, |c, (_, &b)| c ^ b) as i64
    }
    SUBARU => {
      let mut s = 0u32;
      let mut addr = address;
      while addr != 0 {
        s += addr & 0xFF;
        addr >>= 8;
      }
      s += d.iter().skip(1).map(|&b| b as u32).sum::<u32>();
      (s & 0xFF) as i64
    }
    CHRYSLER => (crc8(&CRC8J1850, 0xFF, all_but_last(d)) ^ 0xFF) as i64,
    FCA_GIORGIO => {
      let crc = crc8(&CRC8J1850, 0, all_but_last(d));
      let xor = match address {
        0xDE => 0x10,
        0x106 => 0xF6,
        0x122 => 0xF1,
        _ => 0x0A,
      };
      (crc ^ xor) as i64
    }
    HKG_CAN_FD => {
      let step = |crc: u16, b: u8| (crc << 8) ^ CRC16_XMODEM[((crc >> 8) as u8 ^ b) as usize];
      let mut crc = d.iter().skip(2).fold(0u16, |crc, &b| step(crc, b));
      crc = step(crc, (address & 0xFF) as u8);
      crc = step(crc, ((address >> 8) & 0xFF) as u8);
      crc ^= match d.len() {
        8 => 0x5F29,
        16 => 0x041D,
        24 => 0x819D,
        32 => 0x9F5B,
        _ => 0,
      };
      crc as i64
    }
    TESLA => {
      let mut s = (address & 0xFF) + ((address >> 8) & 0xFF);
      s += d.iter().enumerate().filter(|&(i, _)| i != checksum_byte).map(|(_, &b)| b as u32).sum::<u32>();
      (s & 0xFF) as i64
    }
    PSA => {
      let chk_ini: i64 = match address {
        0x452 => 0x4,
        0x38D => 0x7,
        0x42D => 0xC,
        _ => 0xB,
      };
      let mask = if start_bit % 8 >= 4 { 0x0F } else { 0xF0 };
      let s: i64 = d.iter().enumerate()
        .map(|(i, &b)| if i == checksum_byte { b & mask } else { b })
        .map(|b| ((b >> 4) + (b & 0xF)) as i64)
        .sum();
      (chk_ini - s) & 0xF
    }
    BYD => (!d.iter().take(7).map(|&b| b as u32).sum::<u32>() & 0xFF) as i64,
    RIVIAN => {
      let addr = [(address & 0xFF) as u8, ((address >> 8) & 0xFF) as u8];
      let crc = crc8(&CRC8J1850, 0, &addr);
      crc8(&CRC8J1850, crc, d.get(1..).unwrap_or(&[])) as i64
    }
    MG => {
      if address == 0x1EC {
        let torque = ((byte(d, 0) & 0x7) << 8) | byte(d, 1);
        let valid = (byte(d, 0) >> 3) & 1;
        let status = (byte(d, 6) >> 4) & 0x7;
        let counter = byte(d, 0) >> 4;
        (-(((torque << 1) + (status << 12) + valid + counter) as i64)) & 0x7FFF
      } else {
        (crc8(&CRC8J1850, 0xFF, all_but_last(d)) ^ 0xFF) as i64
      }
    }
    _ => unreachable!("checksum type {sig_type} is not native"),
  }
}
