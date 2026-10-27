// Rust core for opendbc.can.parser.CANParser. Mirrors MessageState and
// CANParser.update/can_valid/bus_timeout from parser.py; the Python wrapper
// owns the vl/vl_all/ts_nanos dicts and this core writes into them.

mod checksum;

use std::collections::{HashMap, VecDeque};

use pyo3::prelude::*;
use pyo3::types::{PyByteArray, PyBytes, PyDict, PyFloat, PyList, PySet, PyString, PyTuple};

const MAX_BAD_COUNTER: u32 = 5;
const CAN_INVALID_CNT: u32 = 5;
const TIMESTAMPS_MAXLEN: usize = 500;
const COUNTER: u32 = 1;

enum Checksum {
  None,
  Native(u32),
  Python { calc: PyObject, sig: PyObject },
}

struct Signal {
  name: Py<PyString>,
  start_bit: u32,
  msb: u32,
  lsb: u32,
  size: u32,
  is_signed: bool,
  is_little_endian: bool,
  factor: f64,
  offset: f64,
  sig_type: u32,
  checksum: Checksum,
}

impl Signal {
  fn get_raw_value(&self, dat: &[u8]) -> u64 {
    let mut ret: u64 = 0;
    let mut i = (self.msb / 8) as isize;
    let mut bits = self.size;
    while i >= 0 && (i as usize) < dat.len() && bits > 0 {
      let iu = i as u32;
      let lsb = if self.lsb / 8 == iu { self.lsb } else { iu * 8 };
      let msb = if self.msb / 8 == iu { self.msb } else { (iu + 1) * 8 - 1 };
      let size = msb - lsb + 1;
      let d = ((dat[i as usize] as u64) >> (lsb - iu * 8)) & ((1u64 << size) - 1);
      ret |= d << (bits - size);
      bits -= size;
      i = if self.is_little_endian { i - 1 } else { i + 1 };
    }
    ret
  }
}

struct MessageState {
  address: u32,
  name: String,
  signals: Vec<Signal>,
  ignore_alive: bool,
  frequency: f64,
  timeout_threshold: f64,
  timestamps: VecDeque<i64>,
  counter: i128,
  counter_fail: u32,
  last_warning_log_nanos: i64,
  tmp_vals: Vec<f64>,

  vl: Py<PyDict>,
  vl_all: Py<PyDict>,
  ts_nanos: Py<PyDict>,
  all_vals: Vec<Py<PyList>>,
  vl_all_linked: bool,
  all_vals_dirty: bool,
}

fn py_hex(v: i128) -> String {
  if v < 0 { format!("-0x{:x}", -v) } else { format!("0x{v:x}") }
}

impl MessageState {
  fn rate_limited_log(&mut self, py: Python<'_>, log: &PyObject, last_update_nanos: i64, msg: &str) -> PyResult<()> {
    if last_update_nanos - self.last_warning_log_nanos >= 1_000_000_000 {
      log.call1(py, (format!("CANParser: {} {} {}", py_hex(self.address as i128), self.name, msg),))?;
      self.last_warning_log_nanos = last_update_nanos;
    }
    Ok(())
  }

  fn update_counter(&mut self, cur_count: i128, cnt_size: u32) -> bool {
    if ((self.counter + 1) & ((1i128 << cnt_size) - 1)) != cur_count {
      self.counter_fail = (self.counter_fail + 1).min(MAX_BAD_COUNTER);
    } else if self.counter_fail > 0 {
      self.counter_fail -= 1;
    }
    self.counter = cur_count;
    self.counter_fail < MAX_BAD_COUNTER
  }

  fn parse(&mut self, py: Python<'_>, log: &PyObject, nanos: i64, dat: &[u8]) -> PyResult<bool> {
    let mut checksum_failed = false;
    let mut counter_failed = false;

    for i in 0..self.signals.len() {
      let sig = &self.signals[i];
      let mut tmp = sig.get_raw_value(dat) as i128;
      if sig.is_signed {
        tmp -= ((tmp >> (sig.size - 1)) & 0x1) * (1i128 << sig.size);
      }

      let expected = match &sig.checksum {
        Checksum::None => None,
        Checksum::Native(t) => Some(checksum::calc(*t, self.address, sig.start_bit, dat)),
        Checksum::Python { calc, sig: sig_obj } => {
          Some(calc.call1(py, (self.address, sig_obj, PyByteArray::new(py, dat)))?.extract::<i64>(py)?)
        }
      };
      let (is_counter, size, factor, offset) = (sig.sig_type == COUNTER, sig.size, sig.factor, sig.offset);

      if let Some(expected) = expected {
        if tmp != expected as i128 {
          checksum_failed = true;
          let msg = format!("checksum failed: received {}, calculated {}", py_hex(tmp), py_hex(expected as i128));
          self.rate_limited_log(py, log, nanos, &msg)?;
        }
      }

      if is_counter && !self.update_counter(tmp, size) {
        counter_failed = true;
      }

      self.tmp_vals[i] = tmp as f64 * factor + offset;
    }

    // must have good counter and checksum to update data
    if checksum_failed || counter_failed {
      return Ok(false);
    }

    if self.timestamps.len() == TIMESTAMPS_MAXLEN {
      self.timestamps.pop_front();
    }
    self.timestamps.push_back(nanos);

    let vl = self.vl.bind(py);
    let ts_nanos = self.ts_nanos.bind(py);
    let ts = nanos.into_pyobject(py)?;
    for (i, sig) in self.signals.iter().enumerate() {
      let v = PyFloat::new(py, self.tmp_vals[i]);
      let name = sig.name.bind(py);
      vl.set_item(name, &v)?;
      self.all_vals[i].bind(py).append(&v)?;
      ts_nanos.set_item(name, &ts)?;
    }
    self.all_vals_dirty = true;
    if !self.vl_all_linked {
      let vl_all = self.vl_all.bind(py);
      for (sig, vals) in self.signals.iter().zip(&self.all_vals) {
        vl_all.set_item(sig.name.bind(py), vals.bind(py))?;
      }
      self.vl_all_linked = true;
    }

    if self.frequency < 1e-5 && self.timestamps.len() >= 3 {
      let dt = (self.timestamps[self.timestamps.len() - 1] - self.timestamps[0]) as f64 * 1e-9;
      if (dt > 1.0 || self.timestamps.len() >= TIMESTAMPS_MAXLEN) && dt != 0.0 {
        self.frequency = (self.timestamps.len() as f64 / dt).min(100.0);
        self.timeout_threshold = (1_000_000_000.0 / self.frequency) * 10.0;
      }
    }
    Ok(true)
  }

  fn valid(&self, current_nanos: i64) -> bool {
    if self.ignore_alive {
      return true;
    }
    match self.timestamps.back() {
      None => false,
      Some(&last) => ((current_nanos - last) as f64) <= self.timeout_threshold,
    }
  }
}

#[pyclass(module = "opendbc.can._parser_rs")]
struct ParserCore {
  bus: i64,
  log: PyObject,
  states: Vec<MessageState>,
  index: HashMap<u32, usize>,
  #[pyo3(get, set)]
  can_invalid_cnt: u32,
  #[pyo3(get, set)]
  last_nonempty_nanos: i64,
  #[pyo3(get, set)]
  last_update_nanos: i64,
}

fn item<'py>(obj: &Bound<'py, PyAny>, i: usize) -> PyResult<Bound<'py, PyAny>> {
  match obj.downcast::<PyTuple>() {
    Ok(t) => t.get_item(i),
    Err(_) => obj.get_item(i),
  }
}

#[pymethods]
impl ParserCore {
  #[new]
  fn new(bus: i64, log: PyObject) -> Self {
    ParserCore {
      bus,
      log,
      states: Vec::new(),
      index: HashMap::new(),
      can_invalid_cnt: CAN_INVALID_CNT,
      last_nonempty_nanos: 0,
      last_update_nanos: 0,
    }
  }

  /// signals: list of opendbc.can.dbc.Signal
  #[allow(clippy::too_many_arguments)]
  fn add_message(&mut self, py: Python<'_>, address: u32, name: String, signals: &Bound<'_, PyList>, ignore_alive: bool,
                 frequency: f64, timeout_threshold: f64, vl: Py<PyDict>, vl_all: Py<PyDict>, ts_nanos: Py<PyDict>) -> PyResult<()> {
    let mut sigs = Vec::with_capacity(signals.len());
    for s in signals.iter() {
      let sig_type: u32 = s.getattr("type")?.extract()?;
      let calc = s.getattr("calc_checksum")?;
      let checksum = if calc.is_none() {
        Checksum::None
      } else if checksum::is_native(sig_type) {
        Checksum::Native(sig_type)
      } else {
        Checksum::Python { calc: calc.unbind(), sig: s.clone().unbind() }
      };
      let name: String = s.getattr("name")?.extract()?;
      sigs.push(Signal {
        name: PyString::intern(py, &name).unbind(),
        start_bit: s.getattr("start_bit")?.extract()?,
        msb: s.getattr("msb")?.extract()?,
        lsb: s.getattr("lsb")?.extract()?,
        size: s.getattr("size")?.extract()?,
        is_signed: s.getattr("is_signed")?.extract()?,
        is_little_endian: s.getattr("is_little_endian")?.extract()?,
        factor: s.getattr("factor")?.extract()?,
        offset: s.getattr("offset")?.extract()?,
        sig_type,
        checksum,
      });
    }

    let n = sigs.len();
    self.index.insert(address, self.states.len());
    self.states.push(MessageState {
      address,
      name,
      signals: sigs,
      ignore_alive,
      frequency,
      timeout_threshold,
      timestamps: VecDeque::with_capacity(TIMESTAMPS_MAXLEN),
      counter: 0,
      counter_fail: 0,
      last_warning_log_nanos: 0,
      tmp_vals: vec![0.0; n],
      vl,
      vl_all,
      ts_nanos,
      all_vals: (0..n).map(|_| PyList::empty(py).unbind()).collect(),
      vl_all_linked: false,
      all_vals_dirty: false,
    });
    Ok(())
  }

  fn update<'py>(&mut self, py: Python<'py>, strings: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PySet>> {
    for st in self.states.iter_mut().filter(|st| st.all_vals_dirty) {
      for vals in &st.all_vals {
        let vals = vals.bind(py);
        vals.as_sequence().del_slice(0, vals.len())?;
      }
      st.all_vals_dirty = false;
    }

    let updated = PySet::empty(py)?;
    let entries = if strings.len()? > 0 && !{
      let first = item(strings, 0)?;
      first.is_instance_of::<PyList>() || first.is_instance_of::<PyTuple>()
    } {
      PyList::new(py, [strings])?.into_any()
    } else {
      strings.clone()
    };

    for entry in entries.try_iter()? {
      let entry = entry?;
      let (t, frames) = (item(&entry, 0)?, item(&entry, 1)?);
      // nanos are ints from can_capnp_to_list, tests also pass floats
      let t: i64 = match t.extract() {
        Ok(t) => t,
        Err(_) => t.extract::<f64>()? as i64,
      };
      let mut bus_empty = true;
      for frame in frames.try_iter()? {
        let frame = frame?;
        let src: i64 = item(&frame, 2)?.extract()?;
        if src != self.bus {
          continue;
        }
        bus_empty = false;
        let address: u32 = item(&frame, 0)?.extract()?;
        let Some(&idx) = self.index.get(&address) else { continue };

        let dat_obj = item(&frame, 1)?;
        let owned;
        let dat: &[u8] = if let Ok(b) = dat_obj.downcast::<PyBytes>() {
          b.as_bytes()
        } else {
          owned = dat_obj.extract::<Vec<u8>>()?;
          &owned
        };
        if dat.len() > 64 {
          continue;
        }
        if self.states[idx].parse(py, &self.log, t, dat)? {
          updated.add(address)?;
        }
      }

      if !bus_empty {
        self.last_nonempty_nanos = t;
      }
      self.last_update_nanos = t;
    }
    Ok(updated)
  }

  fn bus_timeout(&self) -> bool {
    let ignore_alive = self.states.iter().all(|s| s.ignore_alive);
    let mut threshold = 500.0 * 1_000_000.0;
    for st in &self.states {
      if st.timeout_threshold > 0.0 {
        threshold = f64::min(threshold, st.timeout_threshold);
      }
    }
    ((self.last_update_nanos - self.last_nonempty_nanos) as f64 > threshold) && !ignore_alive
  }

  fn can_valid(&mut self, py: Python<'_>) -> PyResult<bool> {
    let mut valid = true;
    let mut counters_valid = true;
    let last_update_nanos = self.last_update_nanos;
    for st in self.states.iter_mut() {
      if st.counter_fail >= MAX_BAD_COUNTER {
        counters_valid = false;
        let msg = format!("counter invalid, state.counter_fail={} MAX_BAD_COUNTER={}", st.counter_fail, MAX_BAD_COUNTER);
        st.rate_limited_log(py, &self.log, last_update_nanos, &msg)?;
      }
      if !st.valid(last_update_nanos) {
        valid = false;
        st.rate_limited_log(py, &self.log, last_update_nanos, "not valid (timeout or missing)")?;
      }
    }

    // TODO: probably only want to increment this once per update() call
    self.can_invalid_cnt = if valid { 0 } else { (self.can_invalid_cnt + 1).min(CAN_INVALID_CNT) };
    Ok(self.can_invalid_cnt < CAN_INVALID_CNT && counters_valid)
  }
}

#[pymodule]
fn _parser_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
  m.add_class::<ParserCore>()
}
