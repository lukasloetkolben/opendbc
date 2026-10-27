// CAN frames decoded from serialized cereal Events. ParserCore.update reads
// these directly; for everything else CanFrames acts like the list
// [(nanos, [(address, dat, src), ...]), ...] that can_capnp_to_list returns.

use pyo3::exceptions::{PyIndexError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyByteArray, PyBytes, PyIterator, PyList, PyTuple};

use crate::capnp::Message;

pub struct Entry {
  pub nanos: i64,
  pub frames: std::ops::Range<usize>,
}

pub struct Frame {
  pub address: u32,
  pub src: u8,
  pub dat: std::ops::Range<usize>,
}

#[pyclass(frozen, sequence, module = "opendbc.can._parser_rs")]
pub struct CanFrames {
  pub entries: Vec<Entry>,
  pub frames: Vec<Frame>,
  pub data: Vec<u8>,
}

impl CanFrames {
  fn entry<'py>(&self, py: Python<'py>, e: &Entry) -> PyResult<Bound<'py, PyTuple>> {
    let frames = PyList::empty(py);
    for f in &self.frames[e.frames.clone()] {
      frames.append((f.address, PyBytes::new(py, &self.data[f.dat.clone()]), f.src))?;
    }
    PyTuple::new(py, [e.nanos.into_pyobject(py)?.into_any(), frames.into_any()])
  }
}

#[pymethods]
impl CanFrames {
  fn __len__(&self) -> usize {
    self.entries.len()
  }

  fn __getitem__<'py>(&self, py: Python<'py>, idx: isize) -> PyResult<Bound<'py, PyTuple>> {
    let i = if idx < 0 { idx + self.entries.len() as isize } else { idx };
    let e = usize::try_from(i).ok().and_then(|i| self.entries.get(i)).ok_or_else(|| PyIndexError::new_err("CanFrames index out of range"))?;
    self.entry(py, e)
  }

  fn __iter__<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyIterator>> {
    let entries = self.entries.iter().map(|e| self.entry(py, e)).collect::<PyResult<Vec<_>>>()?;
    PyList::new(py, entries)?.try_iter()
  }
}

/// Byte offsets of the fields read from Event and CanData, from the capnp schema
#[derive(FromPyObject)]
#[pyo3(from_item_all)]
pub struct Layout {
  log_mono_time: usize,
  discriminant: usize,
  discriminant_value: u16,
  list_ptr: usize,
  address: usize,
  src: usize,
  dat_ptr: usize,
}

fn decode_into(buf: &[u8], layout: &Layout, out: &mut CanFrames) -> crate::capnp::Result<()> {
  let msg = Message::new(buf)?;
  let event = msg.root()?;
  let which = msg.u16(&event, layout.discriminant);
  if which != layout.discriminant_value {
    return Err(format!("Event is not the requested CAN type (union discriminant {which})"));
  }
  let list = msg.struct_list(&event, layout.list_ptr)?;
  let first = out.frames.len();
  out.frames.reserve(list.len());
  for i in 0..list.len() {
    let can = list.get(i);
    let dat = msg.data(&can, layout.dat_ptr)?;
    let start = out.data.len();
    out.data.extend_from_slice(dat);
    out.frames.push(Frame { address: msg.u32(&can, layout.address), src: msg.u8(&can, layout.src), dat: start..out.data.len() });
  }
  out.entries.push(Entry { nanos: msg.u64(&event, layout.log_mono_time) as i64, frames: first..out.frames.len() });
  Ok(())
}

/// Decode serialized cereal Events holding a CAN list (can or sendcan)
#[pyfunction]
pub fn decode_can_events(strings: &Bound<'_, PyAny>, layout: Layout) -> PyResult<CanFrames> {
  let mut out = CanFrames { entries: Vec::new(), frames: Vec::new(), data: Vec::new() };
  for s in strings.try_iter()? {
    let s = s?;
    let res = if let Ok(b) = s.downcast::<PyBytes>() {
      decode_into(b.as_bytes(), &layout, &mut out)
    } else if let Ok(b) = s.downcast::<PyByteArray>() {
      decode_into(&b.to_vec(), &layout, &mut out)
    } else {
      decode_into(&s.extract::<Vec<u8>>()?, &layout, &mut out)
    };
    res.map_err(PyValueError::new_err)?;
  }
  Ok(out)
}
