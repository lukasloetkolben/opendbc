// Minimal Cap'n Proto reader, just enough to walk a serialized message from
// struct to list to data. Field offsets come from the schema at runtime (see
// can_capnp_to_list in openpilot), so nothing here knows about cereal.

pub type Result<T> = std::result::Result<T, String>;

const MAX_SEGMENTS: usize = 512;

pub struct Message<'a> {
  segments: Vec<&'a [u8]>,
}

#[derive(Clone, Copy)]
pub struct StructReader {
  seg: usize,
  data: usize, // word index
  data_words: usize,
  ptrs: usize, // word index
  ptr_count: usize,
}

pub struct StructList {
  seg: usize,
  start: usize, // word index of the first element
  len: usize,
  data_words: usize,
  ptr_count: usize,
}

impl<'a> Message<'a> {
  pub fn new(buf: &'a [u8]) -> Result<Self> {
    let u32_at = |i: usize| -> Result<usize> {
      buf.get(i..i + 4).map(|b| u32::from_le_bytes(b.try_into().unwrap()) as usize).ok_or_else(|| "truncated segment table".to_string())
    };
    let count = u32_at(0)? + 1;
    if count > MAX_SEGMENTS {
      return Err(format!("too many segments: {count}"));
    }
    let mut pos = (4 + 4 * count + 7) & !7;
    let mut segments = Vec::with_capacity(count);
    for i in 0..count {
      let len = u32_at(4 + 4 * i)? * 8;
      let seg = buf.get(pos..pos + len).ok_or_else(|| "truncated segment".to_string())?;
      segments.push(seg);
      pos += len;
    }
    Ok(Message { segments })
  }

  fn word(&self, seg: usize, idx: usize) -> Result<u64> {
    let s = self.segments.get(seg).ok_or_else(|| format!("invalid segment {seg}"))?;
    s.get(idx * 8..idx * 8 + 8).map(|b| u64::from_le_bytes(b.try_into().unwrap())).ok_or_else(|| "pointer out of bounds".to_string())
  }

  fn check_bounds(&self, seg: usize, start: usize, words: usize) -> Result<()> {
    let len = self.segments.get(seg).ok_or_else(|| format!("invalid segment {seg}"))?.len();
    if (start + words) * 8 > len {
      return Err("pointer target out of bounds".to_string());
    }
    Ok(())
  }

  /// Follow the pointer at (seg, idx), resolving far pointers.
  /// Returns (segment, target word index, pointer word with type and sizes), or None for null.
  fn resolve(&self, seg: usize, idx: usize) -> Result<Option<(usize, usize, u64)>> {
    let p = self.word(seg, idx)?;
    if p == 0 {
      return Ok(None);
    }
    if p & 3 == 2 {
      let pad_seg = (p >> 32) as usize;
      let pad = ((p as u32) >> 3) as usize;
      if (p >> 2) & 1 == 0 {
        // single far: the landing pad is a normal pointer
        let landing = self.word(pad_seg, pad)?;
        if landing & 3 == 2 {
          return Err("far pointer lands on a far pointer".to_string());
        }
        return Ok(Some((pad_seg, offset_target(pad, landing)?, landing)));
      }
      // double far: far pointer to the content, then a tag describing it
      let far = self.word(pad_seg, pad)?;
      if far & 7 != 2 {
        return Err("invalid double-far landing pad".to_string());
      }
      let tag = self.word(pad_seg, pad + 1)?;
      return Ok(Some(((far >> 32) as usize, ((far as u32) >> 3) as usize, tag)));
    }
    Ok(Some((seg, offset_target(idx, p)?, p)))
  }

  pub fn root(&self) -> Result<StructReader> {
    self.struct_at(0, 0)?.ok_or_else(|| "null root".to_string())
  }

  fn struct_at(&self, seg: usize, idx: usize) -> Result<Option<StructReader>> {
    let Some((seg, target, p)) = self.resolve(seg, idx)? else { return Ok(None) };
    if p & 3 != 0 {
      return Err("expected struct pointer".to_string());
    }
    let data_words = ((p >> 32) & 0xFFFF) as usize;
    let ptr_count = (p >> 48) as usize;
    self.check_bounds(seg, target, data_words + ptr_count)?;
    Ok(Some(StructReader { seg, data: target, data_words, ptrs: target + data_words, ptr_count }))
  }

  pub fn struct_list(&self, s: &StructReader, ptr: usize) -> Result<StructList> {
    let empty = StructList { seg: 0, start: 0, len: 0, data_words: 0, ptr_count: 0 };
    if ptr >= s.ptr_count {
      return Ok(empty);
    }
    let Some((seg, target, p)) = self.resolve(s.seg, s.ptrs + ptr)? else { return Ok(empty) };
    if p & 3 != 1 || (p >> 32) & 7 != 7 {
      return Err("expected composite list pointer".to_string());
    }
    let words = (p >> 35) as usize;
    let tag = self.word(seg, target)?;
    if tag & 3 != 0 {
      return Err("invalid composite list tag".to_string());
    }
    let len = ((tag as u32) >> 2) as usize;
    let data_words = ((tag >> 32) & 0xFFFF) as usize;
    let ptr_count = (tag >> 48) as usize;
    if len * (data_words + ptr_count) > words {
      return Err("composite list elements exceed list size".to_string());
    }
    self.check_bounds(seg, target + 1, words)?;
    Ok(StructList { seg, start: target + 1, len, data_words, ptr_count })
  }

  pub fn data(&self, s: &StructReader, ptr: usize) -> Result<&'a [u8]> {
    if ptr >= s.ptr_count {
      return Ok(&[]);
    }
    let Some((seg, target, p)) = self.resolve(s.seg, s.ptrs + ptr)? else { return Ok(&[]) };
    if p & 3 != 1 || (p >> 32) & 7 != 2 {
      return Err("expected byte list pointer".to_string());
    }
    let len = (p >> 35) as usize;
    let segment = self.segments.get(seg).ok_or_else(|| format!("invalid segment {seg}"))?;
    segment.get(target * 8..target * 8 + len).ok_or_else(|| "data out of bounds".to_string())
  }

  fn data_bytes<const N: usize>(&self, s: &StructReader, byte_offset: usize) -> [u8; N] {
    // fields past the end of the data section read as their default (zero)
    if byte_offset + N > s.data_words * 8 {
      return [0; N];
    }
    let start = s.data * 8 + byte_offset;
    self.segments[s.seg][start..start + N].try_into().unwrap()
  }

  pub fn u8(&self, s: &StructReader, byte_offset: usize) -> u8 {
    self.data_bytes::<1>(s, byte_offset)[0]
  }

  pub fn u16(&self, s: &StructReader, byte_offset: usize) -> u16 {
    u16::from_le_bytes(self.data_bytes(s, byte_offset))
  }

  pub fn u32(&self, s: &StructReader, byte_offset: usize) -> u32 {
    u32::from_le_bytes(self.data_bytes(s, byte_offset))
  }

  pub fn u64(&self, s: &StructReader, byte_offset: usize) -> u64 {
    u64::from_le_bytes(self.data_bytes(s, byte_offset))
  }
}

impl StructList {
  pub fn len(&self) -> usize {
    self.len
  }

  pub fn get(&self, i: usize) -> StructReader {
    let step = self.data_words + self.ptr_count;
    let data = self.start + i * step;
    StructReader { seg: self.seg, data, data_words: self.data_words, ptrs: data + self.data_words, ptr_count: self.ptr_count }
  }
}

fn offset_target(idx: usize, p: u64) -> Result<usize> {
  let offset = ((p as u32) as i32 >> 2) as isize;
  let target = idx as isize + 1 + offset;
  usize::try_from(target).map_err(|_| "pointer before segment start".to_string())
}
