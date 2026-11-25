//! Zero-copy array storage and the on-disk graph file format.
//!
//! The graph file is a table of named, 64-byte aligned sections of plain-old-data arrays.
//! At load time the file is memory-mapped and every array is a view into the mapping: startup
//! is instant, the OS pages data in on demand, and several server processes share one copy.

use anyhow::{bail, Context};
use bytemuck::Pod;
use memmap2::Mmap;
use std::fs::File;
use std::io::{BufWriter, Seek, SeekFrom, Write};
use std::marker::PhantomData;
use std::ops::Deref;
use std::path::Path;
use std::sync::Arc;

const MAGIC: &[u8; 8] = b"SHEPHRD\x01";
pub const FORMAT_VERSION: u32 = 4;
const ALIGN: u64 = 64;
const NAME_LEN: usize = 24;

enum Owner {
    Vec(#[allow(dead_code)] Box<dyn Send + Sync>),
    Map(#[allow(dead_code)] Arc<Mmap>),
}

/// An immutable array that either owns a `Vec<T>` or borrows from a shared memory map.
/// Dereferences to `&[T]` with no copying.
pub struct Arr<T: Pod> {
    ptr: *const T,
    len: usize,
    _owner: Owner,
    _t: PhantomData<T>,
}

// SAFETY: the data is immutable and the owner keeps it alive; T is Pod (no interior refs).
unsafe impl<T: Pod + Send> Send for Arr<T> {}
unsafe impl<T: Pod + Sync> Sync for Arr<T> {}

impl<T: Pod + Send + Sync> Arr<T> {
    pub fn from_vec(v: Vec<T>) -> Self {
        let b = v.into_boxed_slice();
        let ptr = b.as_ptr();
        let len = b.len();
        Arr {
            ptr,
            len,
            _owner: Owner::Vec(Box::new(b)),
            _t: PhantomData,
        }
    }
}

impl<T: Pod> Deref for Arr<T> {
    type Target = [T];
    #[inline]
    fn deref(&self) -> &[T] {
        if self.len == 0 {
            return &[];
        }
        // SAFETY: ptr/len describe memory kept alive by `_owner`, aligned (checked at load).
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}

impl<T: Pod + Send + Sync> From<Vec<T>> for Arr<T> {
    fn from(v: Vec<T>) -> Self {
        Arr::from_vec(v)
    }
}

impl<T: Pod + std::fmt::Debug> std::fmt::Debug for Arr<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Arr(len={})", self.len)
    }
}

/// Writes named sections to a graph file. Data streams straight to disk; the header (in a
/// fixed reserved area at the start) is written last.
pub struct Writer {
    out: BufWriter<File>,
    entries: Vec<([u8; NAME_LEN], u64, u64)>,
    pos: u64,
}

const MAX_SECTIONS: usize = 100;
const HEADER_AREA: u64 = 4096;

impl Writer {
    pub fn create(path: &Path) -> anyhow::Result<Self> {
        let f = File::create(path).with_context(|| format!("creating {}", path.display()))?;
        let mut out = BufWriter::with_capacity(1 << 20, f);
        out.write_all(&vec![0u8; HEADER_AREA as usize])?;
        Ok(Writer {
            out,
            entries: Vec::new(),
            pos: HEADER_AREA,
        })
    }

    pub fn add<T: Pod>(&mut self, name: &str, data: &[T]) -> anyhow::Result<()> {
        assert!(name.len() <= NAME_LEN, "section name too long: {name}");
        assert!(self.entries.len() < MAX_SECTIONS, "too many sections");
        let mut n = [0u8; NAME_LEN];
        n[..name.len()].copy_from_slice(name.as_bytes());
        let bytes: &[u8] = bytemuck::cast_slice(data);
        self.out.write_all(bytes)?;
        self.entries.push((n, self.pos, bytes.len() as u64));
        self.pos += bytes.len() as u64;
        let aligned = self.pos.div_ceil(ALIGN) * ALIGN;
        self.out.write_all(&vec![0u8; (aligned - self.pos) as usize])?;
        self.pos = aligned;
        Ok(())
    }

    pub fn finish(mut self) -> anyhow::Result<()> {
        self.out.flush()?;
        let mut header = Vec::with_capacity(HEADER_AREA as usize);
        header.extend_from_slice(MAGIC);
        header.extend_from_slice(&FORMAT_VERSION.to_le_bytes());
        header.extend_from_slice(&(self.entries.len() as u32).to_le_bytes());
        for (name, off, len) in &self.entries {
            header.extend_from_slice(name);
            header.extend_from_slice(&off.to_le_bytes());
            header.extend_from_slice(&len.to_le_bytes());
        }
        let f = self.out.get_mut();
        f.seek(SeekFrom::Start(0))?;
        f.write_all(&header)?;
        f.sync_all()?;
        Ok(())
    }
}

/// A memory-mapped graph file.
pub struct Reader {
    map: Arc<Mmap>,
    entries: Vec<(String, u64, u64)>,
}

impl Reader {
    pub fn open(path: &Path) -> anyhow::Result<Self> {
        let f = File::open(path).with_context(|| format!("opening {}", path.display()))?;
        // SAFETY: the file is treated as read-only; replacing it on disk must be done by
        // writing a new file and renaming (see the refresh script), never in place.
        let map = unsafe { Mmap::map(&f)? };
        if map.len() < 16 || &map[..8] != MAGIC {
            bail!("{} is not a shepherd graph file", path.display());
        }
        let version = u32::from_le_bytes(map[8..12].try_into().unwrap());
        if version != FORMAT_VERSION {
            bail!("graph file version {version}, expected {FORMAT_VERSION}; re-run preprocess");
        }
        let count = u32::from_le_bytes(map[12..16].try_into().unwrap()) as usize;
        let mut entries = Vec::with_capacity(count);
        let mut p = 16;
        for _ in 0..count {
            let name = &map[p..p + NAME_LEN];
            let name = String::from_utf8_lossy(name).trim_end_matches('\0').to_string();
            let off = u64::from_le_bytes(map[p + NAME_LEN..p + NAME_LEN + 8].try_into().unwrap());
            let len = u64::from_le_bytes(map[p + NAME_LEN + 8..p + NAME_LEN + 16].try_into().unwrap());
            if off + len > map.len() as u64 {
                bail!("section {name} is out of bounds; file truncated?");
            }
            entries.push((name, off, len));
            p += NAME_LEN + 16;
        }
        Ok(Reader {
            map: Arc::new(map),
            entries,
        })
    }

    pub fn get<T: Pod>(&self, name: &str) -> anyhow::Result<Arr<T>> {
        let (_, off, len) = self
            .entries
            .iter()
            .find(|e| e.0 == name)
            .with_context(|| format!("graph file has no section {name}"))?;
        let size = std::mem::size_of::<T>() as u64;
        if size == 0 || len % size != 0 {
            bail!("section {name} has wrong element size");
        }
        let ptr = unsafe { self.map.as_ptr().add(*off as usize) } as *const T;
        if (ptr as usize) % std::mem::align_of::<T>() != 0 {
            bail!("section {name} is misaligned");
        }
        Ok(Arr {
            ptr,
            len: (len / size) as usize,
            _owner: Owner::Map(self.map.clone()),
            _t: PhantomData,
        })
    }

    pub fn bytes(&self, name: &str) -> anyhow::Result<&[u8]> {
        let (_, off, len) = self
            .entries
            .iter()
            .find(|e| e.0 == name)
            .with_context(|| format!("graph file has no section {name}"))?;
        Ok(&self.map[*off as usize..(*off + *len) as usize])
    }
}
