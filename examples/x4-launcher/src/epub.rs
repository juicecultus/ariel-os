//! EPUB file parser for no_std environments
//!
//! EPUB files are ZIP archives containing:
//! - META-INF/container.xml - points to content.opf
//! - content.opf - manifest, spine (reading order), metadata
//! - *.xhtml/*.html - chapter content
//! - toc.ncx - table of contents (optional)

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;
use core::cell::RefCell;
use embedded_hal::digital::OutputPin;
use embedded_sdmmc::{Mode, VolumeManager, VolumeIdx};

use crate::SdTimeSource;

const ZIP_LOCAL_FILE_HEADER_SIG: u32 = 0x04034b50;
const ZIP_CENTRAL_DIR_SIG: u32 = 0x02014b50;
const ZIP_END_CENTRAL_DIR_SIG: u32 = 0x06054b50;

#[derive(Debug)]
pub enum EpubError {
    SdError,
    NotZip,
    FileNotFound,
    ParseError,
    DecompressError,
    XmlError,
}

pub struct EpubFile {
    pub name: String,
    pub compressed_size: u32,
    pub uncompressed_size: u32,
    pub offset: u32,
    pub compression_method: u16,
}

pub struct EpubMetadata {
    pub title: String,
    pub author: String,
    pub spine: Vec<String>, // ordered list of content file paths
}

pub struct Epub {
    pub path: String,
    pub metadata: Option<EpubMetadata>,
    pub files: Vec<EpubFile>,
    pub content_base_path: String,
}

impl Epub {
    pub fn new(path: String) -> Self {
        Self {
            path,
            metadata: None,
            files: Vec::new(),
            content_base_path: String::new(),
        }
    }

    /// Read the ZIP central directory to get file list
    pub fn read_file_list<D, CS, DELAY>(
        &mut self,
        spi_bus_ref: &RefCell<D>,
        sd_cs: CS,
        delay: DELAY,
    ) -> Result<(), EpubError>
    where
        D: embedded_hal::spi::SpiBus<u8>,
        CS: OutputPin,
        DELAY: embedded_hal::delay::DelayNs + Clone,
    {
        use embedded_hal_bus::spi::RefCellDevice;
        use embedded_sdmmc::SdCard;

        let sd_spi = RefCellDevice::new(spi_bus_ref, sd_cs, delay.clone())
            .map_err(|_| EpubError::SdError)?;
        let sd_card = SdCard::new(sd_spi, delay);
        
        let mut vm: VolumeManager<_, SdTimeSource, 4, 4, 1> = 
            VolumeManager::new(sd_card, SdTimeSource);
        
        let volume = vm.open_volume(VolumeIdx(0)).map_err(|_| EpubError::SdError)?;
        let root = volume.open_root_dir().map_err(|_| EpubError::SdError)?;
        
        // Parse path to get directory and filename
        let (dir_path, filename) = self.split_path();
        
        let dir = if dir_path.is_empty() || dir_path == "/" {
            root
        } else {
            root.open_dir(dir_path.as_str()).map_err(|_| EpubError::FileNotFound)?
        };
        
        let mut file = dir.open_file_in_dir(filename.as_str(), Mode::ReadOnly)
            .map_err(|_| EpubError::FileNotFound)?;
        
        // Read ZIP end of central directory (last 22+ bytes)
        let file_size = file.length();
        if file_size < 22 {
            return Err(EpubError::NotZip);
        }
        
        // Seek to find end of central directory
        let search_start = if file_size > 65557 { file_size - 65557 } else { 0 };
        file.seek_from_start(search_start).map_err(|_| EpubError::SdError)?;
        
        let mut buf = [0u8; 512];
        let mut eocd_offset = None;
        
        loop {
            let bytes_read = file.read(&mut buf).map_err(|_| EpubError::SdError)?;
            if bytes_read == 0 {
                break;
            }
            
            for i in 0..bytes_read.saturating_sub(3) {
                if buf[i] == 0x50 && buf[i+1] == 0x4b && buf[i+2] == 0x05 && buf[i+3] == 0x06 {
                    eocd_offset = Some(search_start + i as u32);
                    break;
                }
            }
            
            if eocd_offset.is_some() {
                break;
            }
        }
        
        let eocd_offset = eocd_offset.ok_or(EpubError::NotZip)?;
        
        // Read EOCD
        file.seek_from_start(eocd_offset).map_err(|_| EpubError::SdError)?;
        let mut eocd = [0u8; 22];
        file.read(&mut eocd).map_err(|_| EpubError::SdError)?;
        
        let central_dir_offset = u32::from_le_bytes([eocd[16], eocd[17], eocd[18], eocd[19]]);
        let num_entries = u16::from_le_bytes([eocd[10], eocd[11]]);
        
        // Read central directory entries
        file.seek_from_start(central_dir_offset).map_err(|_| EpubError::SdError)?;
        
        self.files.clear();
        
        for _ in 0..num_entries {
            let mut header = [0u8; 46];
            file.read(&mut header).map_err(|_| EpubError::SdError)?;
            
            let sig = u32::from_le_bytes([header[0], header[1], header[2], header[3]]);
            if sig != ZIP_CENTRAL_DIR_SIG {
                break;
            }
            
            let compression = u16::from_le_bytes([header[10], header[11]]);
            let compressed_size = u32::from_le_bytes([header[20], header[21], header[22], header[23]]);
            let uncompressed_size = u32::from_le_bytes([header[24], header[25], header[26], header[27]]);
            let name_len = u16::from_le_bytes([header[28], header[29]]) as usize;
            let extra_len = u16::from_le_bytes([header[30], header[31]]) as usize;
            let comment_len = u16::from_le_bytes([header[32], header[33]]) as usize;
            let local_header_offset = u32::from_le_bytes([header[42], header[43], header[44], header[45]]);
            
            // Read filename
            let mut name_buf = [0u8; 256];
            let name_to_read = name_len.min(256);
            file.read(&mut name_buf[..name_to_read]).map_err(|_| EpubError::SdError)?;
            
            if let Ok(name) = core::str::from_utf8(&name_buf[..name_to_read]) {
                self.files.push(EpubFile {
                    name: String::from(name),
                    compressed_size,
                    uncompressed_size,
                    offset: local_header_offset,
                    compression_method: compression,
                });
            }
            
            // Skip extra and comment
            if name_len > 256 {
                let skip = name_len - 256 + extra_len + comment_len;
                let current = file.offset();
                file.seek_from_start(current + skip as u32).map_err(|_| EpubError::SdError)?;
            } else {
                let skip = extra_len + comment_len;
                if skip > 0 {
                    let current = file.offset();
                    file.seek_from_start(current + skip as u32).map_err(|_| EpubError::SdError)?;
                }
            }
        }
        
        Ok(())
    }

    fn split_path(&self) -> (String, String) {
        if let Some(pos) = self.path.rfind('/') {
            let dir = String::from(&self.path[..pos]);
            let file = String::from(&self.path[pos+1..]);
            (dir, file)
        } else {
            (String::new(), self.path.clone())
        }
    }

    pub fn find_file(&self, name: &str) -> Option<&EpubFile> {
        self.files.iter().find(|f| f.name == name || f.name.ends_with(name))
    }

    pub fn get_spine_count(&self) -> usize {
        self.metadata.as_ref().map(|m| m.spine.len()).unwrap_or(0)
    }
}
