use embedded_storage_async::nor_flash::{NorFlash, NorFlashErrorKind};

use crate::crc::*;

const DFU_PROTOCOL_VERSION: u8 = 0x01;
const OBJ_TYPE_COMMAND_IDX: usize = 0;
const OBJ_TYPE_DATA_IDX: usize = 1;

/// Represents the target of a firmware update.
///
/// Implements the DFU protocol by processing requests handed to it
/// from the transport, writing to the underlying storage and returning
/// the appropriate responses according to the protocol.
pub struct DfuTarget<const MTU: usize> {
    crc_receipt_interval: u16,
    receipt_count: u16,
    objects: [Object; 2],
    current: usize,
    fw_info: FirmwareInfo,
    hw_info: HardwareInfo,

    buffer: AlignedBuffer<MTU>,
    offset: usize,
    boffset: usize,
}

/// Status of the DFU process.
#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DfuStatus {
    /// DFU process is still ongoing.
    InProgress,
    /// DFU process is done, should reset.
    DoneReset,
}

/// Object representing a firmware blob. Tracks information about the firmware to be updated such as the CRC.
pub struct Object {
    obj_type: ObjectType,
    offset: u32,
    crc: Crc32,
    size: u32,
}

/// Information about the firmware.
pub struct FirmwareInfo {
    /// Firmware type.
    pub ftype: FirmwareType,
    /// Firmware version.
    pub version: u32,
    /// Firmware flash address.
    pub addr: u32,
    /// Firmware size,
    pub len: u32,
}

/// Information about the hardware.
pub struct HardwareInfo {
    /// Part number.
    pub part: u32,
    /// Chip variant.
    pub variant: u32,
    /// Size of flash.
    pub rom_size: u32,
    /// Size of RAM.
    pub ram_size: u32,
    /// Flash page size,
    pub rom_page_size: u32,
}

/// Represents a DFU request according to the nRF DFU protocol from SDK 17.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum DfuRequest<'m> {
    /// Report protocol version.
    ProtocolVersion,
    /// Create a new firmware object.
    Create { obj_type: ObjectType, obj_size: u32 },
    /// Set receipt notification interval.
    SetReceiptNotification { target: u16 },
    /// Request CRC of currently selected object.
    Crc,
    /// Validate and enable new firmware.
    Execute,
    /// Select object type as current.
    Select { obj_type: ObjectType },
    /// Request MTU.
    MtuGet,
    /// Write firmware data.
    Write { data: &'m [u8] },
    /// Health check.
    Ping { id: u8 },
    /// Request hardware version.
    HwVersion,
    /// Request firmware version.
    FwVersion { image_id: u8 },
    /// Abort firmware update process.
    Abort,
}

/// Represents a DFU response according to the nRF DFU protocol from SDK 17.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DfuResponse<'m> {
    request: DfuRequest<'m>,
    result: DfuResult,
    body: Option<DfuResponseBody>,
}

/// Possible result values for a response as defined by the protocol.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum DfuResult {
    Invalid,
    Success,
    OpNotSupported,
    InvalidParameter,
    InsufficientResources,
    InvalidObject,
    UnsupportedType,
    OpNotPermitted,
    OpFailed,
    ExtError(u8),
}

/// Response bodies for requests that have responses.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum DfuResponseBody {
    /// Protocol version number.
    ProtocolVersion { version: u8 },
    /// CRC of current object.
    Crc { offset: u32, crc: u32 },
    /// Info about selected object.
    Select {
        offset: u32,
        crc: u32,
        max_size: u32,
    },
    /// Requested MTU.
    Mtu { mtu: u16 },
    /// CRC of firmware written so far.
    Write { crc: u32 },
    /// Ping response.
    Ping { id: u8 },
    /// Hardware version.
    HwVersion {
        part: u32,
        variant: u32,
        rom_size: u32,
        ram_size: u32,
        rom_page_size: u32,
    },
    /// Firmware version.
    FwVersion {
        ftype: FirmwareType,
        version: u32,
        addr: u32,
        len: u32,
    },
}

/// Object types supported by the protocol.
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum ObjectType {
    Invalid = 0,
    Command = 1,
    Data = 2,
}

/// Firmware types supported by the protocol.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum FirmwareType {
    Softdevice,
    Application,
    Bootloader,
    Unknown,
}

#[repr(align(32))]
struct AlignedBuffer<const N: usize>(pub [u8; N]);

/// Errors returned by the DFU target.
#[derive(Debug)]
pub enum Error {
    /// Errors from the underlying flash.
    Flash(NorFlashErrorKind),
    /// Unexpected MTU from controller.
    Mtu,
}

impl From<NorFlashErrorKind> for Error {
    fn from(error: NorFlashErrorKind) -> Self {
        Self::Flash(error)
    }
}

impl<const MTU: usize> DfuTarget<MTU> {
    /// Create a new instance of a DFU target.
    pub fn new(flash_size: u32, fw_info: FirmwareInfo, hw_info: HardwareInfo) -> Self {
        Self {
            crc_receipt_interval: 0,
            receipt_count: 0,
            objects: [
                Object {
                    obj_type: ObjectType::Command,
                    offset: 0,
                    crc: Crc32::init(),
                    size: 512,
                },
                Object {
                    obj_type: ObjectType::Data,
                    offset: 0,
                    crc: Crc32::init(),
                    size: flash_size,
                },
            ],

            current: 0,
            fw_info,
            hw_info,

            buffer: AlignedBuffer([0; MTU]),
            boffset: 0,
            offset: 0,
        }
    }

    /// Process the DFU request as specified by the protocol.
    ///
    /// Any firmware writes will be written to the provided flash
    /// region at the offsets requested by the DFU controller.
    ///
    /// The returned response should be sent back to the controller,
    /// and the status should be examined to see if the device should
    /// swap to the new firmware.
    pub async fn process<'m, DFU: NorFlash>(
        &mut self,
        request: DfuRequest<'m>,
        dfu: &mut DFU,
    ) -> (DfuResponse<'m>, DfuStatus) {
        trace!("DFU REQUEST {:?}", request);
        let (response, status) = self.process_inner(request, dfu).await;
        trace!("DFU RESPONSE {:?}", response);
        (response, status)
    }

    async fn process_inner<'m, DFU: NorFlash>(
        &mut self,
        request: DfuRequest<'m>,
        dfu: &mut DFU,
    ) -> (DfuResponse<'m>, DfuStatus) {
        match request {
            DfuRequest::ProtocolVersion => (
                DfuResponse::new(request, DfuResult::Success).body(
                    DfuResponseBody::ProtocolVersion {
                        version: DFU_PROTOCOL_VERSION,
                    },
                ),
                DfuStatus::InProgress,
            ),
            DfuRequest::Create { obj_type, obj_size } => {
                let idx = match obj_type {
                    ObjectType::Command => Some(OBJ_TYPE_COMMAND_IDX),
                    ObjectType::Data => Some(OBJ_TYPE_DATA_IDX),
                    _ => None,
                };
                if let Some(idx) = idx {
                    self.objects[idx] = Object {
                        obj_type,
                        size: obj_size,
                        offset: 0,
                        crc: Crc32::init(),
                    };
                    self.current = idx;
                    if let ObjectType::Data = obj_type {
                        let size = self.objects[self.current].size;
                        let to = size + (DFU::ERASE_SIZE as u32 - size % DFU::ERASE_SIZE as u32);
                        match dfu.erase(0, to as u32).await {
                            Ok(_) => {
                                self.objects[self.current].offset = 0;
                                self.boffset = 0;
                                self.offset = 0;
                            }
                            Err(e) => {
                                #[cfg(feature = "defmt")]
                                let e = defmt::Debug2Format(&e);
                                warn!("Error during erase: {:?}", e);
                            }
                        }
                    }
                }
                self.receipt_count = 0;
                (
                    DfuResponse::new(request, DfuResult::Success),
                    DfuStatus::InProgress,
                )
            }
            DfuRequest::SetReceiptNotification { target } => {
                self.crc_receipt_interval = target;
                (
                    DfuResponse::new(request, DfuResult::Success),
                    DfuStatus::InProgress,
                )
            }
            DfuRequest::Crc => (
                DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::Crc {
                    offset: self.objects[self.current].offset,
                    crc: self.objects[self.current].crc.finish(),
                }),
                DfuStatus::InProgress,
            ),
            DfuRequest::Execute => {
                let obj = &mut self.objects[self.current];
                if obj.offset != obj.size {
                    (
                        DfuResponse::new(request, DfuResult::OpNotSupported),
                        DfuStatus::InProgress,
                    )
                } else {
                    if let ObjectType::Data = obj.obj_type {
                        // Flush remaining data
                        if self.boffset > 0 {
                            // Write at previous boundary which was left in memory.
                            if let Err(e) = dfu.write(self.offset as u32, &self.buffer.0).await {
                                #[cfg(feature = "defmt")]
                                let e = defmt::Debug2Format(&e);
                                warn!("Write Error: {:?}", e);
                                return (
                                    DfuResponse::new(request, DfuResult::OpFailed),
                                    DfuStatus::InProgress,
                                );
                            }
                            self.offset += self.boffset;
                            self.boffset = 0;
                        }
                        info!("Verifying firmware integrity");
                        let mut check = Crc32::init();
                        let mut buf = [0; MTU];
                        let mut offset: u32 = 0;
                        let size = obj.size;
                        while offset < size {
                            let to_read = core::cmp::min(buf.len(), (size - offset) as usize);
                            match dfu.read(offset, &mut buf[..to_read]).await {
                                Ok(_) => {
                                    check.add(&buf[..to_read]);
                                    offset += to_read as u32;
                                }
                                Err(e) => {
                                    #[cfg(feature = "defmt")]
                                    let e = defmt::Debug2Format(&e);
                                    warn!("Error verifying firmware integrity: {:?}", e);
                                    break;
                                }
                            }
                        }

                        if obj.crc.finish() == check.finish() {
                            info!("Firmware CRC check success");

                            // Fill the rest with some magic
                            if size < dfu.capacity() as u32 {
                                let magic = AlignedBuffer([
                                    0xf3, 0x95, 0xc2, 0x77, 0x7f, 0xef, 0xd2, 0x60, 0x0f, 0x50,
                                    0x52, 0x35, 0x80, 0x79, 0xb6, 0x2c,
                                ]);
                                for offset in (size..dfu.capacity() as u32).step_by(magic.0.len()) {
                                    let to_write =
                                        (magic.0.len()).min(dfu.capacity() - offset as usize);
                                    if let Err(e) = dfu.write(offset, &magic.0[..to_write]).await {
                                        #[cfg(feature = "defmt")]
                                        let e = defmt::Debug2Format(&e);
                                        warn!("Write Error: {:?}", e);
                                        return (
                                            DfuResponse::new(request, DfuResult::OpFailed),
                                            DfuStatus::InProgress,
                                        );
                                    }
                                }
                            }
                            (
                                DfuResponse::new(request, DfuResult::Success),
                                DfuStatus::DoneReset,
                            )
                        } else {
                            warn!("Firmware CRC check error");
                            (
                                DfuResponse::new(request, DfuResult::OpFailed),
                                DfuStatus::InProgress,
                            )
                        }
                    } else {
                        (
                            DfuResponse::new(request, DfuResult::Success),
                            DfuStatus::InProgress,
                        )
                    }
                }
            }
            DfuRequest::Select { obj_type } => {
                let idx = match obj_type {
                    ObjectType::Command => Some(OBJ_TYPE_COMMAND_IDX),
                    ObjectType::Data => Some(OBJ_TYPE_DATA_IDX),
                    _ => None,
                };
                if let Some(idx) = idx {
                    (
                        DfuResponse::new(request, DfuResult::Success).body(
                            DfuResponseBody::Select {
                                offset: self.objects[idx].offset,
                                crc: self.objects[idx].crc.finish(),
                                max_size: self.objects[idx].size,
                            },
                        ),
                        DfuStatus::InProgress,
                    )
                } else {
                    (
                        DfuResponse::new(request, DfuResult::InvalidObject),
                        DfuStatus::InProgress,
                    )
                }
            }

            DfuRequest::MtuGet => (
                DfuResponse::new(request, DfuResult::Success)
                    .body(DfuResponseBody::Mtu { mtu: MTU as u16 }),
                DfuStatus::InProgress,
            ),
            DfuRequest::Write { data } => {
                let obj = &mut self.objects[self.current];
                if let ObjectType::Data = obj.obj_type {
                    let mut pos = 0;
                    while pos < data.len() {
                        let to_copy =
                            core::cmp::min(data.len() - pos, self.buffer.0.len() - self.boffset);
                        //info!("Copying {} bytes to internal buffer", to_copy);
                        self.buffer.0[self.boffset..self.boffset + to_copy]
                            .copy_from_slice(&data[pos..pos + to_copy]);

                        if self.boffset == self.buffer.0.len() {
                            if let Err(e) = dfu.write(self.offset as u32, &self.buffer.0).await {
                                #[cfg(feature = "defmt")]
                                let e = defmt::Debug2Format(&e);
                                warn!("Write Error: {:?}", e);
                                return (
                                    DfuResponse::new(request, DfuResult::OpFailed),
                                    DfuStatus::InProgress,
                                );
                            }

                            self.offset += self.boffset;
                            self.boffset = 0;
                        } else {
                            self.boffset += to_copy;
                        }
                        pos += to_copy;
                        // info!("Wrote {} bytes to flash (total {})", to_copy, self.offset);
                    }
                }
                obj.crc.add(data);
                obj.offset += data.len() as u32;

                let mut response = DfuResponse::new(DfuRequest::Crc, DfuResult::Success);
                if self.crc_receipt_interval > 0 {
                    self.receipt_count += 1;
                    if self.receipt_count == self.crc_receipt_interval {
                        self.receipt_count = 0;
                        response = response.body(DfuResponseBody::Crc {
                            offset: obj.offset,
                            crc: obj.crc.finish(),
                        });
                    }
                }
                (response, DfuStatus::InProgress)
            }
            DfuRequest::Ping { id } => (
                DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::Ping { id }),
                DfuStatus::InProgress,
            ),
            DfuRequest::HwVersion => (
                DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::HwVersion {
                    part: self.hw_info.part,
                    variant: self.hw_info.variant,
                    rom_size: self.hw_info.rom_size,
                    ram_size: self.hw_info.ram_size,
                    rom_page_size: self.hw_info.rom_page_size,
                }),
                DfuStatus::InProgress,
            ),
            DfuRequest::FwVersion { image_id: _ } => (
                DfuResponse::new(request, DfuResult::Success).body(DfuResponseBody::FwVersion {
                    ftype: self.fw_info.ftype,
                    version: self.fw_info.version,
                    addr: self.fw_info.addr,
                    len: self.fw_info.len,
                }),
                DfuStatus::InProgress,
            ),
            DfuRequest::Abort => {
                self.objects[0].crc.reset();
                self.objects[0].offset = 0;
                self.objects[1].crc.reset();
                self.objects[1].offset = 0;
                self.receipt_count = 0;
                self.boffset = 0;
                self.offset = 0;
                (
                    DfuResponse::new(request, DfuResult::Success),
                    DfuStatus::InProgress,
                )
            }
        }
    }
}

impl<'m> DfuRequest<'m> {
    fn code(&self) -> u8 {
        match self {
            Self::ProtocolVersion => 0x00,
            Self::Create {
                obj_type: _,
                obj_size: _,
            } => 0x01,
            Self::SetReceiptNotification { target: _ } => 0x02,
            Self::Crc => 0x03,
            Self::Execute => 0x04,
            Self::Select { obj_type: _ } => 0x06,
            Self::MtuGet => 0x07,
            Self::Write { data: _ } => 0x08,
            Self::Ping { id: _ } => 0x09,
            Self::HwVersion => 0x0A,
            Self::FwVersion { image_id: _ } => 0x0B,
            Self::Abort => 0x0C,
        }
    }

    /// Decode the request from the data and return excess data not parsed.
    pub fn decode(data: &'m [u8]) -> Result<(DfuRequest<'m>, &'m [u8]), ()> {
        let mut data = ReadBuf::new(data);
        let op = data.decode_u8()?;
        let req = match op {
            0x00 => Ok(Self::ProtocolVersion),
            0x01 => {
                let obj_type = ObjectType::try_from(data.decode_u8()?)?;
                let obj_size = data.decode_u32()?;

                Ok(Self::Create { obj_type, obj_size })
            }
            0x02 => {
                let target = data.decode_u16()?;
                Ok(Self::SetReceiptNotification { target })
            }
            0x03 => Ok(Self::Crc),
            0x04 => Ok(Self::Execute),
            0x06 => {
                let obj_type = ObjectType::try_from(data.decode_u8()?)?;
                Ok(Self::Select { obj_type })
            }
            0x07 => Ok(Self::MtuGet),
            0x08 => Ok(Self::Write { data: data.slice() }),
            0x09 => {
                let id = data.decode_u8()?;
                Ok(Self::Ping { id })
            }
            0x0A => Ok(Self::HwVersion),
            0x0B => {
                let image_id = data.decode_u8()?;
                Ok(Self::FwVersion { image_id })
            }
            0x0C => Ok(Self::Abort),
            _ => Err(()),
        }?;

        Ok((req, data.release()))
    }
}

impl<'m> DfuResponse<'m> {
    /// Create a response for a given request with the reported result.
    pub fn new(request: DfuRequest<'m>, result: DfuResult) -> DfuResponse<'m> {
        DfuResponse {
            request,
            result,
            body: None,
        }
    }

    /// Set a response body.
    pub fn body(self, body: DfuResponseBody) -> Self {
        Self {
            request: self.request,
            result: self.result,
            body: Some(body),
        }
    }

    /// Encode the response into the provided buffer.
    pub fn encode(&self, buf: &mut [u8]) -> Result<usize, ()> {
        let mut buf = WriteBuf::new(buf);
        const DFU_RESPONSE_OP_CODE: u8 = 0x60;
        buf.encode_u8(DFU_RESPONSE_OP_CODE)?;
        buf.encode_u8(self.request.code())?;

        let len = self.result.encode(buf.slice())?;
        buf.advance(len)?;

        if let Some(body) = self.body {
            let len = body.encode(buf.slice())?;
            buf.advance(len)?;
        }
        Ok(buf.release())
    }
}

impl DfuResult {
    fn encode(&self, buf: &mut [u8]) -> Result<usize, ()> {
        let mut buf = WriteBuf::new(buf);
        let code = match self {
            DfuResult::Invalid => 0x00,
            DfuResult::Success => 0x01,
            DfuResult::OpNotSupported => 0x02,
            DfuResult::InvalidParameter => 0x03,
            DfuResult::InsufficientResources => 0x04,
            DfuResult::InvalidObject => 0x05,
            DfuResult::UnsupportedType => 0x07,
            DfuResult::OpNotPermitted => 0x08,
            DfuResult::OpFailed => 0x0A,
            DfuResult::ExtError(_) => 0x0B,
        };

        buf.encode_u8(code)?;
        if let DfuResult::ExtError(code) = self {
            buf.encode_u8(*code)?;
        }

        Ok(buf.pos)
    }
}

impl DfuResponseBody {
    fn encode(&self, buf: &mut [u8]) -> Result<usize, ()> {
        let mut buf = WriteBuf::new(buf);
        match &self {
            DfuResponseBody::ProtocolVersion { version } => buf.encode_u8(*version)?,
            DfuResponseBody::Crc { offset, crc } => {
                buf.encode_u32(*offset)?;
                buf.encode_u32(*crc)?;
            }
            DfuResponseBody::Select {
                offset,
                crc,
                max_size,
            } => {
                buf.encode_u32(*max_size)?;
                buf.encode_u32(*offset)?;
                buf.encode_u32(*crc)?;
            }
            DfuResponseBody::Mtu { mtu } => {
                buf.encode_u16(*mtu)?;
            }
            DfuResponseBody::Write { crc } => {
                buf.encode_u32(*crc)?;
            }
            DfuResponseBody::Ping { id } => {
                buf.encode_u8(*id)?;
            }
            DfuResponseBody::HwVersion {
                part,
                variant,
                rom_size,
                ram_size,
                rom_page_size,
            } => {
                buf.encode_u32(*part)?;
                buf.encode_u32(*variant)?;
                buf.encode_u32(*rom_size)?;
                buf.encode_u32(*ram_size)?;
                buf.encode_u32(*rom_page_size)?;
            }
            DfuResponseBody::FwVersion {
                ftype,
                version,
                addr,
                len,
            } => {
                buf.encode_u8((*ftype).into())?;
                buf.encode_u32(*version)?;
                buf.encode_u32(*addr)?;
                buf.encode_u32(*len)?;
            }
        }
        Ok(buf.release())
    }
}

impl TryFrom<u8> for ObjectType {
    type Error = ();
    fn try_from(t: u8) -> Result<Self, Self::Error> {
        match t {
            0 => Ok(Self::Invalid),
            1 => Ok(Self::Command),
            2 => Ok(Self::Data),
            _ => Err(()),
        }
    }
}

impl Into<u8> for ObjectType {
    fn into(self) -> u8 {
        match self {
            Self::Invalid => 0x00,
            Self::Command => 0x01,
            Self::Data => 0x02,
        }
    }
}

impl Into<u8> for FirmwareType {
    fn into(self) -> u8 {
        match self {
            Self::Softdevice => 0x00,
            Self::Application => 0x01,
            Self::Bootloader => 0x02,
            Self::Unknown => 0xFF,
        }
    }
}

struct ReadBuf<'m> {
    data: &'m [u8],
    pos: usize,
}

impl<'m> ReadBuf<'m> {
    fn new(data: &'m [u8]) -> ReadBuf<'m> {
        Self { data, pos: 0 }
    }

    fn decode_u8(&mut self) -> Result<u8, ()> {
        if self.data.len() - self.pos >= 1 {
            let b = self.data[self.pos];
            self.pos += 1;
            Ok(b)
        } else {
            Err(())
        }
    }

    fn decode_u16(&mut self) -> Result<u16, ()> {
        if self.data.len() - self.pos >= 2 {
            let b = u16::from_le_bytes([self.data[self.pos], self.data[self.pos + 1]]);
            self.pos += 2;
            Ok(b)
        } else {
            Err(())
        }
    }

    fn decode_u32(&mut self) -> Result<u32, ()> {
        if self.data.len() - self.pos >= 4 {
            let b = u32::from_le_bytes([
                self.data[self.pos],
                self.data[self.pos + 1],
                self.data[self.pos + 2],
                self.data[self.pos + 3],
            ]);
            self.pos += 4;
            Ok(b)
        } else {
            Err(())
        }
    }

    fn slice(&mut self) -> &'m [u8] {
        let s = &self.data[self.pos..];
        self.pos += s.len();
        s
    }

    fn release(self) -> &'m [u8] {
        &self.data[self.pos..]
    }
}

struct WriteBuf<'m> {
    data: &'m mut [u8],
    pos: usize,
}

impl<'m> WriteBuf<'m> {
    fn new(data: &'m mut [u8]) -> WriteBuf<'m> {
        Self { data, pos: 0 }
    }

    fn encode_u8(&mut self, value: u8) -> Result<(), ()> {
        if self.data.len() - self.pos >= 1 {
            self.data[self.pos] = value;
            self.pos += 1;
            Ok(())
        } else {
            Err(())
        }
    }

    fn encode_u16(&mut self, value: u16) -> Result<(), ()> {
        if self.data.len() - self.pos >= 2 {
            let d = value.to_le_bytes();
            self.data[self.pos] = d[0];
            self.data[self.pos + 1] = d[1];
            self.pos += 2;
            Ok(())
        } else {
            Err(())
        }
    }

    fn encode_u32(&mut self, value: u32) -> Result<(), ()> {
        if self.data.len() - self.pos >= 4 {
            let d = value.to_le_bytes();
            self.data[self.pos] = d[0];
            self.data[self.pos + 1] = d[1];
            self.data[self.pos + 2] = d[2];
            self.data[self.pos + 3] = d[3];
            self.pos += 4;
            Ok(())
        } else {
            Err(())
        }
    }

    fn advance(&mut self, amount: usize) -> Result<(), ()> {
        if self.data.len() - self.pos >= amount {
            self.pos += amount;
            Ok(())
        } else {
            Err(())
        }
    }

    fn release(self) -> usize {
        self.pos
    }

    fn slice(&mut self) -> &mut [u8] {
        &mut self.data[self.pos..]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[futures_test::test]
    async fn test_update_protocol() {
        let mut test_flash: MemFlash<65536, 4096, 1> = MemFlash::new(0);
        let firmware = [13; 12345];
        let mut crc = Crc32::init();
        crc.add(&firmware);
        let expected_crc = crc.finish();

        let mut target: DfuTarget<256> = DfuTarget::new(
            65536,
            FirmwareInfo {
                ftype: FirmwareType::Application,
                version: 0,
                addr: 0,
                len: 0,
            },
            HardwareInfo {
                part: 0,
                variant: 0,
                rom_size: 0,
                ram_size: 0,
                rom_page_size: 0,
            },
        );

        let response = target
            .process(
                DfuRequest::Create {
                    obj_type: ObjectType::Data,
                    obj_size: firmware.len() as u32,
                },
                &mut test_flash,
            )
            .await;

        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        assert_eq!(&[0xff; 12345], &test_flash.mem[0..12345]);

        for chunk in firmware.chunks(120) {
            let response = target
                .process(DfuRequest::Write { data: chunk }, &mut test_flash)
                .await;
            assert_eq!(DfuResult::Success, response.0.result);
            assert_eq!(DfuStatus::InProgress, response.1);
        }

        let response = target.process(DfuRequest::Crc, &mut test_flash).await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        let body = response.0.body.unwrap();

        if let Some(DfuResponseBody::Crc { offset, crc }) = response.0.body {
            assert_eq!(expected_crc, crc);
            assert_eq!(12345, offset);
        } else {
            panic!("Unexpected DFU response body: {:?}", body);
        }

        let response = target.process(DfuRequest::Execute, &mut test_flash).await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::DoneReset, response.1);
        assert_eq!(&test_flash.mem[0..12345], firmware);
    }

    #[futures_test::test]
    async fn test_full_firmware_update() {
        let mut test_flash: MemFlash<65536, 4096, 1> = MemFlash::new(0);
        let fake_data: [u8; 8] = [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00, 0x11];

        let mut target: DfuTarget<4> = DfuTarget::new(
            65536,
            FirmwareInfo {
                ftype: FirmwareType::Application,
                version: 0,
                addr: 0,
                len: 0,
            },
            HardwareInfo {
                part: 0,
                variant: 0,
                rom_size: 0,
                ram_size: 0,
                rom_page_size: 0,
            },
        );

        /*
        REQUEST Select { obj_type: Command }
        RESPONSE DfuResponse { request: Select { obj_type: Command }, result: Success, body: Some(Select { offset: 0, crc: 0, max_size: 512 }) }
        */
        let response = target
            .process(
                DfuRequest::Select {
                    obj_type: ObjectType::Command,
                },
                &mut test_flash,
            )
            .await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        let body = response.0.body.unwrap();
        if let Some(DfuResponseBody::Select {
            offset,
            crc,
            max_size,
        }) = response.0.body
        {
            assert_eq!(0, crc);
            assert_eq!(0, offset);
            assert_eq!(target.objects[0].size, max_size);
        } else {
            panic!("Unexpected DFU response body: {:?}", body);
        }

        /*
        REQUEST SetReceiptNotification { target: 0 }
        RESPONSE DfuResponse { request: SetReceiptNotification { target: 0 }, result: Success, body: None }
        */
        let response = target
            .process(
                DfuRequest::SetReceiptNotification { target: 0 },
                &mut test_flash,
            )
            .await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        assert_eq!(response.0.body.is_none(), true);

        assert_eq!(target.offset, 0);

        /*
        // In the original trace, the obj_size was 79 bytes
        REQUEST Create { obj_type: Command, obj_size: 8 }
        RESPONSE DfuResponse { request: Create { obj_type: Command, obj_size: 8 }, result: Success, body: None }
        */
        let response = target
            .process(
                DfuRequest::Create {
                    obj_type: ObjectType::Command,
                    obj_size: fake_data.len() as u32,
                },
                &mut test_flash,
            )
            .await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        assert_eq!(response.0.body.is_none(), true);

        /*
        REQUEST Write { data: [..] }
        RESPONSE DfuResponse { request: Crc, result: Success, body: None }
        */
        // TODO: It seems that header isn't really validated, as long as
        // the response body is empty.
        let response = target
            .process(DfuRequest::Write { data: &fake_data }, &mut test_flash)
            .await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        assert_eq!(response.0.body.is_none(), true);

        // Validate whether DFU target object is there
        assert_eq!(target.current, 0);
        assert_eq!(target.objects[target.current].obj_type, ObjectType::Command);
        assert_eq!(target.objects[target.current].size, fake_data.len() as u32);

        /*
        REQUEST Crc
        RESPONSE DfuResponse { request: Crc, result: Success, body: Some(Crc { offset: 8, crc: }) }
        */
        let response = target.process(DfuRequest::Crc {}, &mut test_flash).await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        if let Some(DfuResponseBody::Crc { offset, crc }) = response.0.body {
            assert_eq!(target.objects[target.current].offset, offset);
            assert_eq!(target.objects[target.current].size, offset);
            assert_eq!(target.objects[target.current].crc.finish(), crc);
        } else {
            panic!("Unexpected DFU response body: {:?}", body);
        };

        /*
        REQUEST Execute
        RESPONSE DfuResponse { request: Execute, result: Success, body: None }
        */
        let response = target
            .process(DfuRequest::Execute {}, &mut test_flash)
            .await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        assert_eq!(response.0.body.is_none(), true);

        // Control packet is now completed, switch to firmware transfer

        /*
        REQUEST SetReceiptNotification { target: 5 }
        RESPONSE DfuResponse { request: SetReceiptNotification { target: 5 }, result: Success, body: None }
        */
        let response = target
            .process(
                DfuRequest::SetReceiptNotification { target: 5 },
                &mut test_flash,
            )
            .await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        assert_eq!(response.0.body.is_none(), true);
        assert_eq!(target.crc_receipt_interval, 5);

        /*
        REQUEST Select { obj_type: Data }
        RESPONSE DfuResponse { request: Select { obj_type: Data }, result: Success, body: Some(Select { offset: 0, crc: 0, max_size: 335872 }) }
        */
        let response = target
            .process(
                DfuRequest::Select {
                    obj_type: ObjectType::Data,
                },
                &mut test_flash,
            )
            .await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        if let Some(DfuResponseBody::Select {
            offset,
            crc,
            max_size,
        }) = response.0.body
        {
            assert_eq!(0, crc);
            assert_eq!(0, offset);
            assert_eq!(target.objects[1].size, max_size);
        } else {
            panic!("Unexpected DFU response body: {:?}", body);
        }

        /*
        REQUEST Create { obj_type: Data, obj_size: 12 }
        RESPONSE DfuResponse { request: Create { obj_type: Data, obj_size: 12}, result: Success, body: None }
        */
        let response = target
            .process(
                DfuRequest::Create {
                    obj_type: ObjectType::Data,
                    obj_size: 12,
                },
                &mut test_flash,
            )
            .await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        assert_eq!(response.0.body.is_none(), true);

        for i in 0..=3 {
            let response = target
                .process(DfuRequest::Write { data: &[0xaa; 2] }, &mut test_flash)
                .await;
            assert_eq!(target.receipt_count, i + 1);
            assert_eq!(response.0.body.is_none(), true);
        }

        // 5th write should trigger the CrC response with body
        let response = target
            .process(DfuRequest::Write { data: &[0xbb; 2] }, &mut test_flash)
            .await;
        if let Some(DfuResponseBody::Crc { offset, crc }) = response.0.body {
            assert_eq!(offset, 10);
            assert_eq!(crc, 0xDB870467);
        } else {
            panic!("Unexpected DFU response body: {:?}", body);
        };
        assert_eq!(target.receipt_count, 0);

        // Final write
        let response = target
            .process(DfuRequest::Write { data: &[0xaa; 2] }, &mut test_flash)
            .await;
        assert_eq!(target.receipt_count, 1);
        assert_eq!(response.0.body.is_none(), true);

        /*
        REQUEST Crc
        RESPONSE DfuResponse { request: Crc, result: Success, body: Some(Crc { offset: 12, crc: }) }
        */
        let response = target.process(DfuRequest::Crc {}, &mut test_flash).await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::InProgress, response.1);
        if let Some(DfuResponseBody::Crc { offset, crc }) = response.0.body {
            assert_eq!(target.objects[target.current].offset, offset);
            assert_eq!(target.objects[target.current].size, offset);
            assert_eq!(target.objects[target.current].crc.finish(), crc);
        } else {
            panic!("Unexpected DFU response body: {:?}", body);
        };

        /*
        REQUEST Execute
        RESPONSE DfuResponse { request: Execute, result: Success, body: None }
        */
        let response = target.process(DfuRequest::Execute, &mut test_flash).await;
        assert_eq!(DfuResult::Success, response.0.result);
        assert_eq!(DfuStatus::DoneReset, response.1);
        assert_eq!(
            &test_flash.mem[0..target.objects[target.current].size as usize],
            &[0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xbb, 0xbb, 0xaa, 0xaa]
        );
        let magic = &[
            0xf3, 0x95, 0xc2, 0x77, 0x7f, 0xef, 0xd2, 0x60, 0x0f, 0x50, 0x52, 0x35, 0x80, 0x79,
            0xb6, 0x2c,
        ];
        let sz = target.objects[target.current].size as usize;
        assert_eq!(&test_flash.mem[sz..sz + magic.len()], magic,);
    }

    ///
    /// In-memory flash implementation taken from embassy-boot
    ///
    use alloc::vec::Vec;

    use embedded_storage_async::nor_flash::{ErrorType, NorFlash, ReadNorFlash};

    extern crate alloc;

    pub(crate) struct MemFlash<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> {
        pub mem: [u8; SIZE],
        pub writes: Vec<(u32, usize)>,
        pub erases: Vec<(u32, u32)>,
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize>
        MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        #[allow(unused)]
        pub const fn new(fill: u8) -> Self {
            Self {
                mem: [fill; SIZE],
                writes: Vec::new(),
                erases: Vec::new(),
            }
        }

        fn read(&mut self, offset: u32, bytes: &mut [u8]) {
            let len = bytes.len();
            bytes.copy_from_slice(&self.mem[offset as usize..offset as usize + len]);
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) {
            self.writes.push((offset, bytes.len()));
            let offset = offset as usize;
            assert_eq!(0, bytes.len() % WRITE_SIZE);
            assert_eq!(0, offset % WRITE_SIZE);
            assert!(offset + bytes.len() <= SIZE);

            self.mem[offset..offset + bytes.len()].copy_from_slice(bytes);
        }

        fn erase(&mut self, from: u32, to: u32) {
            self.erases.push((from, to));
            let from = from as usize;
            let to = to as usize;
            assert_eq!(0, from % ERASE_SIZE);
            assert_eq!(0, to % ERASE_SIZE);
            self.mem[from..to].fill(0xff);
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> Default
        for MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        fn default() -> Self {
            Self::new(0xff)
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> ErrorType
        for MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        type Error = core::convert::Infallible;
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> ReadNorFlash
        for MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        const READ_SIZE: usize = 1;

        async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            self.read(offset, bytes);
            Ok(())
        }

        fn capacity(&self) -> usize {
            SIZE
        }
    }

    impl<const SIZE: usize, const ERASE_SIZE: usize, const WRITE_SIZE: usize> NorFlash
        for MemFlash<SIZE, ERASE_SIZE, WRITE_SIZE>
    {
        const WRITE_SIZE: usize = WRITE_SIZE;
        const ERASE_SIZE: usize = ERASE_SIZE;

        async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            self.write(offset, bytes);
            Ok(())
        }

        async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.erase(from, to);
            Ok(())
        }
    }
}
