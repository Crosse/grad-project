#[derive(Debug)]
pub enum MessageType {
    Unknown,
    Response,
    TimeSync,
    Stats,
}

impl From<u8> for MessageType {
    fn from(val: u8) -> Self {
        match val {
            0 => MessageType::Unknown,
            1 => MessageType::Response,
            2 => MessageType::TimeSync,
            3 => MessageType::Stats,
            _ => panic!("invalid value for message type: {}", val),
        }
    }
}

#[derive(Debug)]
#[repr(C)]
pub struct MessageHeader {
    pub message_type: MessageType,
    pub len: u16,
    pub crc: u16,
}

#[derive(Debug)]
#[repr(C)]
pub enum InverterStatus {
    Unused,
    Off,
    Sleeping,
    Starting,
    MPPT,
    Throttled,
    ShuttingDown,
    Fault,
    Standby,
}
