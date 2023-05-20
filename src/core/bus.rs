
pub struct Bus {
    pub(crate) memory: [u8; 0x10000],
}

impl Bus {
    pub fn new() -> Self {
        Self {
            memory: [0; 0x10000]
        }
    }

    pub fn mem_read(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    pub fn mem_read_u16(&self, address: u16) -> u16 {
        (self.mem_read(address) as u16) << 8 | self.mem_read(address.wrapping_add(1)) as u16
    }

    pub fn mem_write(&mut self, address: u16, value: u8) {
        self.memory[address as usize] = value;
    }
}