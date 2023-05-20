
use std::ops::Shl;

use super::{bus::Bus, opcode::Opcode};

const FLAG_CARRY:       u8 = 0b0000_0001;
const FLAG_ZERO:        u8 = 0b0000_0010;
const FLAG_OVERFLOW:    u8 = 0b0100_0000;
const FLAG_NEG:         u8 = 0b1000_0000;

include!(concat!(env!("OUT_DIR"), "/opcodes.rs"));

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum InstructionType {
    ADC,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AddressingMode {
    Accumulator,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Immediate,
    Implied,
    Indirect,
    IndirectX,
    IndirectY,
    Relative,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
}

pub struct Cpu {
    reg_x: u8,
    reg_y: u8,
    pub acc: u8,
    program_counter: u16,
    stack_pointer: u8,
    status: u8,
    bus: Bus,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            reg_x: 0,
            reg_y: 0,
            acc: 0,
            program_counter: 0,
            stack_pointer: 0,
            status: 0,
            bus: Bus::new()
        }
    }

    fn branch(&mut self, flag: u8, branch_on_set: bool) {
        
        let result = {
            let flag_set = self.status & flag != 0;

            if branch_on_set {
                flag_set
            } else {
                !flag_set
            }
        };

        if result {
            let rel = self.bus.mem_read(self.program_counter) as i8;

            self.program_counter = self.program_counter.wrapping_add_signed(rel as i16);

        }
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        if result == 0 {
            self.status |= FLAG_ZERO;
        } else {
            self.status &= !FLAG_ZERO;
        }

        if result & FLAG_NEG != 0 {
            self.status |= FLAG_NEG;
        } else {
            self.status &= !FLAG_NEG;
        }
    }

    fn get_operand_u8(&self) -> u8 {
        return self.bus.mem_read(self.program_counter)
    }

    fn get_memory_address(&self, mode: &AddressingMode) -> u16 {
        match *mode {
            AddressingMode::Immediate => self.program_counter,

            AddressingMode::ZeroPage => self.bus.mem_read(self.program_counter) as u16,

            AddressingMode::ZeroPageX => {
                let zp_addr = self.bus.mem_read(self.program_counter);
                let indexed_addr = zp_addr.wrapping_add(self.reg_x) as u16;

                indexed_addr
            },

            AddressingMode::ZeroPageY => {
                let zp_addr = self.bus.mem_read(self.program_counter);
                let indexed_addr = zp_addr.wrapping_add(self.reg_y) as u16;

                indexed_addr
            },

            AddressingMode::Indirect => {
                let addr = self.bus.mem_read_u16(self.program_counter);
                
                addr
            },

            AddressingMode::IndirectX => {
                let base = self.bus.mem_read(self.program_counter);

                let ptr = base.wrapping_add(self.reg_x);
                
                self.bus.mem_read_u16(ptr as u16)
            },

            AddressingMode::IndirectY => {
                let base = self.bus.mem_read(self.program_counter);

                let addr = self.bus.mem_read_u16(base as u16);
                
                addr.wrapping_add(self.reg_y as u16)
            }

            _ => 0
        }
    }

    fn set_flag(&mut self, flag: u8) {
        self.status |= flag;
    }

    fn clear_flag(&mut self, flag: u8) {
        self.status &= !flag;
    }

    fn set_flag_value(&mut self, flag: u8, value: bool) {
        if value {
            self.set_flag(flag);
        } else {
            self.clear_flag(flag);
        }
    }

    fn jmp(&mut self, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        self.program_counter = address;
    }

    fn store(&mut self, value: u8, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        self.bus.mem_write(address, value);
    }

    fn load(&mut self, mode: &AddressingMode) -> u8 {
        let address = self.get_memory_address(mode);

        let result = self.bus.mem_read(address);
        self.update_zero_and_negative_flags(result);

        result
    }

    fn and(&mut self, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        self.acc &= self.bus.mem_read(address);
        self.update_zero_and_negative_flags(self.acc);
    }

    fn ora(&mut self, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        self.acc |= self.bus.mem_read(address);
        self.update_zero_and_negative_flags(self.acc);
    }

    fn sbc(&mut self, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        
        self.acc = self.acc.wrapping_add(self.bus.mem_read(address));

        self.update_zero_and_negative_flags(self.acc);
    }

    fn adc(&mut self, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        let (term, overflow) = self.bus.mem_read(address).overflowing_add(self.status & FLAG_CARRY);


        let result = {
            if self.status & FLAG_NEG != 0 {
                self.acc.overflowing_add_signed(term as i8)
            } else {
                self.acc.overflowing_add(term)
            }
        };
        
        self.acc = result.0;
        
        if result.1 | overflow {
            self.status |= FLAG_OVERFLOW;
        } else {
            self.status &= !FLAG_OVERFLOW;
        }

        self.update_zero_and_negative_flags(self.acc);
    }

    fn asl(&mut self, mode: &AddressingMode) {
        
        let (result, carry) = {
            if *mode == AddressingMode::Accumulator {
                let (result, carry) = self.acc.overflowing_shl(1);
                
                self.acc = result;

                (result, carry)
            } else {
                let address = self.get_memory_address(mode);
                let (result, carry) = self.bus.mem_read(address).overflowing_shl(1);

                self.bus.mem_write(address, result);

                (result, carry)
            }

        };

        self.set_flag_value(FLAG_CARRY, carry);
        self.update_zero_and_negative_flags(result);
    }
    

    pub fn load_program(&mut self, code: Vec<u8>) {
        for (idx, byte) in code.iter().enumerate() {
            self.bus.mem_write(idx as u16, *byte);
        }

        self.program_counter = 0;
    }

    pub fn run(&mut self) {
        
        loop {
            let opcode = self.bus.mem_read(self.program_counter);
            let instruction: Opcode = Instructions[opcode as usize].unwrap();
            let mode = instruction.mode;
            let bytes = (instruction.bytes - 1) as u16;
            let instr_type = instruction.instruction_type;
            println!("{:X}", opcode);
            self.program_counter += 1;

            match instr_type {
                InstructionType::SBC => {
                    self.sbc(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::ADC => {
                    self.adc(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::ORA => {
                    self.ora(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::AND => {
                    self.and(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::ASL => {
                    self.asl(&mode);
                    self.program_counter += bytes;
                }
                
                InstructionType::CLC => {
                    self.clear_flag(FLAG_CARRY);
                    self.program_counter += bytes;
                }

                InstructionType::SEC => {
                    self.set_flag(FLAG_CARRY);
                    self.program_counter += bytes;
                }

                InstructionType::STA => {
                    self.store(self.acc, &mode);
                    self.program_counter += bytes;
                }

                InstructionType::LDA => {
                    self.acc = self.load(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::STX => {
                    self.store(self.reg_x, &mode);
                    self.program_counter += bytes;
                }

                InstructionType::LDX => {
                    self.reg_x = self.load(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::STY => {
                    self.store(self.reg_y, &mode);
                    self.program_counter += bytes;
                }

                InstructionType::LDY => {
                    self.reg_y = self.load(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::DEX => {
                    self.reg_x = self.reg_x.wrapping_sub(1);

                    self.update_zero_and_negative_flags(self.reg_x);
                    self.program_counter += bytes;
                }

                
                InstructionType::BNE => {
                    
                    self.branch(FLAG_ZERO, false);
                    self.program_counter += bytes;
                }

                InstructionType::JMP => {
                    self.jmp(&mode);
                }

                InstructionType::NOP => continue,

                _ => break
            }
            
        }
    }
}