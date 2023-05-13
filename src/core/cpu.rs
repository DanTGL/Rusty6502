use std::ops::Add;

use super::{bus::Bus, opcode::Opcode};

const FLAG_ZERO:    u8 = 0b0000_0010;
const FLAG_NEG:     u8 = 0b1000_0000;
const FLAG_CARRY:   u8 = 0b0000_0001;

include!(concat!(env!("OUT_DIR"), "/opcodes.rs"));

#[derive(Clone, Copy)]
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

#[derive(Clone, Copy)]
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

    fn sta(&mut self, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        self.bus.mem_write(address, self.acc);
    }

    fn lda(&mut self, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        self.acc = self.bus.mem_read(address);
        self.update_zero_and_negative_flags(self.acc);
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


        self.acc = self.acc.wrapping_sub(self.bus.mem_read(address));

        self.update_zero_and_negative_flags(self.acc);
    }

    fn adc(&mut self, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        print!("{} ", self.acc);
        self.acc += self.bus.mem_read(address);
        println!("{}", self.acc);
        self.update_zero_and_negative_flags(self.acc);
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
                InstructionType::ORA => {
                    self.ora(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::AND => {
                    self.and(&mode);
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
                    self.sta(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::LDA => {
                    self.lda(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::SBC => {
                    self.sbc(&mode);
                    self.program_counter += bytes;
                }

                InstructionType::BNE => {
                    
                    self.branch(FLAG_ZERO, false);
                    self.program_counter += bytes;
                }

                InstructionType::NOP => continue,

                _ => break
            }
            
        }
    }
}