
use std::ops::Shl;

use super::{bus::Bus, opcode::Opcode};

const FLAG_CARRY:       u8 = 0b0000_0001;
const FLAG_ZERO:        u8 = 0b0000_0010;
const FLAG_INTERRUPT:   u8 = 0b0000_0100;
const FLAG_DECIMAL:     u8 = 0b0000_1000;
const FLAG_BREAK:       u8 = 0b0001_0000;
const FLAG_OVERFLOW:    u8 = 0b0100_0000;
const FLAG_NEG:         u8 = 0b1000_0000;

const RESET_VECTOR:     u16 = 0xFFFC;

include!(concat!(env!("OUT_DIR"), "/opcodes.rs"));

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    pub reg_x: u8,
    pub reg_y: u8,
    pub acc: u8,
    pub program_counter: u16,
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
            stack_pointer: 0xFF,
            status: 0,
            bus: Bus::new()
        }
    }

    pub fn reset(&mut self) {
        self.reg_x = 0;
        self.reg_y = 0;
        self.acc = 0;
        self.program_counter = self.bus.mem_read_u16(RESET_VECTOR);
        self.stack_pointer = 0xFF;

    }

    fn stack_push(&mut self, value: u8) {
        self.bus.memory[0x100 + self.stack_pointer as usize] = value;
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    fn stack_push_u16(&mut self, value: u16) {
        self.stack_push((value >> 8) as u8);
        self.stack_push(value as u8);
    }

    fn stack_pull(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        self.bus.memory[0x100 + self.stack_pointer as usize]
    }

    fn stack_pull_u16(&mut self) -> u16 {
        self.stack_pull() as u16 | (self.stack_pull() as u16) << 8
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
            let rel = self.bus.mem_read(self.program_counter + 1) as i8;
println!("{}, {}", rel, self.bus.mem_read(self.program_counter + 1));
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
            AddressingMode::Immediate => self.program_counter + 1,

            AddressingMode::Absolute => {
                self.bus.mem_read_u16(self.program_counter + 1).swap_bytes()
            }

            AddressingMode::ZeroPage => self.bus.mem_read(self.program_counter + 1) as u16,

            AddressingMode::ZeroPageX => {
                let zp_addr = self.bus.mem_read(self.program_counter + 1);
                let indexed_addr = zp_addr.wrapping_add(self.reg_x) as u16;

                indexed_addr
            },

            AddressingMode::ZeroPageY => {
                let zp_addr = self.bus.mem_read(self.program_counter + 1);
                let indexed_addr = zp_addr.wrapping_add(self.reg_y) as u16;

                indexed_addr
            },

            AddressingMode::Indirect => {
                let addr = self.bus.mem_read_u16(self.program_counter + 1);
                
                addr
            },

            AddressingMode::IndirectX => {
                let base = self.bus.mem_read(self.program_counter + 1);

                let ptr = base.wrapping_add(self.reg_x);
                
                self.bus.mem_read_u16(ptr as u16)
            },

            AddressingMode::IndirectY => {
                let base = self.bus.mem_read(self.program_counter + 1);

                let addr = self.bus.mem_read_u16(base as u16);
                
                addr.wrapping_add(self.reg_y as u16)
            }

            _ => unimplemented!("AddressingMode::{:?}", mode)
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

    fn eor(&mut self, mode: &AddressingMode) {
        let address = self.get_memory_address(mode);

        self.acc ^= self.bus.mem_read(address);
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

    fn lsr(&mut self, mode: &AddressingMode) {
        
        let (result, carry) = {
            if *mode == AddressingMode::Accumulator {
                let (result, carry) = self.acc.overflowing_shr(1);
                
                self.acc = result;

                (result, carry)
            } else {
                let address = self.get_memory_address(mode);
                let (result, carry) = self.bus.mem_read(address).overflowing_shr(1);

                self.bus.mem_write(address, result);

                (result, carry)
            }

        };

        self.set_flag_value(FLAG_CARRY, carry);
        self.update_zero_and_negative_flags(result);
    }

    fn rol(&mut self, mode: &AddressingMode) {
        
        let (result, carry) = {
            if *mode == AddressingMode::Accumulator {
                let (value, carry) = self.acc.overflowing_shl(1);
                
                self.acc = value | self.status & FLAG_CARRY;

                (self.acc, carry)
            } else {
                let address = self.get_memory_address(mode);
                let (value, carry) = self.bus.mem_read(address).overflowing_shr(1);

                let result = value | self.status & FLAG_CARRY;

                self.bus.mem_write(address, result);

                (result, carry)
            }

        };

        self.set_flag_value(FLAG_CARRY, carry);
        self.update_zero_and_negative_flags(result);
    }

    fn ror(&mut self, mode: &AddressingMode) {
        
        let (result, carry) = {
            if *mode == AddressingMode::Accumulator {
                let (value, carry) = self.acc.overflowing_shr(1);
                
                self.acc = value | (self.status & FLAG_CARRY) << 7;

                (self.acc, carry)
            } else {
                let address = self.get_memory_address(mode);
                let (value, carry) = self.bus.mem_read(address).overflowing_shr(1);

                let result = value | (self.status & FLAG_CARRY) << 7;

                self.bus.mem_write(address, result);

                (result, carry)
            }

        };

        self.set_flag_value(FLAG_CARRY, carry);
        self.update_zero_and_negative_flags(result);
    }


    fn cmp(&mut self, mode: &AddressingMode, value: u8) {
        let address = self.get_memory_address(mode);
        let operand = self.bus.mem_read(address);
        self.set_flag_value(FLAG_CARRY, value >= operand);

        let result = value.wrapping_sub(operand);

        self.update_zero_and_negative_flags(result);
    }


    pub fn load_program(&mut self, code: Vec<u8>) {
        for (idx, byte) in code.iter().enumerate() {
            self.bus.mem_write(idx as u16, *byte);
        }

        self.reset();
    }

    pub fn execute_opcode(&mut self) {
        let opcode = self.bus.mem_read(self.program_counter);
        let instruction: Opcode = Instructions[opcode as usize].expect(&format!("Unexpected opcode: {}", opcode));
        let mode = instruction.mode;
        let bytes = instruction.bytes as u16;
        let instr_type = instruction.instruction_type;

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

            InstructionType::EOR => {
                self.eor(&mode);
                self.program_counter += bytes;
            }

            InstructionType::AND => {
                self.and(&mode);
                self.program_counter += bytes;
            }

            // region: Shift & Rotate instructions

            InstructionType::ASL => {
                self.asl(&mode);
                self.program_counter += bytes;
            }

            InstructionType::LSR => {
                self.lsr(&mode);
                self.program_counter += bytes;
            }

            InstructionType::ROL => {
                self.rol(&mode);
                self.program_counter += bytes;
            }

            InstructionType::ROR => {
                self.ror(&mode);
                self.program_counter += bytes;
            }

            // endregion
            
            // region: Flag instructions

            InstructionType::CLC => {
                self.clear_flag(FLAG_CARRY);
                self.program_counter += bytes;
            }

            InstructionType::SEC => {
                self.set_flag(FLAG_CARRY);
                self.program_counter += bytes;
            }

            InstructionType::CLD => {
                self.clear_flag(FLAG_DECIMAL);
                self.program_counter += bytes;
            }

            InstructionType::SED => {
                self.set_flag(FLAG_DECIMAL);
                self.program_counter += bytes;
            }

            InstructionType::CLI => {
                self.clear_flag(FLAG_INTERRUPT);
                self.program_counter += bytes;
            }

            InstructionType::SEI => {
                self.set_flag(FLAG_INTERRUPT);
                self.program_counter += bytes;
            }

            InstructionType::CLV => {
                self.clear_flag(FLAG_OVERFLOW);
                self.program_counter += bytes;
            }

            // endregion

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

            // region: Comparison instructions

            InstructionType::CMP => {
                self.cmp(&mode, self.acc);
                self.program_counter += bytes;
            }

            InstructionType::CPX => {
                self.cmp(&mode, self.reg_x);
                self.program_counter += bytes;
            }

            InstructionType::CPY => {
                self.cmp(&mode, self.reg_y);
                self.program_counter += bytes;
            }

            // endregion


            // region: Increment & Decrement instructions
            InstructionType::DEX => {
                self.reg_x = self.reg_x.wrapping_sub(1);

                self.update_zero_and_negative_flags(self.reg_x);
                self.program_counter += bytes;
            }

            InstructionType::INX => {
                self.reg_x = self.reg_x.wrapping_add(1);

                self.update_zero_and_negative_flags(self.reg_x);
                self.program_counter += bytes;
            }

            InstructionType::DEY => {
                self.reg_y = self.reg_y.wrapping_sub(1);

                self.update_zero_and_negative_flags(self.reg_y);
                self.program_counter += bytes;
            }

            InstructionType::INY => {
                self.reg_y = self.reg_y.wrapping_add(1);

                self.update_zero_and_negative_flags(self.reg_y);
                self.program_counter += bytes;
            }

            InstructionType::DEC => {
                if mode == AddressingMode::Accumulator {
                    self.acc = self.acc.wrapping_sub(1);

                    self.update_zero_and_negative_flags(self.acc);
                } else {
                    let address = self.get_memory_address(&mode);

                    let value = self.bus.mem_read(address).wrapping_sub(1);

                    self.bus.mem_write(address, value);

                    self.update_zero_and_negative_flags(value);
                }
                
                self.program_counter += bytes;
                
            }

            InstructionType::INC => {
                if mode == AddressingMode::Accumulator {
                    self.acc = self.acc.wrapping_add(1);

                    self.update_zero_and_negative_flags(self.acc);
                } else {
                    let address = self.get_memory_address(&mode);

                    let value = self.bus.mem_read(address).wrapping_add(1);

                    self.bus.mem_write(address, value);

                    self.update_zero_and_negative_flags(value);
                }
                
                self.program_counter += bytes;
                
            }

            // endregion

            // region: Branch instructions
            InstructionType::BNE => {
                
                self.branch(FLAG_ZERO, false);
                self.program_counter += bytes;
            }
            
            InstructionType::BEQ => {
                
                self.branch(FLAG_ZERO, true);
                self.program_counter += bytes;
            }

            InstructionType::BVC => {
                
                self.branch(FLAG_OVERFLOW, false);
                self.program_counter += bytes;
            }

            InstructionType::BVS => {
                
                self.branch(FLAG_OVERFLOW, true);
                self.program_counter += bytes;
            }

            InstructionType::BCC => {
                
                self.branch(FLAG_CARRY, false);
                self.program_counter += bytes;
            }

            InstructionType::BCS => {
                
                self.branch(FLAG_CARRY, true);
                self.program_counter += bytes;
            }

            InstructionType::BPL => {
                
                self.branch(FLAG_NEG, false);
                self.program_counter += bytes;
            }

            InstructionType::BMI => {
                
                self.branch(FLAG_NEG, true);
                self.program_counter += bytes;
            }
            // endregion

            // region: Register transfer instructions

            InstructionType::TAX => {
                self.reg_x = self.acc;
                self.update_zero_and_negative_flags(self.reg_x);
            }

            InstructionType::TAY => {
                self.reg_y = self.acc;
                self.update_zero_and_negative_flags(self.reg_y)
            }

            InstructionType::TSX => {
                self.reg_x = self.stack_pointer;
                self.update_zero_and_negative_flags(self.reg_x);
            }

            InstructionType::TXA => {
                self.acc = self.reg_x;
                self.update_zero_and_negative_flags(self.acc);
            }

            InstructionType::TXS => {
                self.stack_pointer = self.reg_x;
            }

            InstructionType::TYA => {
                self.acc = self.reg_y;
                self.update_zero_and_negative_flags(self.acc);
            }

            // endregion

            // region: Stack instructions

            InstructionType::PHA => {
                self.stack_push(self.acc);
                self.program_counter += bytes;
            }

            InstructionType::PLA => {
                self.acc = self.stack_pull();
                self.program_counter += bytes;
            }

            InstructionType::PHP => {
                self.stack_push(self.status);
                self.program_counter += bytes;
            }

            InstructionType::PLP => {
                self.status = self.stack_pull();
                self.program_counter += bytes;
            }

            // endregion

            InstructionType::JMP => {
                self.jmp(&mode);
            }

            InstructionType::JSR => {
                println!("{}", self.program_counter + bytes - 1);
                self.stack_push_u16(self.program_counter + bytes - 1);
                self.program_counter = self.get_memory_address(&mode);
            }

            InstructionType::RTS => {
                self.program_counter = self.stack_pull_u16() + 1;
            }

            InstructionType::NOP => self.program_counter += 1,

            //_ => unimplemented!("Opcode: 0x{:0>2X}", opcode)
        }
    }

    pub fn run<F>(&mut self, mut callback: F)
    where F: FnMut(&mut Cpu) {
        
        loop {
            callback(self);

            self.execute_opcode();
            
        }
    }
}