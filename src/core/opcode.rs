use super::cpu::{AddressingMode, InstructionType};

#[derive(Clone, Copy)]
pub struct Opcode {
    pub bytes: usize,
    pub cycles: usize,

    pub instruction_type: InstructionType,
    pub mode: AddressingMode,
}