use crate::core::cpu::Cpu;

mod core;

fn main() {
    let mut cpu = Cpu::new();

    cpu.load_program(vec![0xA9, 0x10, 0xE9, 0x01, 0xD0, 0xFc]);
    cpu.run();

    println!("0x{:X}", cpu.acc);
    
    println!("Hello, world!");
}
