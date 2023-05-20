use std::io;

use crate::core::cpu::Cpu;

mod core;

fn main() {
    let mut cpu = Cpu::new();

    //cpu.load_program(vec![0xA9, 0x10, 0xE9, 0x01, 0xD0, 0xFc]);
    //cpu.load_program(vec![0xA2, 0x05, 0x69, 0x05, 0xCA, 0xD0, 0xFB]);
    //cpu.load_program(include_bytes!("../6502_functional_test.bin").to_vec());
    cpu.load_program(vec![
        0xa2, 0x00, 0xa0, 0x00, 0x8a, 0x99, 0x00, 0x02, 0x48, 0xe8, 0xc8, 0xc0, 0x10, 0xd0, 0xf5, 0x68,
        0x99, 0x00, 0x02, 0xC8, 0xC0, 0x20, 0xd0, 0xf7,
    ]);

    cpu.program_counter = 0;

    cpu.run(move |cpu| {
        println!("PC=x{:X}, Acc=x{:X}", cpu.program_counter, cpu.acc);
        //io::stdin().read_line(&mut answer);
        
    });
    
    let mut answer = String::new();
    println!("PC=0x{:X}, Acc=0x{:X}, X=0x{:X}, Y=0x{:X}", cpu.program_counter, cpu.acc, cpu.reg_x, cpu.reg_y);
    io::stdin().read_line(&mut answer);

    println!("Hello, world!");
}
