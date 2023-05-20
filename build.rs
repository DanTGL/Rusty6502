use std::{env, path::Path, fs, collections::HashMap};

fn main() {
    let out_dir = env::var_os("OUT_DIR").unwrap();

    let dest_path = Path::new(&out_dir).join("opcodes.rs");

    let data = fs::read_to_string("6502_instructions.json").expect("Unable to read file");
    let json: serde_json::Value =
        serde_json::from_str(&data).expect("JSON was not well-formatted");
    
    if let serde_json::Value::Array(instructions) = json {

        let opcodes: HashMap<u8, String> = instructions.iter().map(|instr| {
            let opcode = {
                &u8::from_str_radix(&instr["opcode"].as_str().unwrap().replace("$", ""), 16).unwrap()
            };
            let opcode_str = {

            let bytes = &instr["bytes"].as_str().unwrap();
            let mode = &instr["mode"].as_str().unwrap().replace("(", "").replace(")", "").replace(",", "");
            let name = &instr["name"].as_str().unwrap();

            let s = (*opcode, format!("Some(Opcode {{instruction_type: InstructionType::{}, bytes: {}, cycles: 1, mode: AddressingMode::{}}})\n", &name, &bytes, &mode).to_string());

            s
            };
            
            opcode_str
        }
        ).collect();

        fs::write(
            &dest_path,
            format!("
            pub static Instructions: [Option<Opcode>; 0x100] = [
                                {}
                            ];",
                            (0..=0xFF).map(|idx| {
                                if let Some(opcode) = opcodes.get(&idx) {
                                    opcode
                                } else {
                                    "None"
                                }
                            }).collect::<Vec<&str>>().join(",")
                        )
        ).unwrap();
    }

    println!("cargo:rerun-if-changed=build.rs");
}