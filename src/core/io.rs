use core::fmt;



pub struct IO;

impl IO {
    pub fn print(args: fmt::Arguments) {
        print!("{}", args);
    }
}