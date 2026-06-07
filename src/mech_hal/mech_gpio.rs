
#[derive(Debug, Clone)]
pub enum GPIOError{
    InvalidPin,
    InvalidFunction,
    Generic
}

pub enum MechGPIOFunc {
    IO,
    I2C,
    UART, 
    SPI,
    PWM
}

pub trait MechGPIO {
    fn set_output(&mut self, num: u8, is_output: bool) -> Result<(), GPIOError>;
    fn set_level(&mut self, num: u8, is_high: bool) -> Result<(), GPIOError>;
    fn get_level(&mut self, num: u8) -> Result<bool, GPIOError>;
    fn set_function(&mut self, num: u8, function: MechGPIOFunc) -> Result<(), GPIOError>;
}