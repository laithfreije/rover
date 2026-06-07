
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
    fn set_output(self, num: u8, is_output: bool) -> Result<(), GPIOError>;
    fn set_level(self, num: u8, is_high: bool) -> Result<(), GPIOError>;
    fn get_level(self, num: u8) -> Result<bool, GPIOError>;
    fn set_function(self, num: u8, function: MechGPIOFunc) -> Result<(), GPIOError>;
}