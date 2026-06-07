
#[derive(Debug, Clone)]
pub enum I2CError{
    BusError
}

pub trait MechI2C {
    fn read(&mut self, buf: &mut[u8], num: u8) -> Result<(), I2CError>;
    fn write(&mut self, buf: &mut[u8], num: u8) -> Result<(), I2CError>;
}