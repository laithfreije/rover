
#[derive(Debug, Clone)]
pub enum I2CError{
    BusError
}

pub trait MechI2C {
    fn read(self, buf: &mut[u8], num: u8) -> Result<(), I2CError>;
    fn write(self, buf: &mut[u8], num: u8) -> Result<(), I2CError>;
}