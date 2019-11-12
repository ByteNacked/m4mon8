/// Minimal and incomplete CDC-ACM implementation for the examples - this will eventually be a real
/// crate!
use usb_device::class_prelude::*;
use usb_device::Result;

pub struct MonitorDev<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
    data_read_ep: EndpointOut<'a, B>,
    data_write_ep: EndpointIn<'a, B>,
    vis_ep: EndpointIn<'a, B>,
    buf: [u8; 64],
    len: usize,
}

impl<B: UsbBus> MonitorDev<'_, B> {
    pub fn new(alloc: &UsbBusAllocator<B>) -> MonitorDev<'_, B> {
        MonitorDev {
            comm_if: alloc.interface(),
            read_ep: alloc.bulk(64),
            write_ep: alloc.bulk(64),
            data_read_ep: alloc.bulk(64),
            data_write_ep: alloc.bulk(64),
            vis_ep: alloc.bulk(64),
            buf: [0; 64],
            len: 0,
        }
    }

    pub fn write(&mut self, data: &[u8]) -> Result<usize> {
        match self.write_ep.write(data) {
            Ok(count) => Ok(count),
            Err(UsbError::WouldBlock) => Ok(0),
            e => e,
        }
    }

    pub fn read(&mut self, data: &mut [u8]) -> Result<usize> {
        self.len = match self.read_ep.read(&mut self.buf) {
            Ok(0) | Err(UsbError::WouldBlock) => return Ok(0),
            Ok(count) => count,
            e => return e,
        };

        let count = self.len;
        &data[..count].copy_from_slice(&self.buf[0..count]);

        Ok(count)
    }

    pub fn vis(&mut self, data: &[u8]) -> Result<usize> {
        match self.vis_ep.write(data) {
            Ok(count) => Ok(count),
            Err(UsbError::WouldBlock) => Ok(0),
            e => e,
        }
    }
}

impl<B: UsbBus> UsbClass<B> for MonitorDev<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        writer.interface(self.comm_if, 0xFF, 0x00, 0x00)?;

        writer.endpoint(&self.write_ep)?;
        writer.endpoint(&self.read_ep)?;
        writer.endpoint(&self.data_write_ep)?;
        writer.endpoint(&self.data_read_ep)?;
        writer.endpoint(&self.vis_ep)?;

        Ok(())
    }
}
