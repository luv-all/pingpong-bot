#[cfg(feature = "real")]
pub(super) struct RealBackend {
    protocol: rustypot::DynamixelProtocolHandler,
    port: Box<dyn serialport::SerialPort>,
}

#[cfg(feature = "real")]
impl RealBackend {
    pub(super) fn new(
        protocol: rustypot::DynamixelProtocolHandler,
        port: Box<dyn serialport::SerialPort>,
    ) -> Self {
        return Self { protocol, port };
    }

    pub(super) fn sync_write_with_retry(
        &mut self,
        ids: &[u8],
        address: u8,
        data: &[Vec<u8>],
        retries: u32,
        retry_delay_ms: u64,
    ) -> Result<(), String> {
        self.run_with_retry(retries, retry_delay_ms, "sync_write", |protocol, port| {
            protocol.sync_write(port, ids, address, data)
        })
    }

    pub(super) fn sync_read_with_retry(
        &mut self,
        ids: &[u8],
        address: u8,
        length: u8,
        retries: u32,
        retry_delay_ms: u64,
    ) -> Result<Vec<Vec<u8>>, String> {
        self.run_with_retry(retries, retry_delay_ms, "sync_read", |protocol, port| {
            protocol.sync_read(port, ids, address, length)
        })
    }

    pub(super) fn reboot_with_retry(
        &mut self,
        id: u8,
        retries: u32,
        retry_delay_ms: u64,
    ) -> Result<(), String> {
        self.run_with_retry(retries, retry_delay_ms, "reboot", |protocol, port| {
            protocol.reboot(port, id).map(|_| ())
        })
    }

    fn run_with_retry<T>(
        &mut self,
        retries: u32,
        retry_delay_ms: u64,
        op: &str,
        mut operation: impl FnMut(
            &rustypot::DynamixelProtocolHandler,
            &mut dyn serialport::SerialPort,
        ) -> Result<T, Box<dyn std::error::Error>>,
    ) -> Result<T, String> {
        let attempts = retries.max(1);
        let mut last_error = format!("{op}: no attempts");
        for attempt in 0..attempts {
            match operation(&self.protocol, self.port.as_mut()) {
                Ok(value) => return Ok(value),
                Err(error) if attempt + 1 < attempts => {
                    tracing::debug!(
                        op,
                        attempt = attempt + 1,
                        attempts,
                        error = %error,
                        "Dynamixel 통신 재시도"
                    );
                    last_error = error.to_string();
                    let _ = self.port.clear(serialport::ClearBuffer::All);
                    std::thread::sleep(std::time::Duration::from_millis(retry_delay_ms));
                }
                Err(error) => {
                    tracing::debug!(
                        op,
                        attempt = attempt + 1,
                        attempts,
                        error = %error,
                        "Dynamixel 통신 최종 실패"
                    );
                    return Err(error.to_string());
                }
            }
        }
        return Err(last_error);
    }
}
