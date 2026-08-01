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

    /// 한 모터만 읽는다. 여러 모터의 Status Packet이 연달아 들어오는
    /// `sync_read`에서 체크섬 오류가 나는 저속(57,600 baud) 버스용 경로다.
    pub(super) fn read_with_retry(
        &mut self,
        id: u8,
        address: u8,
        length: u8,
        retries: u32,
        retry_delay_ms: u64,
    ) -> Result<Vec<u8>, String> {
        self.run_with_retry(retries, retry_delay_ms, "read", |protocol, port| {
            protocol.read(port, id, address, length)
        })
        .map_err(|error| format!("id={id}: {error}"))
    }

    /// 각 모터를 순서대로 개별 읽기한다.
    pub(super) fn read_many_with_retry(
        &mut self,
        ids: &[u8],
        address: u8,
        length: u8,
        retries: u32,
        retry_delay_ms: u64,
    ) -> Result<Vec<Vec<u8>>, String> {
        return ids
            .iter()
            .map(|&id| self.read_with_retry(id, address, length, retries, retry_delay_ms))
            .collect();
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
