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

    /// 예전 실기 코드가 사용하던 Group Sync Read 경로.
    ///
    /// 벤치에서 검증됐던 초기화 동작을 보존하기 위해 Present Position은 이 경로를
    /// 우선 사용하고, 실패할 때만 모터별 순차 읽기로 내려간다.
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

    /// Protocol 2.0 Status Packet의 헤더를 다시 동기화하며 한 모터를 읽는다.
    ///
    /// `rustypot`의 기본 reader는 입력의 현재 위치에서 헤더 크기만큼 바로 읽기 때문에
    /// USB-시리얼 버퍼에 깨진 바이트가 하나라도 앞에 남으면 이후 정상 패킷까지
    /// Parsing/Checksum 오류로 버릴 수 있다. 초기 토크 락은 현재 위치를 반드시 정확히
    /// 알아야 하므로, 여기서는 `FF FF FD 00`을 직접 찾고 CRC가 맞는 응답만 채택한다.
    pub(super) fn robust_read_with_retry(
        &mut self,
        id: u8,
        address: u8,
        length: u8,
        retries: u32,
        retry_delay_ms: u64,
    ) -> Result<Vec<u8>, String> {
        let attempts = retries.max(1);
        let mut last_error = "시도하지 않음".to_owned();
        for attempt in 0..attempts {
            match robust_protocol2_read(self.port.as_mut(), id, address, length) {
                Ok(value) => return Ok(value),
                Err(error) => {
                    last_error = error;
                    let _ = self.port.clear(serialport::ClearBuffer::Input);
                    if attempt + 1 < attempts {
                        tracing::debug!(
                            id,
                            address,
                            attempt = attempt + 1,
                            attempts,
                            error = %last_error,
                            "Dynamixel 헤더 재동기화 read 재시도"
                        );
                        std::thread::sleep(std::time::Duration::from_millis(retry_delay_ms));
                    }
                }
            }
        }
        return Err(format!("id={id}: {last_error}"));
    }

    pub(super) fn robust_read_many_with_retry(
        &mut self,
        ids: &[u8],
        address: u8,
        length: u8,
        retries: u32,
        retry_delay_ms: u64,
    ) -> Result<Vec<Vec<u8>>, String> {
        let mut values = Vec::with_capacity(ids.len());
        for &id in ids {
            values.push(self.robust_read_with_retry(
                id,
                address,
                length,
                retries,
                retry_delay_ms,
            )?);
        }
        return Ok(values);
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

#[cfg(feature = "real")]
fn robust_protocol2_read(
    port: &mut dyn serialport::SerialPort,
    id: u8,
    address: u8,
    length: u8,
) -> Result<Vec<u8>, String> {
    // 이전 명령의 늦은/깨진 응답을 이번 응답으로 오인하지 않는다.
    port.clear(serialport::ClearBuffer::Input)
        .map_err(|error| format!("입력 버퍼 초기화 실패: {error}"))?;

    // Protocol 2.0 READ: header + id + packet length(7) + instruction(0x02)
    // + address(u16) + read length(u16) + CRC16.
    let mut request = vec![
        0xFF, 0xFF, 0xFD, 0x00, id, 0x07, 0x00, 0x02, address, 0x00, length, 0x00,
    ];
    request.extend(protocol2_crc(&request).to_le_bytes());
    port.write_all(&request)
        .map_err(|error| format!("READ 패킷 전송 실패: {error}"))?;
    port.flush()
        .map_err(|error| format!("READ 패킷 flush 실패: {error}"))?;

    // 바이트 경계가 어긋나도 다음 정상 Status Packet 헤더를 다시 찾는다.
    const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
    let mut window = [0u8; 4];
    let mut received = 0usize;
    for _ in 0..512 {
        let mut byte = [0u8; 1];
        port.read_exact(&mut byte)
            .map_err(|error| format!("Status Packet 헤더 대기 실패: {error}"))?;
        if received < window.len() {
            window[received] = byte[0];
            received += 1;
        } else {
            window.rotate_left(1);
            window[3] = byte[0];
        }
        if received == window.len() && window == HEADER {
            break;
        }
    }
    if received != window.len() || window != HEADER {
        return Err("Status Packet 헤더를 찾지 못함".to_owned());
    }

    let mut prefix = [0u8; 3];
    port.read_exact(&mut prefix)
        .map_err(|error| format!("Status Packet 길이 읽기 실패: {error}"))?;
    let response_id = prefix[0];
    let packet_length = usize::from(u16::from_le_bytes([prefix[1], prefix[2]]));
    if !(4..=256).contains(&packet_length) {
        return Err(format!("비정상 Status Packet 길이: {packet_length}"));
    }

    let mut frame = Vec::with_capacity(7 + packet_length);
    frame.extend(HEADER);
    frame.extend(prefix);
    let mut remainder = vec![0u8; packet_length];
    port.read_exact(&mut remainder)
        .map_err(|error| format!("Status Packet 본문 읽기 실패: {error}"))?;
    frame.extend(remainder);

    let crc_index = frame.len() - 2;
    let received_crc = u16::from_le_bytes([frame[crc_index], frame[crc_index + 1]]);
    let calculated_crc = protocol2_crc(&frame[..crc_index]);
    if received_crc != calculated_crc {
        return Err(format!(
            "CRC 불일치: received=0x{received_crc:04X}, calculated=0x{calculated_crc:04X}"
        ));
    }
    if response_id != id {
        return Err(format!("응답 ID 불일치: 요청={id}, 응답={response_id}"));
    }

    let payload = protocol2_unstuff(&frame[7..crc_index]);
    if payload.len() < 2 || payload[0] != 0x55 {
        return Err("Status Packet instruction(0x55) 불일치".to_owned());
    }
    let params = &payload[2..]; // payload[1]은 모터의 Error 필드다.
    if params.len() != usize::from(length) {
        return Err(format!(
            "응답 데이터 길이 불일치: got {}, want {length}",
            params.len()
        ));
    }
    return Ok(params.to_vec());
}

#[cfg(feature = "real")]
fn protocol2_unstuff(bytes: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len());
    let mut index = 0usize;
    while index < bytes.len() {
        if index + 3 < bytes.len() && bytes[index..index + 4] == [0xFF, 0xFF, 0xFD, 0xFD] {
            out.extend([0xFF, 0xFF, 0xFD]);
            index += 4;
        } else {
            out.push(bytes[index]);
            index += 1;
        }
    }
    return out;
}

#[cfg(feature = "real")]
fn protocol2_crc(bytes: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x8005
            } else {
                crc << 1
            };
        }
    }
    return crc;
}

#[cfg(all(test, feature = "real"))]
mod tests {
    use super::{protocol2_crc, protocol2_unstuff};

    #[test]
    fn protocol2_crc_matches_robotis_example() {
        let packet_without_crc = [0xFF, 0xFF, 0xFD, 0x00, 0x2A, 0x03, 0x00, 0x01];
        assert_eq!(protocol2_crc(&packet_without_crc), 0xD216);
    }

    #[test]
    fn protocol2_unstuff_removes_only_inserted_fd() {
        assert_eq!(
            protocol2_unstuff(&[0x55, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x12]),
            [0x55, 0x00, 0xFF, 0xFF, 0xFD, 0x12]
        );
    }
}
