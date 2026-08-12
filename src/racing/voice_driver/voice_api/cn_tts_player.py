"""CN-TTS UART 语音合成模块驱动

支持自定义文本朗读的 CN-TTS 语音模块（GBK 编码）
硬件接线：
  - 5V (红) -> RDK X5 Pin 2/4
  - GND (黑) -> RDK X5 Pin 6
  - RX (绿) -> RDK X5 Pin 8 (UART1_TX / /dev/ttyS1)
  - TX (黄) -> RDK X5 Pin 10 (UART1_RX，可选)
"""

from __future__ import annotations

import time
from typing import Any

try:
    import serial
except ImportError:
    serial = None


class CnTtsPlayer:
    """CN-TTS 语音合成模块驱动（GBK 编码）。"""

    MOTOR_PORTS = frozenset({'/dev/ttyACM0', '/dev/ttyACM1'})

    def __init__(
        self,
        *,
        port: str = '/dev/ttyS1',
        baudrate: int = 9600,
        logger: Any | None = None,
    ) -> None:
        """
        初始化 CN-TTS 驱动
        
        Args:
            port: 串口设备路径，默认 /dev/ttyS1 (RDK X5 UART1)
            baudrate: 波特率，默认 9600（CN-TTS 8N1）
            logger: ROS 2 logger 或 None
        """
        self._port = port
        self._baudrate = baudrate
        self._logger = logger

        if port in self.MOTOR_PORTS:
            self._log_error(
                f'{port} 是 OriginCar 电机 STM32 串口，请使用 TTS 模块的 UART（如 /dev/ttyS1）'
            )

    def speak_text(self, text: str) -> bool:
        """
        播报自定义文本（GBK 编码）
        
        Args:
            text: 要播报的中文/英文/数字文本
            
        Returns:
            bool: 播报是否成功
        """
        cleaned = text.strip()
        if not cleaned:
            self._log_error('CN-TTS 跳过：文本为空')
            return False

        # CN-TTS 模块限制建议不超过 200 字符
        original_len = len(cleaned)
        if len(cleaned) > 200:
            cleaned = cleaned[:200]
            self._log_warn(f'文本过长（{original_len} 字），截断至 200 字符')

        # 直接发送 GBK 编码文本
        try:
            gbk_bytes = cleaned.encode('gbk', errors='ignore')
            self._log_info(f'GBK 编码完成: {len(gbk_bytes)} 字节，内容="{cleaned}"')
        except Exception as e:
            self._log_error(f'GBK 编码失败: {e}')
            return False

        self._log_info(f'开始播报: 长度={len(cleaned)} 字，预计耗时={(len(cleaned) * 0.4):.1f}秒')
        ok = self._write_serial(gbk_bytes, label=f'文本播报 ({len(cleaned)} 字)')
        if ok:
            # 根据文本长度估算播报时间（中文约 0.4s/字，英文约 0.2s/字）
            estimated_time = min(10.0, len(cleaned) * 0.4)
            self._log_info(f'播报已发送，等待 {estimated_time:.1f} 秒')
            time.sleep(estimated_time)
            self._log_info('播报完成')
        else:
            self._log_error('播报发送失败')
        return ok

    def _write_serial(self, payload: bytes, *, label: str) -> bool:
        """写入串口数据"""
        if self._port in self.MOTOR_PORTS:
            self._log_error(f'禁止使用电机串口 {self._port} 进行 TTS 播报')
            return False

        if serial is None:
            self._log_error('pyserial 未安装，请执行: pip install pyserial')
            return False

        try:
            self._log_info(f'打开串口: {self._port}@{self._baudrate}')
            with serial.Serial(
                self._port,
                self._baudrate,
                timeout=1,
                write_timeout=2,
            ) as ser:
                ser.reset_input_buffer()
                self._log_info(f'写入数据: {len(payload)} 字节')
                ser.write(payload)
                ser.flush()
                self._log_info('数据发送完成，串口已关闭')

            hex_preview = payload[:50].hex(' ') if len(payload) <= 50 else payload[:50].hex(' ') + '...'
            self._log_info(f'CN-TTS {label} 发送至 {self._port}@{self._baudrate}: {hex_preview}')
            return True

        except Exception as exc:
            self._log_error(f'CN-TTS 串口写入失败 ({self._port}): {exc}')
            import traceback
            self._log_error(f'异常堆栈: {traceback.format_exc()}')
            return False

    def _log_info(self, message: str) -> None:
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(f'[CnTtsPlayer] {message}')

    def _log_warn(self, message: str) -> None:
        if self._logger is not None:
            self._logger.warn(message)
        else:
            print(f'[CnTtsPlayer] WARNING: {message}')

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[CnTtsPlayer] ERROR: {message}')
