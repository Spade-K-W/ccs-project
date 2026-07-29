#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
泰山派 SPI 主机：与 TI MSPM0 从机握手并接收状态帧。

协议：
  握手中：
    - 主机 MOSI 前 2 字节循环发 b'ok'，其后填 0x00
    - 从机 MISO 应出现 ASCII 帧 Mode:N（N=2..5）
    - 主机看到 Mode:N 后置 handshake_done
  握手后：
    - 主机 MOSI 全 0x00（仅提供时钟）
    - 从机 MISO 发 sState:0|1,Angle:±x.xt

接线（默认）：
  泰山派 CLK → TI PB18
  泰山派 MOSI → TI PA14
  泰山派 MISO ← TI PA13
  共地
"""

from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import spidev
except ImportError:  # 允许在非板端做语法检查
    spidev = None  # type: ignore


@dataclass
class VisionData:
    state: int = 0
    angle_deg: float = 0.0
    is_valid: bool = False
    raw: str = ""


class SpiMasterReceiver:
    def __init__(
        self,
        bus: int = 3,
        chip: int = 0,
        speed: int = 1_000_000,
        xfer_len: int = 48,
        period_s: float = 0.05,
        diagnose: bool = False,
        handshake_mode: int = 2,
        handshake_timeout: float = 10.0,
        mode: int = 0,  # SPI Mode0
    ) -> None:
        self.bus = bus
        self.chip = chip
        self.speed = speed
        self.xfer_len = max(8, int(xfer_len))
        self.period_s = period_s
        self.diagnose = diagnose
        self.handshake_mode = int(handshake_mode)
        self.handshake_timeout = float(handshake_timeout)
        self.handshake_timeout_s = self.handshake_timeout
        self.spi_mode = mode

        self._spi = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.lock = threading.Lock()

        self.handshake_done = False
        self.send_ok_count = 0
        self.frame_count = 0
        self.error_count = 0
        self._data = VisionData()
        self._last_miso_ascii = ""

        self._re_mode = re.compile(rb"Mode:([2-5])")
        self._re_state = re.compile(
            rb"sState:([01]),Angle:([+\-]?\d+(?:\.\d+)?)t"
        )

    def start(self) -> bool:
        if spidev is None:
            print("[错误] 未安装 spidev，请在泰山派上: pip3 install spidev")
            return False
        try:
            spi = spidev.SpiDev()
            spi.open(self.bus, self.chip)
            spi.max_speed_hz = self.speed
            spi.mode = self.spi_mode
            spi.bits_per_word = 8
            # 三线从机无 CS 时，主机仍可能拉 CS；保持默认即可
            try:
                spi.no_cs = False
            except Exception:
                pass
            self._spi = spi
        except Exception as e:
            print(f"[错误] 打开 SPI {self.bus}.{self.chip} 失败: {e}")
            return False

        self._stop.clear()
        self.handshake_done = False
        self.send_ok_count = 0
        self.frame_count = 0
        self.error_count = 0
        self._thread = threading.Thread(target=self._worker, name="spi-vision", daemon=True)
        self._thread.start()
        print(
            f"[SPI] 已启动 bus={self.bus}.{self.chip} speed={self.speed} "
            f"len={self.xfer_len} handshake=Mode:{self.handshake_mode}"
        )
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._spi is not None:
            try:
                self._spi.close()
            except Exception:
                pass
            self._spi = None

    def wait_handshake_done(self, timeout_s: Optional[float] = None) -> bool:
        limit = self.handshake_timeout_s if timeout_s is None else float(timeout_s)
        t0 = time.time()
        while time.time() - t0 < limit:
            with self.lock:
                done = self.handshake_done
            if done:
                return True
            time.sleep(0.05)
        return False

    def get_data(self) -> VisionData:
        with self.lock:
            return VisionData(
                state=self._data.state,
                angle_deg=self._data.angle_deg,
                is_valid=self._data.is_valid,
                raw=self._data.raw,
            )

    def print_stats(self) -> None:
        with self.lock:
            print(
                f"[统计] handshake_done={self.handshake_done} "
                f"send_ok={self.send_ok_count} frames={self.frame_count} "
                f"errors={self.error_count} last={self._last_miso_ascii!r}"
            )

    def _build_mosi(self, send_ok: bool) -> list[int]:
        buf = [0x00] * self.xfer_len
        if send_ok:
            for i in range(0, self.xfer_len, 2):
                buf[i] = ord("o")
                if i + 1 < self.xfer_len:
                    buf[i + 1] = ord("k")
        return buf

    def _xfer(self, mosi: list[int]) -> bytes:
        assert self._spi is not None
        miso = self._spi.xfer2(mosi)
        return bytes(miso)

    def _parse_miso(self, raw: bytes) -> None:
        trimmed = raw.rstrip(b"\x00\xff")
        ascii_view = "".join(chr(b) if 32 <= b < 127 else "." for b in trimmed)
        self._last_miso_ascii = ascii_view

        if self.diagnose:
            hx = " ".join(f"{b:02X}" for b in raw[: min(24, len(raw))])
            print(
                f"[DIAG] send_ok={'Y' if not self.handshake_done else 'N'} "
                f"MISO[{len(raw)}]: {hx} | '{ascii_view}'"
            )

        if not self.handshake_done:
            m = self._re_mode.search(raw)
            if m is not None:
                got = int(m.group(1))
                if got == self.handshake_mode:
                    self.handshake_done = True
                    print(f"[SPI] 收到 Mode:{got}，握手完成，停止发送 ok")
                elif self.diagnose:
                    print(f"[DIAG] 看到 Mode:{got}，期望 Mode:{self.handshake_mode}")
            return

        m = self._re_state.search(raw)
        if m is None:
            m = self._re_state.search(trimmed)
        if m is None:
            if trimmed:
                self.error_count += 1
            return

        state = int(m.group(1))
        angle = float(m.group(2))
        self._data = VisionData(
            state=state,
            angle_deg=angle,
            is_valid=True,
            raw=m.group(0).decode("ascii", errors="ignore"),
        )
        self.frame_count += 1

    def _worker(self) -> None:
        assert self._spi is not None
        while not self._stop.is_set():
            with self.lock:
                send_ok = not self.handshake_done
            try:
                mosi = self._build_mosi(send_ok)
                miso = self._xfer(mosi)
                with self.lock:
                    if send_ok:
                        self.send_ok_count += 1
                    self._parse_miso(miso)
            except Exception as e:
                with self.lock:
                    self.error_count += 1
                if self.diagnose:
                    print(f"[DIAG] xfer 异常: {e}")
            time.sleep(self.period_s)
