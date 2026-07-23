#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
泰山派 SPI 主机 ←→ TI MSPM0 从机

帧格式：
  常态：sState:0,Angle:+12.3t
        State: 0=直行，1=转弯
        Angle: 陀螺仪 Z 角（度）
  握手：TI 发 Mode:2|3|4|5 → 本机 MOSI 回小写 ok

接线：
  泰山派 CLK  → TI PB18
  泰山派 MOSI → TI PA14
  泰山派 MISO ← TI PA13
  GND 共地（无 CS）

用法：
  sudo python3 connect_signal.py --bus 3 --speed 1000000 --diagnose
"""

from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

FRAME_RE = re.compile(
    rb"sState:([01]),Angle:([+-]?\d+(?:\.\d+)?)t"
)
MODE_RE = re.compile(rb"Mode:([2-5])")


@dataclass
class SensorData:
    state: int = 0            # 0=直行 1=转弯
    angle_deg: float = 0.0
    timestamp: float = 0.0
    is_valid: bool = False


class SpiMasterReceiver:
    def __init__(
        self,
        bus: int = 3,
        chip: int = 0,
        speed: int = 1_000_000,
        xfer_len: int = 48,
        period_s: float = 0.05,
        diagnose: bool = False,
    ) -> None:
        self.bus = bus
        self.chip = chip
        self.speed = speed
        self.xfer_len = xfer_len
        self.period_s = period_s
        self.diagnose = diagnose
        self.spi = None
        self._run = False
        self._th: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.data = SensorData()
        self.rx_bytes = 0
        self.frame_count = 0
        self.error_count = 0
        self.rx_buffer = bytearray()
        self.mode: Optional[int] = None
        self._ack_pending = False
        self._ack_sent = False
        self._ack_rounds = 0

    def start(self) -> bool:
        try:
            import spidev
        except ImportError:
            print("[SPI] pip3 install spidev")
            return False

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.bus, self.chip)
            self.spi.max_speed_hz = self.speed
            self.spi.mode = 0
            self.spi.bits_per_word = 8
            self.spi.lsbfirst = False
            if hasattr(self.spi, "no_cs"):
                try:
                    self.spi.no_cs = True
                except Exception:
                    pass
        except Exception as e:
            print(f"[SPI] 打开 /dev/spidev{self.bus}.{self.chip} 失败: {e}")
            return False

        print(f"[SPI] 主机 /dev/spidev{self.bus}.{self.chip}")
        print(f"[SPI] Mode0 8bit MSB {self.speed}Hz xfer={self.xfer_len} 无CS")
        print("[SPI] 接线: CLK→PB18 MOSI→PA14 MISO←PA13 GND")
        print("[SPI] 期望帧: Mode:N 或 sState:0|1,Angle:+x.xt")

        self._run = True
        self._th = threading.Thread(target=self._worker, daemon=True)
        self._th.start()
        return True

    def _tx_bytes(self) -> List[int]:
        """握手确认：对 TI 发小写 ok，其余填 0。"""
        if self._ack_pending:
            payload = list(b"ok") + [0x00] * (self.xfer_len - 2)
            return payload[: self.xfer_len]
        return [0x00] * self.xfer_len

    def _parse_mode(self) -> None:
        m = MODE_RE.search(self.rx_buffer)
        if not m:
            return
        mode = int(m.group(1))
        # 消费到 Mode:N 末尾，避免重复触发
        end = m.end()
        del self.rx_buffer[:end]
        with self.lock:
            if self.mode != mode:
                self.mode = mode
                self._ack_pending = True
                self._ack_sent = False
                self._ack_rounds = 0
                print(f"[MODE] 收到 Mode:{mode}，准备回 ok")

    def _parse(self) -> None:
        self._parse_mode()
        while True:
            start = self.rx_buffer.find(b"sState:")
            if start < 0:
                if len(self.rx_buffer) > 24:
                    self.rx_buffer = self.rx_buffer[-24:]
                break
            if start > 0:
                del self.rx_buffer[:start]
            end = self.rx_buffer.find(b"t", 7)
            if end < 0:
                break
            frame = bytes(self.rx_buffer[: end + 1])
            del self.rx_buffer[: end + 1]
            m = FRAME_RE.fullmatch(frame)
            if not m:
                self.error_count += 1
                continue
            data = SensorData(
                state=int(m.group(1)),
                angle_deg=float(m.group(2)),
                timestamp=time.time() * 1000.0,
                is_valid=True,
            )
            with self.lock:
                self.data = data
                self.frame_count += 1
            mode = "直行" if data.state == 0 else "转弯"
            print(
                f"[RX] #{self.frame_count} "
                f"状态={data.state}({mode}) 角={data.angle_deg:+.1f}°"
            )

    def _worker(self) -> None:
        assert self.spi is not None
        t0 = time.time()
        while self._run:
            try:
                tx = self._tx_bytes()
                raw = self.spi.xfer2(tx)
                chunk = bytes(raw)
                self.rx_bytes += len(chunk)

                if self._ack_pending and not self._ack_sent:
                    self._ack_sent = True
                    self._ack_rounds = 0
                    print("[MODE] 已发送 ok")

                if self.diagnose and (time.time() - t0) < 8.0:
                    text = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
                    print(f"[RAW] {chunk[:48].hex(' ')}")
                    print(f"      {text[:48]!r}")

                if not all(b in (0x00, 0xFF) for b in chunk):
                    self.rx_buffer.extend(chunk)
                    if len(self.rx_buffer) > 4096:
                        self.rx_buffer = self.rx_buffer[-512:]
                    self._parse()

                # ok 连续发送约 0.5s 后停止，避免一直占 MOSI
                if self._ack_pending and self._ack_sent:
                    self._ack_rounds += 1
                    if self._ack_rounds >= 10:
                        self._ack_pending = False
                        self._ack_rounds = 0
                        print("[MODE] 握手发送结束，恢复空时钟")
            except Exception as e:
                print(f"[SPI] xfer 错: {e}")
                self.error_count += 1
                time.sleep(0.05)
                continue
            time.sleep(self.period_s)

    def stop(self) -> None:
        self._run = False
        if self._th:
            self._th.join(timeout=2.0)
        if self.spi:
            self.spi.close()

    def print_stats(self) -> None:
        with self.lock:
            d = self.data
            fc = self.frame_count
            mode_n = self.mode
        mode = "直行" if d.state == 0 else "转弯"
        mode_s = f"Mode={mode_n}" if mode_n is not None else "Mode=-"
        print(
            f"[统计] bytes={self.rx_bytes} frames={fc} "
            f"err={self.error_count} {mode_s} "
            f"状态={d.state}({mode}) 角={d.angle_deg:+.1f}"
        )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, default=3)
    ap.add_argument("--chip", type=int, default=0)
    ap.add_argument("--speed", type=int, default=1_000_000)
    ap.add_argument("--len", type=int, default=48, dest="xfer_len")
    ap.add_argument("--period", type=float, default=0.05)
    ap.add_argument("--diagnose", action="store_true")
    args = ap.parse_args()

    recv = SpiMasterReceiver(
        args.bus, args.chip, args.speed, args.xfer_len, args.period, args.diagnose
    )
    if not recv.start():
        sys.exit(1)
    try:
        while True:
            time.sleep(1.0)
            recv.print_stats()
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        recv.stop()


if __name__ == "__main__":
    main()
