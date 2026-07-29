#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
握手测试脚本
------------
调用 spi_master.py 中的 SpiMasterReceiver，测试泰山派(主机) 与
TI MSPM0(从机) 之间的握手流程：

  1. 主机以 MOSI 前2字节循环发送 'ok'，等待从机确认收到 Mode:N 帧后切换状态
  2. 从机收到 'ok' 后应停止发送 Mode:N，转为发送常态帧 sState:x,Angle:y
  3. 本脚本负责：
       - 启动接收线程（内部会自动按 handshake_mode 发送 'ok'）
       - 轮询等待 handshake_done 置位
       - 握手成功后，再连续采集若干秒数据，验证帧是否正常
       - 打印详细的握手耗时 / 统计信息 / 失败原因

用法示例：
  sudo python3 connect_signal.py --bus 3 --speed 1000000 --handshake 2 --diagnose
  sudo python3 connect_signal.py --handshake 3 --timeout 15 --observe 5

注意：
  同目录需要 spi_master.py（本仓库已提供）。
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    from spi_master import SpiMasterReceiver
except ImportError:
    print("[错误] 找不到 spi_master.py，请确保它与本脚本在同一目录下")
    sys.exit(1)


def _patch_missing_attrs(recv: SpiMasterReceiver) -> None:
    """
    兜底修补 spi_master.py 中命名不一致导致的缺失属性，
    避免握手线程运行时因 AttributeError 崩溃。
    建议同步修正源文件，这里只是让测试能跑起来。
    """
    if not hasattr(recv, "send_ok_count"):
        recv.send_ok_count = 0
    if not hasattr(recv, "handshake_timeout_s"):
        recv.handshake_timeout_s = getattr(recv, "handshake_timeout", 5.0)


def run_handshake_test(
    bus: int,
    chip: int,
    speed: int,
    xfer_len: int,
    period: float,
    mode: int,
    timeout_s: float,
    observe_s: float,
    diagnose: bool,
) -> bool:
    print("=" * 60)
    print(f" SPI 握手测试  Mode:{mode}  bus={bus}.{chip}  speed={speed}Hz")
    print("=" * 60)

    recv = SpiMasterReceiver(
        bus=bus,
        chip=chip,
        speed=speed,
        xfer_len=xfer_len,
        period_s=period,
        diagnose=diagnose,
        handshake_mode=mode,
    )

    # 修补潜在的属性缺失问题（见文件头说明）
    _patch_missing_attrs(recv)

    if not recv.start():
        print("[测试] SPI 打开失败，测试终止")
        return False

    # 再次修补，防止 start() 内部逻辑覆盖后又缺失
    _patch_missing_attrs(recv)

    t0 = time.time()
    ok = recv.wait_handshake_done(timeout_s=timeout_s)
    elapsed = time.time() - t0

    if not ok:
        print(f"[测试] 握手失败！耗时 {elapsed:.2f}s 仍未收到有效 Mode:{mode} 响应")
        print("[排查建议]")
        print("  1) 检查接线：泰山派 CLK→PB18, MOSI→PA14, MISO←PA13, 共地")
        print("  2) 确认从机固件里此刻确实处于等待 Mode:N 握手的状态")
        print("  3) 用 --diagnose 参数查看原始收发字节，确认从机是否有回包")
        print("  4) 确认 SPI 模式(Mode0)、速率、字长(8bit)、位序(MSB)双方一致")
        recv.stop()
        return False

    print(f"[测试] 握手成功！耗时 {elapsed:.2f}s")
    print(f"[测试] 进入常态数据观察阶段，持续 {observe_s:.1f}s ...")

    t_obs = time.time()
    last_frame_count = 0
    while time.time() - t_obs < observe_s:
        time.sleep(0.5)
        d = recv.get_data()
        with recv.lock:
            fc = recv.frame_count
            errs = recv.error_count
        delta = fc - last_frame_count
        last_frame_count = fc
        mode_str = "直行" if d.state == 0 else "转弯"
        print(
            f"  [观察] 累计帧数={fc} (+{delta}) 错误帧={errs} "
            f"状态={d.state}({mode_str}) 角度={d.angle_deg:+.1f}° "
            f"有效={'是' if d.is_valid else '否'}"
        )

    recv.print_stats()
    recv.stop()

    if last_frame_count == 0:
        print("[测试] 警告：握手成功但握手后未收到任何有效常态帧，"
              "请检查从机是否已切换到数据发送模式")
        return False

    print("[测试] 握手 + 数据接收 全部通过 ✅")
    return True


def main() -> None:
    ap = argparse.ArgumentParser(description="SPI 握手测试")
    ap.add_argument("--bus", type=int, default=3, help="SPI 总线号")
    ap.add_argument("--chip", type=int, default=0, help="SPI 片选号")
    ap.add_argument("--speed", type=int, default=1_000_000, help="SPI 速率(Hz)")
    ap.add_argument("--len", type=int, default=48, dest="xfer_len", help="每次传输字节数")
    ap.add_argument("--period", type=float, default=0.05, help="传输间隔(s)")
    ap.add_argument("--handshake", type=int, default=2, choices=[2, 3, 4, 5],
                     help="握手项目号 Mode:N，取值 2~5")
    ap.add_argument("--timeout", type=float, default=10.0, help="握手超时时间(s)")
    ap.add_argument("--observe", type=float, default=5.0, help="握手成功后观察数据的时长(s)")
    ap.add_argument("--diagnose", action="store_true", help="打印原始收发字节，便于排查")
    args = ap.parse_args()

    success = run_handshake_test(
        bus=args.bus,
        chip=args.chip,
        speed=args.speed,
        xfer_len=args.xfer_len,
        period=args.period,
        mode=args.handshake,
        timeout_s=args.timeout,
        observe_s=args.observe,
        diagnose=args.diagnose,
    )

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()