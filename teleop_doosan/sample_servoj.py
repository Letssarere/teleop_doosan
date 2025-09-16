#!/usr/bin/env python3
import math
import time
import rclpy

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main():
    rclpy.init()
    node = rclpy.create_node("sample_servoj_100hz", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        get_current_posj,
        servoj,
        # 필요 상수
        DR_SERVO_OVERRIDE,
    )

    # 제어 파라미터
    freq = 100.0      # 목표 주파수 (Hz)
    dt = 1.0 / freq   # 0.01 s
    run_time = 10.0   # 동작 시간 (초)
    amp_deg = 5.0     # J0 관절 사인파 진폭 (deg)
    sine_freq = 0.3   # 사인파 주파수 (Hz)

    # 속도/가속도 제한 (관절 6개 동일)
    vel = [60.0] * 6      # deg/s
    acc = [300.0] * 6     # deg/s^2

    # 시작 관절 위치
    pos0 = get_current_posj()
    if pos0 is None:
        node.get_logger().error("현재 관절을 읽지 못했습니다.")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("Start servoj streaming (100 Hz) ...")

    t_start = time.perf_counter()
    next_tick = t_start
    count = 0

    try:
        while rclpy.ok():
            now = time.perf_counter()
            elapsed = now - t_start
            if elapsed >= run_time:
                break

            phase = 2.0 * math.pi * sine_freq * elapsed

            target = list(pos0)
            target[0] = pos0[0] + amp_deg * math.sin(phase)

            # servoj: pos, vel, acc, time(=dt), mode=OVERRIDE
            servoj(target, vel=vel, acc=acc, t=dt, mode=DR_SERVO_OVERRIDE)

            # 콜백 처리
            rclpy.spin_once(node, timeout_sec=0.0)

            # 주기 유지 (단순 방식)
            next_tick += dt
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 주기 초과(지터) 시 다음 틱 재계산(드리프트 축소)
                next_tick = time.perf_counter()

            count += 1

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")

    # 마지막 위치(현재 사인파 끝 위치) 한 번 더 보내 안정화
    final_pos = get_current_posj()
    if final_pos:
        servoj(final_pos, vel=vel, acc=acc, t=0.05, mode=DR_SERVO_OVERRIDE)

    node.get_logger().info(f"Done. Sent {count} servo samples.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()