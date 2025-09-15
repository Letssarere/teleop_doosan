#!/usr/bin/env python3
import math
import time
import rclpy

# 로봇 식별자
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main():
    rclpy.init()
    node = rclpy.create_node("sample_joint_rt_400hz", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # DSR_ROBOT2 API import
    from DSR_ROBOT2 import (
        get_current_posj,
        servoj_rt,
        speedj_rt,
        connect_rt_control,
        start_rt_control,
        stop_rt_control,
        disconnect_rt_control,
    )

    # RT 연결 (필요 시만. 이미 연결되어 있을 수도 있음)
    if connect_rt_control() != 0:
        node.get_logger().error("connect_rt_control failed")
        return
    if start_rt_control() != 0:
        node.get_logger().error("start_rt_control failed")
        disconnect_rt_control()
        return

    # 제어 파라미터
    hz = 400.0
    dt = 1.0 / hz
    run_time = 5.0  # 데모 시간(초)
    use_servoj = False  # True: servoj_rt(위치 기반), False: speedj_rt(속도 기반)

    try:
        # 초기 상태
        pos0 = get_current_posj()  # posj 타입
        t0 = time.perf_counter()

        while rclpy.ok():
            now = time.perf_counter()
            elapsed = now - t0
            if elapsed >= run_time:
                break

            # 간단한 사인파 명령
            f = 0.5  # Hz
            amp_deg = 10.0  # 속도 모션일 때는 deg/s, 위치 모션일 때는 deg
            phase = 2.0 * math.pi * f * elapsed

            if use_servoj:
                # 위치 기반: J0 을 기준으로 작은 사인파 위치 목표
                target = list(pos0)
                target[0] = pos0[0] + amp_deg * math.sin(phase)
                # vel/acc는 양수 리스트(6원소). time은 주기.
                vel = [50.0] * 6
                acc = [200.0] * 6
                servoj_rt(target, vel=vel, acc=acc, t=dt)
            else:
                # 속도 기반: J0 속도에 사인파
                vel = [0.0] * 6
                vel[0] = amp_deg * math.sin(phase)  # deg/s
                acc = [300.0] * 6  # deg/s^2 (양수)
                speedj_rt(vel, acc, t=dt)

            # 이벤트 펌프 및 주기 유지
            rclpy.spin_once(node, timeout_sec=0.0)
            # 간단한 주기 유지(파이썬 타이밍 한계가 있으니 참고)
            sleep_to = now + dt
            remain = sleep_to - time.perf_counter()
            if remain > 0:
                time.sleep(remain)

        # 정지 명령(속도 0)
        speedj_rt([0.0] * 6, [300.0] * 6, t=dt)

    except KeyboardInterrupt:
        pass
    finally:
        # RT 해제
        try:
            stop_rt_control()
        finally:
            disconnect_rt_control()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()