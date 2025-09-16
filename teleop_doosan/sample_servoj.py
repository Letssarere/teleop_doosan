#!/usr/bin/env python3
import math
import time
import rclpy

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def wait_for_sub(node, topic, timeout=3.0):
    start = time.time()
    while time.time() - start < timeout and rclpy.ok():
        subs = node.count_subscribers(topic)
        if subs > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return False

def main():
    rclpy.init()
    node = rclpy.create_node("sample_servoj_stream", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        get_current_posj,
        get_robot_state,
        servoj,
        movej,
        set_velj,
        set_accj,
        DR_SERVO_OVERRIDE,
    )

    # 1. 초기 대기
    node.get_logger().info("Discovery 대기...")
    rclpy.spin_once(node, timeout_sec=0.3)

    # 2. 구독자 존재 확인 (참고용: 없어도 진행)
    if not wait_for_sub(node, "servoj_stream", timeout=2.0):
        node.get_logger().warn("servoj_stream 구독자 감지 못함. ros2 topic info /dsr01/servoj_stream 로 재확인 필요.")

    # 3. 로봇 상태
    try:
        st = get_robot_state()
        node.get_logger().info(f"robot_state(raw)={st}")
    except Exception as e:
        node.get_logger().warn(f"get_robot_state 실패: {e}")

    # 4. 속도/가속도 기본값
    try:
        set_velj(60)
        set_accj(300)
    except Exception as e:
        node.get_logger().warn(f"set_velj/set_accj 실패: {e}")

    # 5. 현재 관절
    pos0 = get_current_posj()
    if not pos0:
        node.get_logger().error("현재 관절 읽기 실패.")
        node.destroy_node(); rclpy.shutdown(); return
    node.get_logger().info(f"Init pos: {pos0}")

    # 6. (중요) movej 로 먼저 활성화 (INIT 상태라면)
    warm_target = list(pos0)
    warm_target[0] = pos0[0] + 5.0  # 5도 이동
    node.get_logger().info("워밍업 movej 실행 (스트리밍 적용 전 상태 활성화)")
    try:
        movej(warm_target, v=30, a=30, t=2.0)
        movej(pos0, v=30, a=30, t=2.0)
    except Exception as e:
        node.get_logger().warn(f"movej 워밍업 실패: {e}")

    # 7. 스트리밍 설정
    freq = 100.0
    dt = 1.0 / freq
    run_time = 8.0
    amp_deg = 5.0
    sine_freq = 0.3

    node.get_logger().info("servoj 스트리밍 시작 (연속 publish, override). 단발 호출로는 움직이지 않습니다.")
    t_start = time.perf_counter()
    next_tick = t_start
    count = 0
    last_log = t_start

    try:
        while rclpy.ok():
            now = time.perf_counter()
            elapsed = now - t_start
            if elapsed >= run_time:
                break

            phase = 2.0 * math.pi * sine_freq * elapsed
            target = list(pos0)
            target[0] = pos0[0] + amp_deg * math.sin(phase)

            # 연속 스트리밍: time=dt (혹은 0.0 도 가능) / override
            servoj(target, t=dt, mode=DR_SERVO_OVERRIDE)

            rclpy.spin_once(node, timeout_sec=0.0)

            if now - last_log >= 1.0 and count > 0:
                eff_hz = count / (now - t_start)
                node.get_logger().info(f"Sent={count}, Hz≈{eff_hz:.1f}")
                last_log = now

            next_tick += dt
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 지터 누적 방지 재동기
                next_tick = time.perf_counter()
            count += 1
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")

    # 8. 종료 안정화: 마지막 위치 유지
    final_pos = get_current_posj()
    if final_pos:
        servoj(final_pos, t=0.05, mode=DR_SERVO_OVERRIDE)

    node.get_logger().info(f"스트리밍 종료. 보낸 샘플: {count}")
    node.get_logger().info(f"최종 관절: {final_pos}")
    if final_pos:
        node.get_logger().info(f"J0 변화량: {final_pos[0]-pos0[0]:.2f} deg")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()