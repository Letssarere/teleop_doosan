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
        info = node.get_topic_names_and_types()
        # 간단히 topic 존재만 확인 후 ros2 topic info 를 대체
        pubs = node.count_publishers(topic)
        subs = node.count_subscribers(topic)
        if subs > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return False

def main():
    rclpy.init()
    node = rclpy.create_node("sample_servoj_debug", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        get_current_posj,
        get_robot_state,
        servoj,
        set_velj,
        set_accj,
        DR_SERVO_OVERRIDE,
        DR_COND_NONE,
    )

    # 1. discovery 및 구독자 대기
    node.get_logger().info("Discovery 대기...")
    rclpy.spin_once(node, timeout_sec=0.2)

    # (선택) servo stream subscriber 대기
    if not wait_for_sub(node, "servoj_stream", timeout=2.0):
        node.get_logger().warn("servoj_stream 구독자 확인 실패(계속 진행). dsr_control2가 구독 중인지 ros2 topic info로 재확인 요망.")

    # 2. 로봇 상태 확인
    try:
        state = get_robot_state()
        node.get_logger().info(f"robot_state(raw)={state}")
    except Exception as e:
        node.get_logger().warn(f"로봇 상태 조회 실패: {e}")

    # 3. 기본 속도/가속도 세팅 (global)
    try:
        set_velj(60)   # 모든 관절 동일 속도 제한
        set_accj(300)
    except Exception as e:
        node.get_logger().warn(f"set_velj/set_accj 실패: {e}")

    # 4. 현재 관절
    pos0 = get_current_posj()
    if not pos0:
        node.get_logger().error("현재 관절을 읽지 못했습니다.")
        node.destroy_node()
        rclpy.shutdown()
        return
    node.get_logger().info(f"Start pos: {pos0}")

    # 5. 단일 servoj 테스트 (time=2.0 으로 명확히 이동)
    test_target = list(pos0)
    test_target[0] = pos0[0] + 10.0  # 10deg 이동
    node.get_logger().info(f"[단일 이동] J0 +10deg -> {test_target}")
    servoj(test_target, t=2.0, mode=DR_SERVO_OVERRIDE)
    time.sleep(2.2)
    rclpy.spin_once(node, timeout_sec=0.0)
    pos_after = get_current_posj()
    node.get_logger().info(f"단일 이동 후 관절: {pos_after}")

    moved = (pos_after and abs(pos_after[0] - pos0[0]) > 5.0)
    if not moved:
        node.get_logger().error("단일 servoj 이동 반영 안 됨. (1) 구독자/네임스페이스 (2) 드라이버 버전 (3) 모션 점유 상태 확인 필요.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 원위치 복귀
    node.get_logger().info("원위치 복귀")
    servoj(pos0, t=2.0, mode=DR_SERVO_OVERRIDE)
    time.sleep(2.2)

    # 6. 스트리밍 사인파 (time=0 사용)
    freq = 100.0
    dt = 1.0 / freq
    run_time = 8.0
    amp_deg = 5.0
    sine_freq = 0.3

    node.get_logger().info("사인파 스트리밍 시작 (time=0, override) ...")
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

            # time=0 권장 테스트 (내부 즉시 반영 성격)
            servoj(target, t=0.0, mode=DR_SERVO_OVERRIDE)

            rclpy.spin_once(node, timeout_sec=0.0)

            if now - last_log >= 1.0 and count > 0:
                eff_hz = count / (now - t_start)
                node.get_logger().info(f"Streaming Hz≈{eff_hz:.1f}")
                last_log = now

            next_tick += dt
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()
            count += 1
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")

    # 마지막 위치 유지
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