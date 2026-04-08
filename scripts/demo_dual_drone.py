#!/usr/bin/env python3
"""두 드론을 동시에 제어하는 데모 스크립트.

Drone0: 전진 후 좌회전
Drone1: 전진 후 우회전
각 드론의 카메라 이미지도 캡처하여 저장.
"""
import airsim
import time
import os
import numpy as np

def main():
    # AirSim 연결
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("AirSim 연결 완료!")

    drones = ["Drone0", "Drone1"]

    # 각 드론 API 제어 활성화 + 시동
    for name in drones:
        client.enableApiControl(True, vehicle_name=name)
        client.armDisarm(True, vehicle_name=name)
        print(f"  {name}: API 제어 활성화, 시동 걸림")

    # 동시 이륙
    print("\n--- 이륙 ---")
    f0 = client.takeoffAsync(vehicle_name="Drone0")
    f1 = client.takeoffAsync(vehicle_name="Drone1")
    f0.join()
    f1.join()
    print("  두 드론 이륙 완료!")
    time.sleep(2)

    # 상태 확인
    for name in drones:
        state = client.getMultirotorState(vehicle_name=name)
        pos = state.kinematics_estimated.position
        print(f"  {name} 위치: x={pos.x_val:.1f}, y={pos.y_val:.1f}, z={pos.z_val:.1f}")

    # 카메라 이미지 캡처
    print("\n--- 카메라 이미지 캡처 ---")
    os.makedirs("/tmp/airsim_demo", exist_ok=True)
    for name in drones:
        responses = client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
        ], vehicle_name=name)
        if responses and responses[0].width > 0:
            r = responses[0]
            channels = len(r.image_data_uint8) // (r.height * r.width)
            img = np.frombuffer(r.image_data_uint8, dtype=np.uint8).reshape(r.height, r.width, channels)
            import cv2
            bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR) if channels == 4 else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            path = f"/tmp/airsim_demo/{name}_camera.png"
            cv2.imwrite(path, bgr)
            print(f"  {name}: {r.width}x{r.height} 이미지 저장 → {path}")

        # 카메라 정보
        info = client.simGetCameraInfo("front_center", vehicle_name=name)
        print(f"  {name}: FOV={info.fov:.1f}°")

    # 드론 이동 (병렬)
    print("\n--- 이동 시작 ---")
    print("  Drone0: 전방 10m 이동")
    print("  Drone1: 우측 10m 이동")
    f0 = client.moveToPositionAsync(-10, 0, -5, 5, vehicle_name="Drone0")
    f1 = client.moveToPositionAsync(0, 10, -5, 5, vehicle_name="Drone1")
    f0.join()
    f1.join()
    print("  이동 완료!")

    # 최종 위치 확인
    print("\n--- 최종 위치 ---")
    for name in drones:
        state = client.getMultirotorState(vehicle_name=name)
        pos = state.kinematics_estimated.position
        print(f"  {name}: x={pos.x_val:.1f}, y={pos.y_val:.1f}, z={pos.z_val:.1f}")

    # 이동 후 카메라 재캡처
    print("\n--- 이동 후 카메라 캡처 ---")
    for name in drones:
        responses = client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
        ], vehicle_name=name)
        if responses and responses[0].width > 0:
            r = responses[0]
            channels = len(r.image_data_uint8) // (r.height * r.width)
            img = np.frombuffer(r.image_data_uint8, dtype=np.uint8).reshape(r.height, r.width, channels)
            import cv2
            bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR) if channels == 4 else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            path = f"/tmp/airsim_demo/{name}_camera_moved.png"
            cv2.imwrite(path, bgr)
            print(f"  {name}: 이동 후 이미지 저장 → {path}")

    # 호버링
    print("\n--- 호버링 ---")
    client.hoverAsync(vehicle_name="Drone0").join()
    client.hoverAsync(vehicle_name="Drone1").join()

    print("\n데모 완료! 이미지 파일: /tmp/airsim_demo/")
    print("드론은 호버링 상태입니다.")


if __name__ == "__main__":
    main()
