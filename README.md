# SeBAR - Self-Balancing Assistant Robot

산업용 환경을 위한 자가 균형 어시스턴트 로봇 프로젝트입니다.  
PID 제어 기반으로 안정적인 밸런스를 유지하며, ROS2 통신을 통해 실시간 제어가 가능합니다. 손동작 제스처 인식과 폼 체인지 기능도 지원합니다.

---

## 📌 프로젝트 개요

- **프로젝트명**: SeBAR (Self–Balancing Assistant Robot)
- **목표**: 자가 균형 기능과 제스처 기반 직관적 제어를 통한 작업 효율성 및 안정성 향상
- **개발 기간**: 2024년
- **담당 역할**: 밸런스 제어 시스템 개발 (PID 제어기 기반)

---

## ⚙️ 시스템 구성

- **하드웨어**
  - NVIDIA Jetson AGX Xavier (ROS2 기반 제어)
  - Dynamixel MX-106 (4개)
  - IAHRS (RB-SDA-v1) IMU 센서
  - ZED Stereo Camera
  - OPENCR 1.0 컨트롤러

- **소프트웨어**
  - Ubuntu 20.04 + ROS2 Foxy
  - C++, Python
  - PID 제어 알고리즘
  - IMU 데이터 기반 밸런스 제어
  - YOLOv5 (Gesture Recognition)
  - Roboflow (Dataset 관리)

---

## 🔥 주요 기능

- **자가 균형 유지**: IMU 센서 기반 PID 제어
- **제스처 제어**: YOLOv5로 손동작 인식
- **폼 체인지**: Balancing Mode / Creeping Mode 전환
- **ROS2 통신 기반 제어**: Topic 발행/구독으로 모듈간 통신
- **객체 추적 및 거리 계산**: ZED Camera 활용

---

## 🛠️ 밸런스 제어 로직

```text
1. IMU 센서에서 오일러 각도(roll, pitch) 수집
2. 목표 자세(0°)와 현재 자세를 비교하여 오차 계산
3. PID 제어기로 오차 보정
4. Dynamixel 모터에 제어 신호 전달
5. 모터 목표 위치 업데이트 → 자세 복원
