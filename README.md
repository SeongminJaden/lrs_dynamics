# ROS2 기반 LEO 위성 및 로봇팔 통합 시뮬레이션 플랫폼 개발
## 초록 (Abstract)

기존 Gazebo 및 ROS 기반 시뮬레이션 환경은 지상 로봇 중심으로 설계되어, 정밀한 저궤도(LEO) 중력장, 궤도 섭동, 위성-로봇 결합 동역학 등의 복잡한 우주 환경을 충분히 재현하기 어렵다.

본 연구에서는 ROS2와 Gazebo를 통합하여 고정밀 궤도 동역학 및 로봇팔 결합 시뮬레이션을 지원하는 LEO 위성용 통합 시뮬레이션 플랫폼을 제안한다.

제안된 시스템은 고정밀 궤도 전파기(High Precision Orbit Propagator, HPOP) 또는 선택적 SGP4를 이용한 실시간 궤도 계산, 위치 의존적 중력(GGM05C), 대기항력, 태양복사압 등의 환경 모델을 포함한다.

또한 ROS2 기반 통신 구조와 MoveIt2를 통한 로봇팔 제어를 지원하며, 위성 서비스, 우주 쓰레기 제거, 자율 조립 등의 시나리오를 정밀하게 검증할 수 있다.

본 플랫폼은 자율 우주 로봇 운용, 궤도 유지, 임무 계획 등 차세대 우주로봇 연구의 기반 환경을 제공한다.

### 주요어: ROS2, Gazebo, LEO, 위성 시뮬레이션, 로봇팔, 중력장 모델, HPOP

## 1. 서론 (Introduction)

최근 우주 서비스(위성 수리, 연료 보급, 우주 쓰레기 제거 등) 분야에서 자율 로봇팔을 탑재한 인공위성의 활용이 급증하고 있다.

이러한 시스템은 궤도 운동과 로봇팔 동역학이 상호작용하기 때문에, 단순 지상 환경 시뮬레이션으로는 그 성능을 검증하기 어렵다.


기존 Gazebo 및 ROS 기반 시뮬레이션은 중력 가속도를 일정한 지상값으로 고정하고, 궤도 외란이나 자세 제어 반작용을 충분히 반영하지 못한다.

따라서 궤도 섭동, 대기항력, 태양복사압, 비대칭 중력장 등 LEO 환경 특성을 반영한 시뮬레이션 프레임워크가 필요하다.

본 연구에서는 이러한 한계를 해결하기 위해 ROS2-Gazebo 통합형 LEO 위성 시뮬레이션 플랫폼을 제안한다.

이 시스템은 HPOP 기반 실시간 궤도 계산과 Gazebo 물리엔진의 로봇팔 동역학을 통합하여, 위성-로봇 결합 운동을 정밀하게 재현한다.

## 2. 관련 연구 (Related Works)

ROS2와 Gazebo는 지상 로봇 및 자율주행 분야에서 널리 사용되어 왔으나,
우주 환경(중력장 변화, 궤도 섭동, 무중력 자세제어)을 직접 반영한 시뮬레이션 연구는 제한적이다.

SGP4 (Simplified General Perturbation Model): TLE 기반 궤도 예측 모델로, 저정밀 시뮬레이션에는 적합하지만 실시간 물리 계산에는 부정확함.

HPOP (High Precision Orbit Propagator): 수치적 적분 기반 궤도 전파기. 대기, 복사압, 섭동 등을 포함 가능.

GGM05C Gravity Model: GRACE 위성 데이터를 기반으로 한 고차 중력장 모델로, 위치 의존적 중력 가속도를 계산 가능.

MoveIt2: ROS2 기반의 로봇팔 제어 및 경로 계획 프레임워크.

본 연구는 위 세 가지 기술을 통합하여 ROS2 + Gazebo 환경에서의 실시간 우주 로봇 시뮬레이션을 구현하였다.

## 3. 제안 시스템 ROS2-Based LEO Satellite & Robotic Arm Integrated Simulation

### 1. Executive Summary

This project builds a high-fidelity simulation platform integrating ROS2 and Gazebo for LEO satellite dynamics and a satellite-mounted robotic arm. It includes real-time orbit propagation using HPOP (or optional SGP4), satellite attitude and manipulator dynamics, coupled dynamics, and environmental physics including position-dependent gravity using GGM05C, atmospheric drag, and solar radiation.

### 2. Background & Motivation

* Robotic arms are essential for space services (repair, assembly, cargo handling).
* Ground verification requires combined orbit propagation and manipulator dynamics.
* Gazebo alone cannot simulate high-precision orbital motion or LEO gravity accurately.

### 3. Objectives

1. HPOP-based real-time orbit propagator ROS2 node
2. Orbit-to-Gazebo bridge for model updates
3. Satellite + 6-DoF manipulator URDF/SDF modeling
4. Coupled dynamics simulation (arm torque → satellite attitude)
5. Verification scenarios: approach, grasp, manipulation

### 4. Scope & Assumptions

* ROS2 Humble/Iron
* Gazebo Fortress/Garden
* HPOP numerical integration with dt-based publishing
* Optional SGP4 for rapid prototyping
* Development PC: quad-core+, 16GB RAM
* dt: 0.1–1.0 s recommended

### 5. System Architecture

**Components:**

1. Orbit Propagator Node (HPOP) → publishes `/satellite/orbit_state`
2. Orbit-to-Gazebo Bridge → ECI → Gazebo conversion
3. Gazebo World & Plugins → ModelPlugin, WorldPlugin
4. Satellite & Manipulator → URDF/SDF with 6-DoF arm
5. Control Stack → Arm + Satellite attitude controller
6. Visualization & Logging → RViz, Gazebo GUI, CSV/rosbag

**Data Flow:**

```
Initial state → HPOP → (r,v) → ECI→Gazebo → Plugin → Satellite Pose → Physics → Arm Controller → Joint Torques → Reaction → Next Step
```

### 6. Messages & Interfaces

* `/satellite/orbit_state` : `nav_msgs/Odometry`
* `/satellite/eci_state` : custom `{ header, r[3], v[3], epoch }`
* `/gazebo/set_model_state` : `gazebo_msgs/ModelState`
* Services: `/orbit/reset`, `/orbit/set_parameters`

### 7. Physics & Gravity Modeling

**Earth Gravity Potential (GGM05C):**

$$
V(r, \phi, \lambda) = \frac{GM}{r} \left[1 + \sum_{n=2}^{n_{max}} \left(\frac{a}{r}\right)^n \sum_{m=0}^{n} \bar{P}*{n,m}(\sin \phi)(\bar{C}*{n,m}\cos(m\lambda) + \bar{S}_{n,m}\sin(m\lambda))\right]
$$


**Fully-Normalized Associated Legendre Functions:**

$$
\bar{P}*{n,m}(x) = N*{n,m} P_{n,m}(x),\quad N_{n,m} = \sqrt{(2-\delta_{m0})(2n+1)\frac{(n-m)!}{(n+m)!}}
$$


**Gravitational Acceleration:**

$$
\mathbf{a} = -\nabla V
$$

Components:

$$
a_r = -\frac{\partial V}{\partial r},\quad a_\phi = -\frac{1}{r} \frac{\partial V}{\partial \phi},\quad a_\lambda = -\frac{1}{r\cos\phi} \frac{\partial V}{\partial \lambda}
$$


**Spherical → Cartesian:**

$$
\begin{aligned}
a_x &= a_r \cos\phi \cos\lambda - a_\phi \sin\phi \cos\lambda - a_\lambda \sin\lambda \
a_y &= a_r \cos\phi \sin\lambda - a_\phi \sin\phi \sin\lambda + a_\lambda \cos\lambda \
a_z &= a_r \sin\phi + a_\phi \cos\phi
\end{aligned}
$$


**Force in Gazebo:**

$$
\mathbf{F} = m \mathbf{a}
$$


**Coupled Satellite-Arm Dynamics:** Arm motion induces torque on satellite. Use HPOP for CoM orbit, Gazebo for attitude + arm, ModelPlugin for reaction.

### 8. Synchronization

* HPOP publish: 10 Hz (dt=0.1s)
* Gazebo update: faster than HPOP
* Frames: ECI → Gazebo world (or ENU)

### 9. Development Environment

* Ubuntu 22.04
* ROS2 Humble/Iron
* Gazebo Fortress/Garden
* MoveIt2
* C++ (plugins), Python (nodes)

### 10. Build & Run

```bash
colcon build --packages-select satellite_description satellite_plugins satellite_control satellite_moveit satellite_utils gazebo_leo_gravity
source install/setup.bash
ros2 launch satellite_sim satellite_sim.launch.py
```

---

## 한글 버전

### 1. 요약

본 프로젝트는 ROS2 + Gazebo 환경에서 LEO 위성의 궤도와 로봇팔 동역학을 통합하는 고정밀 시뮬레이션 플랫폼을 구축하는 것을 목표로 합니다.

### 2. 배경 및 필요성

* 위성 로봇팔은 수리, 조립, 물자 이송 등 우주서비스 핵심
* 지상 검증에는 궤도 + 로봇팔 동역학 통합 시뮬레이션 필요
* Gazebo 단독은 고정밀 궤도와 LEO 중력 시뮬레이션에 한계

### 3. 목표

1. HPOP 기반 실시간 궤도 ROS2 노드
2. Orbit-to-Gazebo 브리지 구현
3. 위성 + 6-DoF 로봇팔 URDF/SDF 모델링
4. 팔 토크 → 위성 자세 반작용 시뮬레이션
5. 검증 시나리오: 접근, 조작, 잡기

### 4. 범위 및 가정

* ROS2 Humble 이상
* Gazebo Fortress/Garden
* HPOP dt 단위 결과 publish
* 선택적 SGP4 프로토타이핑
* 개발 PC: 쿼드코어+, 16GB RAM
* dt: 0.1–1s

### 5. 시스템 아키텍처

**구성요소:**

1. Orbit Propagator Node → `/satellite/orbit_state` publish
2. Orbit-to-Gazebo Bridge → ECI → Gazebo
3. Gazebo World & Plugins → ModelPlugin, WorldPlugin
4. Satellite & Manipulator → URDF/SDF, 6-DoF arm
5. Control Stack → 팔 + 위성 자세 제어
6. Visualization & Logging → RViz, Gazebo GUI, CSV/rosbag

**데이터 흐름:**

```
초기 상태 → HPOP → (r,v) → ECI→Gazebo → Plugin → 위성 pose → 물리엔진 → 팔 제어 → joint torque → Plugin 반작용 → 다음 스텝
```

### 6. 메시지/인터페이스

* `/satellite/orbit_state` : `nav_msgs/Odometry`
* `/satellite/eci_state` : custom `{ header, r[3], v[3], epoch }`
* `/gazebo/set_model_state` : `gazebo_msgs/ModelState`
* 서비스: `/orbit/reset`, `/orbit/set_parameters`

### 7. 물리 모델링

**지구 중력 퍼텐셜 (GGM05C):**

$$
V(r, \phi, \lambda) = \frac{GM}{r} [1 + \sum_{n=2}^{n_{max}} (\frac{a}{r})^n \sum_{m=0}^{n} \bar{P}*{n,m}(\sin \phi)(\bar{C}*{n,m}\cos(m\lambda) + \bar{S}_{n,m}\sin(m\lambda))]
$$


**완전정규화 레젠드르 함수:**

$$
\bar{P}*{n,m}(x) = N*{n,m} P_{n,m}(x),\quad N_{n,m} = \sqrt{(2-\delta_{m0})(2n+1)\frac{(n-m)!}{(n+m)!}}
$$


**중력 가속도:**

$$
\mathbf{a} = -\nabla V,\quad a_r = -\frac{\partial V}{\partial r}, a_\phi = -\frac{1}{r}\frac{\partial V}{\partial \phi}, a_\lambda = -\frac{1}{r\cos\phi}\frac{\partial V}{\partial \lambda}
$$


**구면 → Cartesian 변환:**

$$
\begin{aligned} a_x &= a_r \cos\phi \cos\lambda - a_\phi \sin\phi \cos\lambda - a_\lambda \sin\lambda \
a_y &= a_r \cos\phi \sin\lambda - a_\phi \sin\phi \sin\lambda + a_\lambda \cos\lambda \
a_z &= a_r \sin\phi + a_\phi \cos\phi \end{aligned}
$$


**Gazebo 힘 적용:**

$$
\mathbf{F} = m \mathbf{a}
$$


**팔-위성 결합 동역학:** 팔 운동 → 위성 토크 반작용. HPOP: CoM, Gazebo: 자세+팔, Plugin: 반작용 계산.

### 8. 동기화

* HPOP publish: 10 Hz (dt=0.1s)
* Gazebo update: HPOP보다 빠름
* 좌표계: ECI → Gazebo world (or ENU)

### 9. 개발 환경

* Ubuntu 22.04
* ROS2 Humble/Iron
* Gazebo Fortress/Garden
* MoveIt2
* C++ (Plugin), Python (Node)

### 10. 빌드 & 실행

```bash
colcon build --packages-select satellite_description satellite_plugins satellite_control satellite_moveit satellite_utils gazebo_leo_gravity
source install/setup.bash
ros2 launch satellite_sim satellite_sim.launch.py
```

