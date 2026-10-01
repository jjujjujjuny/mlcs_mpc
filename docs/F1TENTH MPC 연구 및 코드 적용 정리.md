## 0. 선행 학습 자료

### 최적화 및 MPC 기본 이론

**Optimization / Model Predictive Control 강의 영상**

* 최적화 문제의 기본 개념 및 수식 이해
* MPC의 optimization formulation을 이해하기 위한 선행 학습 자료
* 이후 MPCC 및 Learning-based MPC 논문을 이해하는 데 필요한 기본 이론 학습용

YouTube 강의:
https://www.youtube.com/watch?v=SvAYJC7jug8&list=PLZnJoM76RM6IAJfMXd1PgGNXn3dxhkVgI

## 1. 코드 및 실제 적용

### 1.1 기본 코드 수정

* 기존 기본 코드 수정 가능 여부 확인
* 가능하면 기존 구조를 최대한 유지하면서 필요한 MPC 및 차량 모델 부분만 수정
* 이후 F1TENTH 차량 모델 및 실제 플랫폼에 적용 가능한지 검토

### 1.2 F1TENTH 모델 적용

* 기본 MPC 코드 검증
* F1TENTH 차량 동역학 모델로 변경
* F1TENTH 환경에서 폐루프 주행 검증
* 이후 Learning-based Dynamics 적용 가능성 검토

### 1.3 실제 F1TENTH 데이터 수집 및 실험

**F1TENTH Mocap Data Recording Pipeline**

* F1TENTH 실제 차량의 주행 데이터를 수집하기 위한 코드
* OptiTrack Motion Capture 시스템을 이용한 차량 상태 및 궤적 측정
* ROS 기반 F1TENTH 실험 환경과 Motion Capture 시스템 연동
* 실제 차량의 상태 및 입력 데이터 기록에 활용 가능
* 이후 차량 모델 식별, Learning-based Dynamics 학습 데이터 구축, MPC 실차 검증 등에 활용 가능

GitHub:
https://github.com/Tinker-Twins/F1TENTH-Mocap-Data-Recording-Pipeline

## 2. 논문 및 오픈소스

### 2.1 MPCC 기본

**Optimization-Based Autonomous Racing of 1:43 Scale RC Cars**

* Autonomous Racing에서 대표적으로 사용되는 MPCC formulation
* Contouring error와 lag error를 이용하여 경로 진행과 추종을 동시에 최적화
* MPCC의 기본적인 수식 및 구현 구조 참고

GitHub:
https://github.com/alexliniger/MPCC

### 2.2 Trajectory Optimization

**Global Race Trajectory Optimization**

* 주어진 트랙으로부터 racing line 및 velocity profile 생성
* Minimum-curvature trajectory 등을 생성하여 MPC의 reference로 활용

GitHub:
https://github.com/Technion-F1Tenth/Global-Race-Trajectory-Optimization

F1TENTH 관련 구현:
https://github.com/ryanxxxhuang/F1tenth

## 3. Learning-Based MPC 관련 논문

### 3.1 GP 기반 Learning MPC

**Learning-Based Model Predictive Control for Autonomous Racing**

* Gaussian Process 기반 모델 보정
* Learning-based vehicle dynamics를 MPC에 적용한 기본적인 참고 논문

### 3.2 Neural Network 기반 Real-Time MPC

**Real-time Neural MPC: Deep Learning Model Predictive Control for Quadrotors and Agile Robotic Platforms**

* Neural Network dynamics를 MPC에 적용
* NN 기반 dynamics와 real-time MPC를 결합하는 구조 참고

### 3.3 acados 기반 Learning MPC

**L4acados: Learning-Based Models for acados, Applied to Gaussian Process-Based Predictive Control**

* Learning-based model을 acados에 적용하기 위한 프레임워크
* GP 및 NN과 같은 learned dynamics와 MPC를 결합하는 구조 참고
* 실제 real-time Learning-based MPC 구현 관점에서 참고
