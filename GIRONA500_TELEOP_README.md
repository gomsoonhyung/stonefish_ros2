# GIRONA500 AUV Keyboard Teleoperation

GIRONA500 AUV를 키보드로 제어할 수 있는 ROS2 노드입니다.

## 파일 구성

- `scripts/girona500_teleop.py`: 키보드 제어 노드
- `launch/girona500_teleop.launch.py`: 시뮬레이터와 제어 노드를 함께 실행하는 런치 파일

## 빌드 방법

```bash
cd /home/foc/projectAlpha
colcon build --packages-select stonefish_ros2
source install/setup.bash
```

## 실행 방법

### 방법 1: 런치 파일로 실행 (시뮬레이터 + 제어 노드)

```bash
source install/setup.bash
ros2 launch stonefish_ros2 girona500_teleop.launch.py
```

**주의**: 이 방법은 `xterm`이 필요합니다. 만약 xterm이 없다면:
```bash
sudo apt install xterm
```

### 방법 2: 개별 실행

터미널 1 - 시뮬레이터 실행:
```bash
source install/setup.bash
ros2 launch stonefish_ros2 stonefish_simulator.launch.py \
  simulation_data:=/home/foc/projectAlpha/stonefish/Tests/Data \
  scenario_desc:=/home/foc/projectAlpha/stonefish/Tests/Data/console_test.scn
```

터미널 2 - 키보드 제어 노드 실행:
```bash
source install/setup.bash
ros2 run stonefish_ros2 girona500_teleop.py
```

## 조작 방법

```
W/S : 전진 / 후진 (Surge)
A/D : 왼쪽 / 오른쪽 (Sway)
Q/Z : 위로 / 아래로 (Heave)
X   : 모든 추진기 정지
ESC : 종료
```

## 작동 원리

### Thruster 구성
GIRONA500 AUV는 5개의 쓰러스터를 가지고 있습니다:
- **ThrusterSurgePort**: 좌측 전후진 추진기
- **ThrusterSurgeStarboard**: 우측 전후진 추진기
- **ThrusterSway**: 좌우 이동 추진기
- **ThrusterHeaveBow**: 전방 상하 추진기
- **ThrusterHeaveStern**: 후방 상하 추진기

### 제어 방식
- 키 입력 시 0.1씩 추력이 증가/감소합니다
- 추력 범위: -1.0 ~ 1.0 (정규화된 값)
- 현재 추력 값이 화면에 실시간으로 표시됩니다

### ROS2 토픽
각 추진기는 다음 토픽으로 제어됩니다:
- `/GIRONA500/ThrusterSurgePort/setpoint`
- `/GIRONA500/ThrusterSurgeStarboard/setpoint`
- `/GIRONA500/ThrusterHeaveBow/setpoint`
- `/GIRONA500/ThrusterHeaveStern/setpoint`
- `/GIRONA500/ThrusterSway/setpoint`

## 문제 해결

### 키 입력이 작동하지 않는 경우
- 터미널이 활성화(포커스) 되어있는지 확인하세요
- 런치 파일로 실행했다면 xterm 창에서 키를 입력하세요

### 로봇이 움직이지 않는 경우
- 시뮬레이터가 정상적으로 실행되었는지 확인하세요
- ROS2 토픽이 정상적으로 발행되는지 확인:
  ```bash
  ros2 topic echo /GIRONA500/ThrusterSurgePort/setpoint
  ```

### 빌드 오류가 발생하는 경우
- ROS2가 제대로 소싱되었는지 확인:
  ```bash
  source /opt/ros/humble/setup.bash  # 또는 사용 중인 ROS2 배포판
  ```
- 의존성 패키지가 설치되어 있는지 확인:
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```
