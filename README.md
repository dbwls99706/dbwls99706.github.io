# Yujin Hong - Robotics Software Engineer

로봇 제어와 자율주행을 실제 현장에 연결하는 소프트웨어 엔지니어 홍유진의 포트폴리오입니다.

**사이트 바로 가기: <https://dbwls99706.github.io>**

## 담긴 내용

| 구분 | 내용 |
| --- | --- |
| [Experience](https://dbwls99706.github.io/#experience) | VIRNECT(Unitree Go2 기반 공장 자율주행·관제), NBYUS(산업 자동화 로봇 SW) |
| [Open Source](https://dbwls99706.github.io/#opensource) | ROS 2 업스트림 기여(rclcpp, rclpy, rosbag2), ROS 2 Engineering Skills 등 |
| [Apps & Services](https://dbwls99706.github.io/#apps) | One Page Today, 만렙개미, Velog Backup |
| [Robotics Projects](https://dbwls99706.github.io/#projects) | ROS 2, Nav2, SLAM, YOLOv8 기반 프로젝트 11개 |

## 첫 화면 시뮬레이션

첫 화면에서는 사족보행 로봇이 LiDAR로 공장 바닥 지도를 만들어 갑니다. 브라우저에서 실시간으로 계산합니다.

- 점유 격자 위에서 레이캐스팅으로 지도를 넓히고, 코스트맵 팽창과 A*로 경로를 계획합니다.
- 지도를 누르면 그 지점까지 주행하고, W A S D 키나 터치 조이스틱으로 직접 조종할 수 있습니다.
- 로봇 몸체와 상자·콘·드럼통은 강체 물리로 계산해서, 부딪히면 밀리고 쓰러집니다.
- 로봇은 CadQuery로 직접 모델링했고, 다리는 3관절 역기구학으로 움직입니다. 모델 소스는 [`cad/`](cad)에 있습니다.

## 구조

| 경로 | 설명 |
| --- | --- |
| `index.html`, `projects/` | 메인 페이지와 프로젝트 상세 페이지 |
| `css/style.css` | 전체 스타일 |
| `js/lidar.js`, `js/robot.js`, `js/physics.js` | 첫 화면 시뮬레이션(지도·경로 계획, 로봇 보행, 물리) |
| `js/motion.js`, `js/main.js`, `js/boot.js` | 스크롤·페이지 전환 모션, 테마, 공통 동작 |
| `cad/`, `assets/` | 로봇 CAD 모델과 변환된 메시 |
| `vendor/` | GSAP, Lenis, OGL, cannon-es |

## 로컬에서 보기

빌드 과정 없이 정적 파일로 동작합니다.

```bash
python3 -m http.server 8000
```

이후 <http://localhost:8000>을 엽니다.
