# 기초로봇공학실험 프로젝트
### WEGO Robotics의 젯슨나노 기반의 로봇 limo를 이용하여 운전면허 기능시험 보조 로봇을 만드는 프로젝트를 진행하였다. (Ubuntu 18.04, ROS1 Melodic)
---
![화면 캡처 2023-11-27 163649](https://github.com/PolyGon-13/limo_project/assets/107293272/1aa50f86-bc82-4e6f-bbff-0e401558e8b3)
![화면 캡처 2023-11-27 164011](https://github.com/PolyGon-13/limo_project/assets/107293272/56fe814c-83d9-490b-a807-bfd14649384f)

limo가 수행해야 하는 동작의 큰 틀은 다음과 같다.
1. 차선 인식 및 자율주행
2. AR 마커 인식 및 해당 동작 수행
3. LiDAR를 이용한 장애물 인식 및 정지
4. 방지턱 구간에서의 동작 처리
---
AR 마커 동작을 세부적으로 나열해보면 다음과 같다.
- ID 0 : 정지 표지판
- ID 1 : 우회전 표지판
- ID 2 : 좌회전 표지판
- ID 3 : T자 주차 표지판
---
LiDAR를 이용한 세부적인 동작은 다음과 같다.
- 횡단보도에서 보행자 장애물을 감지하여 정지
- 차단기가 내려갈시 이를 감지하여 정지
---
## /src/limo_legend/scripts
### control.py
- 다른 코드들의 노드들을 서브스크라이브하여 로봇의 동작을 조정

### lane_detect.py
- 노란색 차선을 인식한 결과를 퍼블리시

### ar_marker.py
- AR 마커를 인식한 결과와 그에 따른 동작을 퍼블리시

### lidar_stop.py
- LiDAR를 이용해 장애물을 인식했는지 여부를 퍼블리시
---
