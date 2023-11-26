# Dynamic Reconfigure about lane detection application

### 공통 사항
- 각각의 Parameter에 대해 동작 중에 수정할 수 있는 기능 제공
- 각각의 기본 값 및 최소, 최대 값에 대해 정의 가능

### image_processing.cfg
- 이미지 기반의 차선 인식에 사용하는 Dynamic Reconfigure

### crosswalk.cfg
- 이미지 기반의 횡단 보도 인식에 사용하는 Dynimic Reconfigure

### lidar_e_stop.cfg
- LiDAR 기반의 장애물 유무 판단에 사용하는 Dynamic Reconfigure

### control.cfg
- 인식한 차선, 횡단 보도, 장애물 등을 통합하여 주행하는 제어에 사용하는 Dynamic Reconfigure