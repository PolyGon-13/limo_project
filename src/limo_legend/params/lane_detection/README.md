# Params about lane detection application

### lane_detection.yaml
- 이미지 기반의 차선 인식에 관련된 Parameter
- HLS 색 영역을 이용
- 노란색 차선을 인식하는 형태로 사용
- yellow_`colorspace`_`low or high`
- `colorspace` 부분은 `h`, `l`, `s`로 입력 가능
- `low or high` 부분은 `low`, `high`로 입력 가능
- ex) `yellow_h_low` --> H 영역에 대한 최소 값
- ex) `yellow_l_high` --> L 영역에 대한 최대 값
- 각 Parameter는 0 ~ 255까지 입력할 수 있습니다.
- 각 값에 따라, 해당하는 색 영역을 차선 검출에 사용합니다.

### crosswalk.yaml
- 이미지 기반의 횡단 보도 인식에 관련된 Parameter
- 차선과 동일하게 HLS 색 영역을 활용
- 차선과 다르게, 흰색으로 된 횡단 보도를 인식하는 형태
- white_`colorspace`_`low or high`
- `colorspace` 부분은 `h`, `l`, `s`로 입력 가능
- `low or high` 부분은 `low`, `high`로 입력 가능
- ex) `white_s_low` --> S 영역에 대한 최소 값
- ex) `white_h_high` --> H 영역에 대한 최대 값
- 각 Parameter는 0 ~ 255까지 입력할 수 있습니다.
- 각 값에 따라, 해당하는 색 영역을 횡단 보도 검출에 사용합니다.
- 직선 검출을 위한 Parameter인 `rho`, `theta`, `threshold` 를 변경하여, 직선 검출의 기준 변경 가능
- 직선이 어느 정도 이상 검출되었을 때, 횡단 보도로 판단할 지를 정하는 `crosswalk_detect_threshold`

### e_stop.yaml
- LiDAR 기반의 장애물 유무 판단에 관련된 Parameter
- `e_stop_min_angle_deg` --> 장애물을 판단할 최소 각도
- `e_stop_max_angle_deg` --> 장애물을 판단할 최대 각도
- `e_stop_distance_meter` --> 장애물을 판단할 거리 (meter)
- `e_stop_count` --> 장애물을 판단할 점의 개수
- 지정한 범위 내부에 LiDAR 점이 일정 이상일 경우, 장애물이 있다고 판단

### control.yaml
- 인식한 차선, 횡단 보도, 장애물 등을 통합하여 주행하는 제어에 관련된 Parameter
- `base_speed` --> 일반 주행 시 속도
- `lateral_gain` --> 차선에 따라 회전할 정도를 지정하는 값
- `reference_lane_x` --> 기준 차선의 Pixel 값
- `pedestrian_width_min` --> 보행자 표지판의 정상인지를 위한 최소 너비 (Pixel)