# GNSS-Python: Python으로 구현하는 GNSS 수신기 위치 결정

[![Python](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

🛰️ 이 프로젝트는 GNSS(Global Navigation Satellite System) 데이터를 처리하여 수신기의 위치를 계산하는 알고리즘을 순수 Python으로 구현한 것입니다.

RINEX 형식의 관측 파일과 항법 메시지 파일을 읽어, **최소제곱법(Least Squares)**을 이용해 수신기의 3차원 좌표를 추정합니다. GNSS의 기본 원리를 학습하고 직접 구현해보고자 하는 학생 및 연구자들을 위한 교육용 자료입니다.

## ✨ 주요 기능

*   **RINEX 파일 파서**: RINEX 2.xx 버전의 관측 파일(`.o`)과 항법 메시지 파일(`.n`)을 파싱합니다.
*   **위성 위치 계산**: 항법 메시지(Ephemeris)를 이용해 특정 시점의 위성 궤도와 시계 오차를 계산합니다.
*   **수신기 위치 추정**: 최소제곱법(Least Squares) 알고리즘을 기반으로 수신기의 위치(x, y, z)와 시계 오차를 추정합니다.
*   **좌표계 변환**: 계산된 ECEF 좌표를 일반적인 측지 좌표계(위도, 경도, 고도)로 변환합니다.
*   **결과 시각화**: Matplotlib을 이용해 계산된 위성 궤도와 수신기 위치를 3D 그래프로 시각화합니다.

## 📂 프로젝트 구조

프로젝트는 각 기능별로 모듈화되어 있어 코드의 역할을 쉽게 파악할 수 있습니다.

| 파일명           | 주요 역할                                                              |
| :--------------- | :--------------------------------------------------------------------- |
| `main.py`        | **프로그램의 진입점(Entry Point)**. 전체 프로세스를 총괄하고 조율합니다. |
| `rinex.py`       | **RINEX 파일 파서**. 관측 및 항법 메시지 파일을 읽고 파싱합니다.         |
| `ephemeris.py`   | **위성 위치 계산기**. 항법 메시지를 이용해 위성의 위치와 시계 오차를 계산합니다. |
| `position.py`    | **수신기 위치 계산기**. 최소제곱법으로 수신기 위치를 계산합니다.         |
| `plot.py`        | **시각화 도구**. 계산 결과를 Matplotlib 그래프로 표시합니다.             |
| `utils.py`       | **유틸리티 함수 모음**. 좌표 변환, 시간 변환 등 공통 함수를 포함합니다.  |
| `data/`          | **샘플 데이터 디렉토리**. 예제 RINEX 파일이 포함되어 있습니다.           |

## 🚀 실행 원리 (How it Works)

`main.py`를 실행하면 다음과 같은 순서로 수신기 위치 결정이 진행됩니다.

1.  **데이터 로딩 및 파싱 (`rinex.py`)**
    *   사용자가 지정한 RINEX 관측 파일(`.o`)과 그에 대응하는 항법 메시지 파일(`.n`)을 읽습니다.
    *   관측 파일에서 각 시간(epoch)별 의사거리(pseudorange) 데이터를, 항법 메시지 파일에서 위성 궤도 정보를 추출하여 메모리에 적재합니다.

2.  **위성 위치 계산 (`ephemeris.py`)**
    *   관측 데이터의 각 시간(epoch)을 순회하며, 해당 시간에 관측된 모든 위성의 위치를 계산합니다.
    *   항법 메시지의 궤도 파라미터를 이용해 IS-GPS-200D 표준에 명시된 공식을 따라 위성의 ECEF 좌표(X, Y, Z)와 시계 오차를 정확히 계산합니다.

3.  **수신기 위치 추정 (`position.py`)**
    *   계산된 위성들의 위치와 관측된 의사거리 데이터를 이용해 선형화된 관측 방정식을 구성합니다.
    *   **최소제곱법(Least Squares)**을 반복적으로 적용하여 관측 잔차(residual)를 최소화하는 해(수신기 위치 변위 `Δx, Δy, Δz` 및 시계 오차 `Δdt`)를 구합니다.
    *   계산된 해가 일정 기준 이하로 수렴할 때까지 이 과정을 반복하여 최종 수신기 위치를 확정합니다.

4.  **좌표 변환 및 결과 출력 (`utils.py`)**
    *   계산된 ECEF 좌표(x, y, z)를 사람이 이해하기 쉬운 측지 좌표계(위도, 경도, 고도)로 변환합니다.
    *   최종 계산된 위치를 콘솔에 출력합니다.

5.  **결과 시각화 (`plot.py`)**
    *   계산된 위성들의 궤도와 최종 수신기 위치를 3D 공간상에 시각화하여 보여줍니다.

## 🏁 시작하기

### 1. 환경 설정

*   Python 3.x

### 2. 저장소 클론

```bash
git clone https://github.com/imhyeonwoo/gnss.git
cd gnss
```

### 3. 의존성 설치

```bash
pip install numpy matplotlib pandas
pip install utm
```

### 4. 실행

```bash
**1. gps 드라이버 실행** <br>
roslaunch ublox_gps ublox_zed-f9p.launch

**2. ntrip 수신** <br>
roslaunch ntrip_ros ntrip_ros.launch

**3. local cartesian launch 파일 실행** <br>
roslaunch gps_to_utm_pkg local_cartesian.launch

**4. roi_path_publisher 실행** <br>
rosrun gps_to_utm_pkg roi_path_publisher.py
```
프로그램이 실행되면 콘솔에 각 시간대별로 계산된 수신기의 위치(위도, 경도, 고도)가 출력되고, 모든 계산이 완료된 후 3D 시각화 결과 창이 나타납니다.



## 💡 향후 개선 방향 (Future Work)

이 프로젝트는 GNSS의 기본 원리 구현에 초점을 맞추고 있습니다. 정확도와 기능을 향상시키기 위해 다음과 같은 확장을 고려할 수 있습니다.

- [ ] **고급 보정 모델 추가**
    - [ ] 전리층 지연 보정 (Klobuchar 모델 등)
    - [ ] 대류층 지연 보정 (Saastamoinen 모델 등)
- [ ] **다중 GNSS 지원**
    - [ ] GLONASS, Galileo, BeiDou 등 다른 위성 시스템 처리 기능 추가 (RINEX 3.x 파서 필요)
- [ ] **필터링 기법 적용**
    - [ ] 칼만 필터(Kalman Filter)를 도입하여 시간에 따른 위치 변화를 더 부드럽고 정확하게 추정
- [ ] **코드 문서화**
    - [ ] Docstring 및 주석 보강을 통해 코드 가독성 향상
- [ ] **단위 테스트 (Unit Test)**
    - [ ] 모듈별 기능 검증을 위한 테스트 코드를 작성하여 코드의 신뢰성 및 안정성 확보
    
    
## 📄 라이선스
이 프로젝트는 MIT 라이선스를 따릅니다.

