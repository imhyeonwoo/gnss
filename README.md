**1. gps 드라이버 실행** <br>
roslaunch ublox_gps ublox_zed-f9p.launch

**2. ntrip 수신** <br>
roslaunch ntrip_ros ntrip_ros.launch

**3. local cartesian launch 파일 실행** <br>
roslaunch gps_to_utm_pkg local_cartesian.launch

**4. roi_path_publisher 실행** <br>
rosrun gps_to_utm_pkg roi_path_publisher.py

