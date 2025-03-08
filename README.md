**1. gps 드라이버 실행**
'''roslaunch ublox_gps ublox_zed-f9p.launch'''

**2. ntrip 수신**
'''roslaunch ntrip_ros ntrip_ros.launch'''

**3. local cartesian launch 파일 실행**
'''roslaunch gps_to_utm_pkg local_cartesian.launch'''

**4. roi_path_publisher 실행**
'''rosrun gps_to_utm_pkg roi_path_publisher.py'''

