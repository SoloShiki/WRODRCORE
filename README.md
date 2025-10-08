1. ***Talker:***  (initial tests)
* [publisher.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/1.%20Talker/publisher.py) Simple node to publish a hello world
* [subscriber.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/1.%20Talker/subscriber.py) Simple node to subcscribe to the publisher and show data   
2. ***Talker_B***
* [rpm_pub.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/2.%20Talker_B/rpm_pub.py) Simple node to publish a variable like RPM
* [speed_pub.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/2.%20Talker_B/speed_calc.py) Simple node to subcscribe to rpm and pusblis speed
3. ***Motion***
* [motion_simple.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/3.%20Motion/Motion_Simple.py) Run foward for 2 seconds and stop
* Motion_controll.py
4. ***Motion_B***
* Motion_control_B.py
5. ***Turning***
* Motion_control_C.py
6. ***Distance***
* [Mazed_test.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/6.%20Distance/Mazed_test.py) Solve a simple mazed , robot only avances each grid
* [Mazed_Printing.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/6.%20Distance/Mazed_Printing.py) Solve a simple mazed , robot only avances each grid, this program prints each step
* [Mazed_Ascci_disp.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/6.%20Distance/Mazed_Ascci_disp.py) This program clears the screen and provide an updated ascii map for the robot
* [Mazed_Matpolib.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/6.%20Distance/Mazed_Matpolib.py) This program  create a visual map, create a variable defining the grid
* [Mazed_Turning_PID.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/6.%20Distance/Mazed_Turning_PID.py) Same Program as above but now turing and using PID, to be tested.
* [Simple_Lidar_Only.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/6.%20Distance/Simple_Lidar_Only.py) Simple Lidar Test, plot using matplotlib (to be tested)
* [Motion_Control_S_Curve.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/6.%20Distance/Motion_Control_S_Curve.py) Turn in S shape for 10 Secs
* Mazed_test_B.py
* Motion_Control_D.py
* Motion_Control_E_Lidar.py
* Motion_Control_F_Lidar_Test.py

============================================================

7. ***Mecanum Tests:***
* [drivingcheck_aftercal.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/7.%20Mecanum/drivingcheck_aftercal.py) After calibration in new chasis, this is distance testing to 1 M checking for tf and odometry.
  at the end tf and odometry must agree.
* [Mecanum_Maze_A.py](https://github.com/EdwinMarteZorrilla/WROS_RD_ROS/blob/main/7.%20Mecanum/Mecanum_Maze_A.py) Esto trabaja perfectamente usando el A*, se desplaza 0.4 con la grid en 0.2 M. El mapa lo actualiza perfectmente
  y es 100% funcional, no gira sino que se va de lado, y se va hacia atras pero siempre va de frente hacia la meta.

