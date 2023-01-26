# STM32F103_Line_Follower_Robot

• This is Line Follower Robot project with PID controller. Main purpose of this project is to control the robot with PID algorithm. STM32F1 series MCU is used for controller. Keil uVision used as programming software

<img src="https://user-images.githubusercontent.com/109805805/214834536-6c4dc1b9-1dc1-45c2-8cdf-09c8c6f6d49f.png" width="600" height="500"> 





• This part of the PID system is necessary to follow how curves take easily. Therefore, a fixed 
Kp value should be determined for the equation of proportional control based on error. It was 
observed that if the Kp has a small value, the curves would be followed by the robot easily. 
On the other hand, if Kp has a large value, the curves would be taken suddenly. Even though, 
Kp value was increased, there would be a situation as oscillation of robot and splitting from 
the line.

<img src="https://user-images.githubusercontent.com/109805805/214835491-2a79db35-0360-43b3-9b8d-2f27f8af90ef.png" width="500" height="300">

• As a conclusion, the line follower robot car was built with STM32F103 microcontroller, 
QTR8-RC reflectance sensor, L298N motor driver, 3.7V Li-Ion rechargeable batteries and 
robot kit successfully. Furthermore, the PID controller system was integrated in the code part 
to follow the track more reliably. The algorithm was designed for the specific cases situations 
on the track. Finally, the robot car was tested on the sample line and it was observed to finish 
the track successfully.
