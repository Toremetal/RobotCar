

SmartRobotCarV4.0_V1_20220303

1#Compatible with TB6612 motor drive versionSmartRobotCarV4.0_V1_20230201
2#GY-521


主控模块：uno
测距模块：HC-SRO4
寻迹模块：LTI-PCB
陀螺模块：GY-521
电机驱动：TB6612
摄像模块：ESP32-WROVER



***************************************************************************

张德智   20220303

1.因为最新的AVR BOARD编译出错，所以去掉软件中断库PinChangeInt，改为硬件中断0

***************************************************************************

***************************************************************************

张德智   20230201

1.102命令加入速度控制Rocker_CarSpeed = doc["D2"];

***************************************************************************