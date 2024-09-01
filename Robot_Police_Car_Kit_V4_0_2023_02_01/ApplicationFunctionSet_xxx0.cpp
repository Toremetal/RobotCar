/*
 * @Author: ™T©ReMeTaL
 * @Date: 2023-3-18 11:59:09
 * @LastEditTime: 2023-3-18 2:14:35
 * @LastEditors: ™T©ReMeTaL
 * @Description: Robot Police Car V1.0
 * @FilePath: 
 */
//uint8_t cur = EEPROM.read(/*address=*/0);if (cur < 250 && cur > 0) {EEPROM.update(/*address=*/0,/*value=*/Rocker_CarSpeed);} else {EEPROM.write(/*address=*/0,/*value=*/Rocker_CarSpeed);}
#include <avr/wdt.h>
//#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>

//#include <EEPROM.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h"  //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 0
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

/*Hardware device object list*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;
/*f(x) int */
static boolean
function_xxx(long x, long s, long e)  //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}
static void
delay_xxx(uint16_t _ms) {
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++) {
    delay(1);
  }
}

/*Movement Direction Control List*/
enum SmartRobotCarMotionControl {
  Forward,        //(1)
  Backward,       //(2)
  Left,           //(3)
  Right,          //(4)
  LeftForward,    //(5)
  LeftBackward,   //(6)
  RightForward,   //(7)
  RightBackward,  //(8)
  stop_it         //(9)
};                //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

/*Mode Control List*/
enum SmartRobotCarFunctionalModel {
  Standby_mode,           /*Standby Mode*/
  TraceBased_mode,        /*Line Tracking Mode*/
  ObstacleAvoidance_mode, /*Obstacle Avoidance Mode*/
  Follow_mode,            /*Following Mode*/
  Rocker_mode,            /*Rocker Control Mode*/
  CMD_inspect,
  CMD_Programming_mode,                   /*Programming Mode*/
  CMD_ClearAllFunctions_Standby_mode,     /*Clear All Functions And Enter Standby Mode*/
  CMD_ClearAllFunctions_Programming_mode, /*Clear All Functions And Enter Programming Mode*/
  CMD_MotorControl,                       /*Motor Control Mode*/
  CMD_CarControl_TimeLimit,               /*Car Movement Direction Control With Time Limit*/
  CMD_CarControl_NoTimeLimit,             /*Car Movement Direction Control Without Time Limit*/
  CMD_MotorControl_Speed,                 /*Motor Speed Control*/
  CMD_ServoControl,                       /*Servo Motor Control*/
  CMD_LightingControl_TimeLimit,          /*RGB Lighting Control With Time Limit*/
  CMD_LightingControl_NoTimeLimit,        /*RGB Lighting Control Without Time Limit*/

};

/*Application Management list*/
struct Application_xxx {
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
  uint8_t current_Speed = 200;
};
Application_xxx Application_SmartRobotCarxxx0;

static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);
void delay10(void);
void delay30(void);
void delay50(void);
void delay70(void);
void delay100(void);
void delay300(void);

void delay10(void) {
  delay_xxx(10);
}
void delay30(void) {
  delay_xxx(30);
}
void delay50(void) {
  delay_xxx(50);
}
void delay70(void) {
  delay_xxx(70);
}

void delay100(void) {
  delay_xxx(100);
}

void delay300(void) {
  delay_xxx(300);
}

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void) {
  //bool res_error = true;
  Serial.begin(19200);//9600
  delay300();
  //Serial.println("™T©ReMeTaL Robotics\r\ntoremetal.com\r\n©2023 ™T©ReMeTaL");
  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  AppIRrecv.DeviceDriverSet_IRrecv_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  //res_error =
  delay300();
  AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();
  //uint8_t current_Speed = 200;//EEPROM.read(/*address=*/0);
  //Serial.print("current_Speed:");
  //Serial.println(current_Speed);
  //if (isnan(current_Speed)) {
  //EEPROM.write(/*address=*/0, /*value=*/250);
  //current_Speed = EEPROM.read(/*address=*/0);
  //Serial.print("current_Speed(init):");
  //Serial.println(current_Speed);
  //}
  Serial.println("");
  Serial.println("™T©ReMeTaL Robotics");
  Serial.println("toremetal.com");
  Serial.println("©2024 ™T©ReMeTaL");
}

/*ITR20001 Check if the car leaves the ground static */
static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void) {
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V && AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V && AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V) {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  } else {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}
/*
  Straight line movement control：For dual-drive motors, due to frequent motor coefficient deviations and many external interference factors, 
  it is difficult for the car to achieve relative Straight line movement. For this reason, the feedback of the yaw control loop is added.
  direction：only forward/backward
  directionRecord：Used to update the direction and position data (Yaw value) when entering the function for the first time.
  speed：the speed range is 0~255
  Kp：Position error proportional constant（The feedback of improving location resuming status，will be modified according to different mode），improve damping control.
  UpperLimit：Maximum output upper limit control
*/
//static
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit) {
  static float Yaw;  //Yaw
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10) {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable);  //Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  //if (en != directionRecord)
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false) {
    en = directionRecord;
    yaw_So = Yaw;
  }
  //Add proportional constant Kp to increase rebound effect
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit) {
    R = UpperLimit;
  } else if (R < 10) {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit) {
    L = UpperLimit;
  } else if (L < 10) {
    L = 10;
  }
  if (direction == Forward)  //Forward
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
  } else if (direction == Backward)  //Backward
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
  }
}
/*
  Movement Direction Control:
  Input parameters:     1# direction:Forward（1）、Backward（2）、 Left（3）、Right（4）、LeftForward（5）、LeftBackward（6）、RightForward（7）RightBackward（8）
                        2# speed(0--255)
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed) {
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  //uint8_t Kp, UpperLimit;
  //uint8_t speed = is_speed;//EEPROM.read(/*address=*/0);
  //is_speed;
  //Control mode that requires straight line movement adjustment（Car will has movement offset easily in the below mode，the movement cannot achieve the effect of a relatively straight direction
  //so it needs to add control adjustment）
  /*switch (Application_SmartRobotCarxxx0.Functional_Mode) {
    case Rocker_mode:
      Kp = 10;
      UpperLimit = 255;
      break;
    case ObstacleAvoidance_mode:
      Kp = 2;
      UpperLimit = 180;
      break;
    case Follow_mode:
      Kp = 2;
      UpperLimit = 180;
      break;
    case CMD_CarControl_TimeLimit:
      Kp = 2;
      UpperLimit = 180;
      break;
    case CMD_CarControl_NoTimeLimit:
      Kp = 2;
      UpperLimit = 180;
      break;
    default:
      Kp = 10;
      UpperLimit = 255;
      break;
  }*/
  switch (direction) {
    case /* constant-expression */
      Forward:
      /* code */
      //if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode) {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ is_speed, /*controlED*/ control_enable);  //Motor control
                                                                                                                                   //} else {                               //When moving forward, enter the direction and position approach control loop processing
      directionRecord = 1;
      //  ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, is_speed, Kp, UpperLimit);
      //}

      break;
    case /* constant-expression */ Backward:
      /* code */
      //if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode) {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ is_speed, /*controlED*/ control_enable);  //Motor control
                                                                                                                                   //} else {                                                                                                                    //When moving backward, enter the direction and position approach control loop processing
      directionRecord = 2;
      //  ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, is_speed, Kp, UpperLimit);
      //}

      break;
    case /* constant-expression */ Left:
      /* code */
      directionRecord = 3;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ is_speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ Right:
      /* code */
      directionRecord = 4;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ is_speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ LeftForward:
      /* code */
      directionRecord = 5;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ is_speed / 2, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ LeftBackward:
      /* code */
      directionRecord = 6;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ is_speed / 2, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ RightForward:
      /* code */
      directionRecord = 7;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_speed / 2,
                                             /*direction_B*/ direction_just, /*speed_B*/ is_speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ RightBackward:
      /* code */
      directionRecord = 8;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_speed / 2,
                                             /*direction_B*/ direction_back, /*speed_B*/ is_speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ stop_it:
      /* code */
      directionRecord = 9;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                             /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable);  //Motor control

      break;
    default:
      directionRecord = 10;
      break;
  }
}
/*
 Robot car update sensors' data:Partial update (selective update)
*/
void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void) {

  // AppMotor.DeviceDriverSet_Motor_Test();
  { /*Battery voltage status update*/
    static unsigned long VoltageData_time = 0;
    static int VoltageData_number = 1;
    if (millis() - VoltageData_time > 10)  //read and update the data per 10ms
    {
      VoltageData_time = millis();
      VoltageData_V = AppVoltage.DeviceDriverSet_Voltage_getAnalogue();
      if (VoltageData_V < VoltageDetection) {
        VoltageData_number++;
        if (VoltageData_number == 500)  //Continuity to judge the latest voltage value multiple
        {
          VoltageDetectionStatus = true;
          VoltageData_number = 0;
        }
      } else {
        VoltageDetectionStatus = false;
      }
    }
  }

  // { /*value updation for the ultrasonic sensor：for the Obstacle Avoidance mode*/
  //   AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&UltrasoundData_cm /*out*/);
  //   UltrasoundDetectionStatus = function_xxx(UltrasoundData_cm, 0, ObstacleDetection);
  // }

  { /*value updation for the IR sensors on the line tracking module：for the line tracking mode*/
    TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    TrackingDetectionStatus_R = function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingDetectionStatus_M = function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    TrackingDetectionStatus_L = function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E);
    //ITR20001 Check if the car leaves the ground
    ApplicationFunctionSet_SmartRobotCarLeaveTheGround();
  }

  // acquire timestamp
  // static unsigned long Test_time;
  // if (millis() - Test_time > 200)
  // {
  //   Test_time = millis();
  //   //AppITR20001.DeviceDriverSet_ITR20001_Test();
  // }
}
/*
  Startup operation requirement：
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Bootup(void) {
  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

static void CMD_Lighting(uint8_t is_LightingSequence, int8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B) {
  switch (is_LightingSequence) {
    case 0:
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(NUM_LEDS, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 1: /*Left*/
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(3, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 2: /*Forward*/
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(2, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 3: /*Right*/
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(1, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 4: /*Back*/
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(0, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    case 5: /*Middle*/
      AppRBG_LED.DeviceDriverSet_RBGLED_Color(4, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
      break;
    default:
      break;
  }
}

/*RBG_LED set*/
void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void) {
  static unsigned long getAnalogue_time = 0;
  FastLED.clear(true);
  if (true == VoltageDetectionStatus)  //Act on low power state？
  {
    if ((millis() - getAnalogue_time) > 3000) {
      getAnalogue_time = millis();
    }
  }
  unsigned long temp = millis() - getAnalogue_time;
  if (function_xxx((temp), 0, 500) && VoltageDetectionStatus == true) {
    switch (temp) {
      case 0 ... 83:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        break;
      case 84 ... 168:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);
        break;
      case 169 ... 253:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        break;
      case 254 ... 338:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);
        break;
      case 339 ... 419:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        break;
      case 420 ... 499:
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);
        break;
      default:
        break;
    }
  } else if (((function_xxx((temp), 500, 3000)) && VoltageDetectionStatus == true) || VoltageDetectionStatus == false) {
    switch (Application_SmartRobotCarxxx0.Functional_Mode)  //Act on mode control sequence
    {
      case /* constant-expression */ Standby_mode:
        /* code */
        {
          if (VoltageDetectionStatus == true) {
            static uint8_t setBrightness = 0;
            static boolean et = false;
            static unsigned long time = 0;

            if ((millis() - time) > 10) {
              time = millis();
              if (et == false) {
                setBrightness += 1;
                if (setBrightness == 100)
                  et = true;
              } else if (et == true) {
                setBrightness -= 1;
                if (setBrightness == 0)
                  et = false;
              }
            }
            // AppRBG_LED.leds[1] = CRGB::Blue;
            //AppRBG_LED.leds[0] = CRGB::Violet;
            AppRBG_LED.leds[0] = CRGB::Blue;
            FastLED.setBrightness(setBrightness);
            FastLED.show();
          } else {
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
            delay100();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
            delay100();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
            delay100();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
            delay(500);
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
            delay300();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
            delay300();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
            delay300();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
            delay300();
          }
        }
        break;
      case /* constant-expression */ CMD_Programming_mode:
        /* code */
        {
        }
        break;
      case /* constant-expression */ TraceBased_mode:
        /* code */
        {
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay50();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
          delay10();
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay50();
          //AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Green);
        }
        break;
      case /* constant-expression */ ObstacleAvoidance_mode:
        /* code */
        {
          unsigned int c = 0;
          while (c < 2) {
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
            delay70();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
            delay70();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
            delay70();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
            delay70();
            c += 1;
          }
        }
        break;
      case /* constant-expression */ Follow_mode:
        /* code */
        {
          unsigned int c = 0;
          while (c < 2) {
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
            delay30();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(1 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
            delay30();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(1 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
            delay30();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(1 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
            delay30();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(1 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
            delay30();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(1 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
            delay30();
            AppRBG_LED.DeviceDriverSet_RBGLED_xxx(1 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
            delay30();
            c += 1;
          }
          //AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
        }
        break;
      case /* constant-expression */ Rocker_mode:
        /* code */
        {
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Violet);
        }
        break;
      default:
        break;
    }
  }
}

/* ==============================\
    ^  Rocker control mode ###
  < O >  ####################
    v   ######################
 */
void ApplicationFunctionSet::ApplicationFunctionSet_Rocker(void) {
  if (Application_SmartRobotCarxxx0.Functional_Mode == Rocker_mode) {
    unsigned int pos = AppServo.DeviceDriverSet_Servo_position(PIN_Servo_z);
    //Rocker_CarSpeed = EEPROM.read(/*address=*/0);
    switch (Application_SmartRobotCarxxx0.Motion_Control) {
      case 0 ... 1:  // F, R
        //AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, Application_SmartRobotCarxxx0.current_Speed /*is_speed*/);
        // ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, Rocker_CarSpeed /*speed*/);
        break;
      case 2:  // L
        //if (pos < 110) {
        //AppServo.DeviceDriverSet_Servo_control(pos + 10 /*Position_angle*/);
        //}
        AppServo.DeviceDriverSet_Servo_control(110 /*Position_angle*/);
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
      case 3:  // R
        //if (pos > 70) {
        //AppServo.DeviceDriverSet_Servo_control(pos - 10 /*Position_angle*/);
        //}
        AppServo.DeviceDriverSet_Servo_control(70 /*Position_angle*/);
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
      case 4:  // LF
        //if (pos < 110) {
        //  AppServo.DeviceDriverSet_Servo_control(pos + 10 /*Position_angle*/);
        //}
        //ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, Rocker_CarSpeed /*speed*/);
        AppServo.DeviceDriverSet_Servo_control(100 /*Position_angle*/);
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
      case 5:  // LR
        //if (pos < 110) {
        //  AppServo.DeviceDriverSet_Servo_control(pos + 10 /*Position_angle*/);
        //}
        //ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, Rocker_CarSpeed /*speed*/);
        AppServo.DeviceDriverSet_Servo_control(100 /*Position_angle*/);
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
      case 6:  // RF
        //if (pos > 70) {
        //  AppServo.DeviceDriverSet_Servo_control(pos - 10 /*Position_angle*/);
        //}
        //ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, Rocker_CarSpeed /*speed*/);
        AppServo.DeviceDriverSet_Servo_control(80 /*Position_angle*/);
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
      case 7:  // RR
        //if (pos > 70) {
        //AppServo.DeviceDriverSet_Servo_control(pos - 10 /*Position_angle*/);
        //}
        //ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, Rocker_CarSpeed /*speed*/);
        AppServo.DeviceDriverSet_Servo_control(80 /*Position_angle*/);
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
      case 9:  // F
        //if (pos > 70) {
        //AppServo.DeviceDriverSet_Servo_control(pos - 10 /*Position_angle*/);
        //}
        //ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, Rocker_CarSpeed /*speed*/);
        AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
      default:
        AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
    }
    //ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, Rocker_CarSpeed /*speed*/);
  }
}

/*Line tracking mode*/
void ApplicationFunctionSet::ApplicationFunctionSet_Tracking(void) {
  static boolean timestamp = true;
  static boolean BlindDetection = true;
  static unsigned long MotorRL_time = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode) {
    if (Car_LeaveTheGround == false)  //Check if the car leaves the ground
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
      return;
    }

    // int getAnaloguexxx_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    // int getAnaloguexxx_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    // int getAnaloguexxx_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
#if _Test_print
    static unsigned long print_time = 0;
    if (millis() - print_time > 500) {
      print_time = millis();
      Serial.print("ITR20001_getAnaloguexxx_L=");
      Serial.println(getAnaloguexxx_L);
      Serial.print("ITR20001_getAnaloguexxx_M=");
      Serial.println(getAnaloguexxx_M);
      Serial.print("ITR20001_getAnaloguexxx_R=");
      Serial.println(getAnaloguexxx_R);
    }
#endif
    if (function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E)) {
      /*Achieve straight and uniform speed movement*/
      AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      timestamp = true;
      BlindDetection = true;
    } else if (function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E)) {
      /*Turn right*/
      AppServo.DeviceDriverSet_Servo_control(70 /*Position_angle*/);
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      timestamp = true;
      BlindDetection = true;
    } else if (function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E)) {
      /*Turn left*/
      AppServo.DeviceDriverSet_Servo_control(110 /*Position_angle*/);
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      timestamp = true;
      BlindDetection = true;
    } else  ////The car is not on the black line. execute Blind scan
    {
      if (timestamp == true)  //acquire timestamp
      {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
      /*Blind Detection*/
      if ((function_xxx((millis() - MotorRL_time), 0, 200) || function_xxx((millis() - MotorRL_time), 1600, 2000)) && BlindDetection == true) {
        AppServo.DeviceDriverSet_Servo_control(70 /*Position_angle*/);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      } else if (((function_xxx((millis() - MotorRL_time), 200, 1600))) && BlindDetection == true) {
        AppServo.DeviceDriverSet_Servo_control(110 /*Position_angle*/);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      } else if ((function_xxx((millis() - MotorRL_time), 3000, 3500)))  // Blind Detection ...s ?
      {
        BlindDetection = false;
        //ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 100);
        AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
      }
    }
  } else if (false == timestamp) {
    BlindDetection = true;
    timestamp = true;
    MotorRL_time = 0;
  }
}

/*
  Obstacle Avoidance Mode
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void) {
  static boolean first_is = true;
  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode) {
    //uint8_t switc_ctrl = 0;
    uint16_t get_Distance;
    if (Car_LeaveTheGround == false) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    if (first_is == true)  //Enter the mode for the first time, and modulate the steering gear to 90 degrees
    {
      AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
      first_is = false;
    }

    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);
    if (function_xxx(get_Distance, 0, 50)) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

      //for (uint8_t i = 7; i < 12; i++)  //1、3、5 Omnidirectional detection of obstacle avoidance status
      //{
      AppServo.DeviceDriverSet_Servo_control(70 /* (10 * i) Position_angle*/);
      ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 100);
      delay_xxx(500);
      AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);

      if (function_xxx(get_Distance, 0, 50)) {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 100);
        delay_xxx(500);
      } else {
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        AppServo.DeviceDriverSet_Servo_control(120 /*Position_angle*/);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
        //switc_ctrl = 0;
        /*switch (i) {
            case 7:
              ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
              break;
            case 9:
              ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
              break;
            case 11:
              ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
              break;
          }*/
        delay_xxx(50);
        first_is = true;
        //break;
      }
      //}
    } else
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
    }
  } else {
    first_is = true;
  }
}

/*
  Following mode：
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Follow(void) {
  static uint16_t ULTRASONIC_Get = 0;
  static unsigned long ULTRASONIC_time = 0;
  static uint8_t Position_Servo = 1;
  static uint8_t timestamp = 3;
  static uint8_t OneCycle = 1;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Follow_mode) {

    if (Car_LeaveTheGround == false) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&ULTRASONIC_Get /*out*/);
    if (function_xxx(ULTRASONIC_Get, 0, 5))  //There is an obstacle 5 cm ahead?
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      //delay_xxx(500);
    }
    if (false == function_xxx(ULTRASONIC_Get, 0, 40))  //There is no obstacle 5-40 cm ahead?
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      static unsigned long time_Servo = 0;
      static uint8_t Position_Servo_xx = 0;

      if (timestamp == 3) {
        if (Position_Servo_xx != Position_Servo)  //Act on servo motor：avoid loop execution
        {
          Position_Servo_xx = Position_Servo;  //Act on servo motor：rotation angle record

          if (Position_Servo == 1) {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
          } else if (Position_Servo == 2) {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(70 /*Position_angle*/);
          } else if (Position_Servo == 3) {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
          } else if (Position_Servo == 4) {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(110 /*Position_angle*/);
          }
        }
      } else {
        if (timestamp == 1) {
          timestamp = 2;
          time_Servo = millis();
        }
      }
      if (millis() - time_Servo > 1000)  //Act on servo motor：stop at the current location for 2s
      {
        timestamp = 3;
        Position_Servo += 1;
        OneCycle += 1;
        if (OneCycle > 4) {
          Position_Servo = 1;
          OneCycle = 5;
        }
      }
    } else if (function_xxx(ULTRASONIC_Get, 5, 40)) {
      OneCycle = 1;
      timestamp = 1;
      if ((Position_Servo == 1)) { /*Move forward*/
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
      } else if ((Position_Servo == 2)) { /*Turn right*/
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
      } else if ((Position_Servo == 3)) {
        /*Move forward*/
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
      } else if ((Position_Servo == 4)) { /*Turn left*/
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
      }
    }
  } else {
    ULTRASONIC_Get = 0;
    ULTRASONIC_time = 0;
  }
}

/*Servo motor control*/
void ApplicationFunctionSet::ApplicationFunctionSet_Servo(uint8_t Set_Servo) {
  //static int z_angle = 9;
  //static int y_angle = 9;
  uint8_t temp_Set_Servo = Set_Servo;

  switch (temp_Set_Servo) {
    case 1 ... 5:
      {
        if (1 == temp_Set_Servo) {
          AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 11);
          //z_angle += 1;
        } else if (2 == temp_Set_Servo) {
          AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 7);
          //z_angle -= 1;
        } else if (3 == temp_Set_Servo) {
          AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 10);
          //AppServo.DeviceDriverSet_Servo2_control();
          // z_angle = 8
        } else if (4 == temp_Set_Servo) {
          AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 8);
          /*if (FastLED.getBrightness() > 0) {
            FastLED.setBrightness(0);
            FastLED.show();
          } else {
            FastLED.setBrightness(250);
            FastLED.show();
          }*/
          //z_angle = 7
        } else if (5 == temp_Set_Servo) {
          AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 9);
        }

        /*if (z_angle <= 7)  //minimum control
        {
          z_angle = 7;
        }
        if (z_angle >= 11)  //maximum control
        {
          z_angle = 11;
        }
        * /
        //AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/
        //1, /*unsigned int Position_angle*/ //z_angle);
      }
      break;
    default:
      AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 9);
      break;
  }
}
/*Standby mode*/
void ApplicationFunctionSet::ApplicationFunctionSet_Standby(void) {
  static bool is_ED = true;
  static uint8_t cout = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Standby_mode) {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    if (true == is_ED)  //Used to zero yaw raw data(Make sure the car is placed on a stationary surface!)
    {
      static unsigned long timestamp;  //acquire timestamp
      if (millis() - timestamp > 20) {
        timestamp = millis();
        if (ApplicationFunctionSet_SmartRobotCarLeaveTheGround() /* condition */) {
          cout += 1;
        } else {
          cout = 0;
        }
        if (cout > 10) {
          is_ED = false;
          AppMPU6050getdata.MPU6050_calibration();
        }
      }
    }
  }
}

/* 
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Begin:CMD
 * Graphical programming and command control module
 $ Elegoo & SmartRobot & 2020-06
*/

void ApplicationFunctionSet::CMD_inspect_xxx0(void) {
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_inspect) {
    Serial.println("CMD_inspect");
    delay100();
  }
}
/*
  N1:command
  CMD mode：Car receive the control commands from the APP<motor control command>,perform unidirectional drive of the motor
  Input parameters:     uint8_t CMD_is_MotorSelection,  Motor selection  1left motor  2right motor  0both motors
                        uint8_t CMD_is_MotorDirection,  rotation direction selection  1forward  2backward  0stop
                        uint8_t CMD_is_MotorSpeed,      motor speed  0-250
  No time limited
*/
void ApplicationFunctionSet::CMD_MotorControl_xxx0(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed) {
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl) {
    MotorControl = true;
    if (0 == is_MotorDirection) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    } else {
      switch (is_MotorSelection)  //motor selection
      {
        case 0:
          {
            is_MotorSpeed_A = is_MotorSpeed;
            is_MotorSpeed_B = is_MotorSpeed;
            if (1 == is_MotorDirection) {  //turn forward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else if (2 == is_MotorDirection) {                                     //turn backward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else {
              return;
            }
          }
          break;
        case 1:
          {
            is_MotorSpeed_A = is_MotorSpeed;
            if (1 == is_MotorDirection) {  //turn forward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else if (2 == is_MotorDirection) {                                     //turn backward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else {
              return;
            }
          }
          break;
        case 2:
          {
            is_MotorSpeed_B = is_MotorSpeed;
            if (1 == is_MotorDirection) {  //turn forward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else if (2 == is_MotorDirection) {                                     //turn backward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else {
              return;
            }
          }
          break;
        default:
          break;
      }
    }
  } else {
    if (MotorControl == true) {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}
void ApplicationFunctionSet::CMD_MotorControl_xxx0(void) {
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl) {
    MotorControl = true;
    if (0 == CMD_is_MotorDirection) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    } else {
      switch (CMD_is_MotorSelection)  //motor selection
      {
        case 0:
          {
            is_MotorSpeed_A = CMD_is_MotorSpeed;
            is_MotorSpeed_B = CMD_is_MotorSpeed;
            if (1 == CMD_is_MotorDirection) {  //turn forward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else if (2 == CMD_is_MotorDirection) {                                 //turn backward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else {
              return;
            }
          }
          break;
        case 1:
          {
            is_MotorSpeed_A = CMD_is_MotorSpeed;
            if (1 == CMD_is_MotorDirection) {  //turn forward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B / 2,
                                                     /*controlED*/ control_enable);  //Motor control
            } else if (2 == CMD_is_MotorDirection) {                                 //turn backward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                     /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B / 2,
                                                     /*controlED*/ control_enable);  //Motor control
            } else {
              return;
            }
          }
          break;
        case 2:
          {
            is_MotorSpeed_B = CMD_is_MotorSpeed;
            if (1 == CMD_is_MotorDirection) {  //turn forward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A / 2,
                                                     /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else if (2 == CMD_is_MotorDirection) {                                 //turn backward
              AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A / 2,
                                                     /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                     /*controlED*/ control_enable);  //Motor control
            } else {
              return;
            }
          }
          break;
        default:
          break;
      }
    }
  } else {
    if (MotorControl == true) {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}

static void CMD_CarControl(uint8_t is_CarDirection, uint8_t is_CarSpeed) {
  switch (is_CarDirection) {
    case 1:
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, is_CarSpeed);
      break;
    case 2:
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, is_CarSpeed);
      break;
    case 3: /*movement direction mode forward*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, is_CarSpeed);
      break;
    case 4: /*movement direction mode backward*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, is_CarSpeed);
      break;
    default:
      break;
  }
}
/*
  N2：command
  CMD mode：Receive the control commands from the APP,perform movement direction and speed control of the car
  Time limited
*/
void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint32_t is_Timer) {
  static boolean CarControl = false;
  static boolean CarControl_TE = false;  //Time stamp
  static boolean CarControl_return = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit)  //enter time-limited control mode
  {
    CarControl = true;
    if (is_Timer != 0)  //#1 if the pre-set time is not ... (zero)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_CarControl_Millis) > (is_Timer))  //check the timestamp
      {
        CarControl_TE = true;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (CarControl_return == false) {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      } else {
        CarControl_TE = false;  //There still has time left
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false) {
      CMD_CarControl(is_CarDirection, is_CarSpeed);
    }
  } else {
    if (CarControl == true) {
      CarControl_return = false;
      CarControl = false;
      Application_SmartRobotCarxxx0.CMD_CarControl_Millis = 0;
    }
  }
}

void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(void) {
  static boolean CarControl = false;
  static boolean CarControl_TE = false;  //Time stamp
  static boolean CarControl_return = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit)  //enter time-limited control mode
  {
    CarControl = true;
    if (CMD_is_CarTimer != 0)  //#1 if the pre-set time is not ... (zero)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_CarControl_Millis) > (CMD_is_CarTimer))  //check the timestamp
      {
        CarControl_TE = true;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (CarControl_return == false) {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      } else {
        CarControl_TE = false;  //There still has time left
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false) {
      CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
    }
  } else {
    if (CarControl == true) {
      CarControl_return = false;
      CarControl = false;
      Application_SmartRobotCarxxx0.CMD_CarControl_Millis = 0;
    }
  }
}
/*
  N3 command
  CMD mode：Receive the control commands from the APP,perform movement direction and speed control of the car
  No time limited
*/
void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed) {
  static boolean CarControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit)  //enter no time-limited control mode
  {
    CarControl = true;
    CMD_CarControl(is_CarDirection, is_CarSpeed);
  } else {
    if (CarControl == true) {
      CarControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(void) {
  static boolean CarControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit)  //enter no time-limited control mode
  {
    CarControl = true;
    CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
  } else {
    if (CarControl == true) {
      CarControl = false;
    }
  }
}

/*
  N4 command
  CMD mode：movement mode<motor control>
  Receive the control commands from the APP,perform motion control of the left and right motors
*/
void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(uint8_t is_Speed_L, uint8_t is_Speed_R) {
  static boolean MotorControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl_Speed) {
    MotorControl = true;
    if (is_Speed_L == 0 && is_Speed_R == 0) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    } else {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_Speed_L,
                                             /*direction_B*/ direction_just, /*speed_B*/ is_Speed_R,
                                             /*controlED*/ control_enable);  //Motor control
    }
  } else {
    if (MotorControl == true) {
      MotorControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(void) {
  static boolean MotorControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl_Speed) {
    MotorControl = true;
    if (CMD_is_MotorSpeed_L == 0 && CMD_is_MotorSpeed_R == 0) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    } else {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ CMD_is_MotorSpeed_L,
                                             /*direction_B*/ direction_just, /*speed_B*/ CMD_is_MotorSpeed_R,
                                             /*controlED*/ control_enable);  //Motor control
    }
  } else {
    if (MotorControl == true) {
      MotorControl = false;
    }
  }
}

/*
  N5:command
  CMD mode：<servo motor control>
*/
void ApplicationFunctionSet::CMD_ServoControl_xxx0(void) {
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ServoControl) {
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo*/ CMD_is_Servo, /*unsigned int Position_angle*/ CMD_is_Servo_angle / 9);
    Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
  }
}
/*
  N7:command
  CMD mode：<Lighting Control>
  Time limited：Enter programming mode after the time is over
*/
void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B,
                                                               uint32_t is_LightingTimer) {
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false;  //Time stamp
  static boolean LightingControl_return = false;

  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit)  //enter time-limited control mode
  {
    LightingControl = true;
    if (is_LightingTimer != 0)  //#1 if the pre-set time is not ... (zero)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_LightingControl_Millis) > (is_LightingTimer))  //Check the timestamp
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (LightingControl_return == false) {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      } else {
        LightingControl_TE = false;  //There still has time left
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false) {
      CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    }
  } else {
    if (LightingControl == true) {
      LightingControl_return = false;
      LightingControl = false;
      Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}

void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(void) {
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false;  //Time stamp
  static boolean LightingControl_return = false;

  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit)  //Enter Lighting Control mode with time-limited
  {
    LightingControl = true;
    if (CMD_is_LightingTimer != 0)  //#1 if the pre-set time is not ... (zero)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_LightingControl_Millis) > (CMD_is_LightingTimer))  //Check the timestamp
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (LightingControl_return == false) {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      } else {
        LightingControl_TE = false;  //There still has time left
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false) {
      CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
    }
  } else {
    if (LightingControl == true) {
      LightingControl_return = false;
      LightingControl = false;
      Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}
/*
  N8:command
  CMD mode：<Lighting control>
  No time limited
*/
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B) {
  static boolean LightingControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit)  //Enter Lighting Control mode without time-limited
  {
    LightingControl = true;
    CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
  } else {
    if (LightingControl == true) {
      LightingControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(void) {
  static boolean LightingControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit)  //Enter Lighting Control mode without time-limited
  {
    LightingControl = true;
    CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
  } else {
    if (LightingControl == true) {
      LightingControl = false;
    }
  }
}

/*
  N100/N110:command
  CMD mode：Clear all functions
*/
void ApplicationFunctionSet::CMD_ClearAllFunctions_xxx0(void) {
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Standby_mode)  //Command:N100 Clear all functions to enter standby mode
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, NUM_LEDS /*Traversal_Number*/, CRGB::Black);
    Application_SmartRobotCarxxx0.Motion_Control = stop_it;
    Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
  }
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Programming_mode)  //Command:N110 Clear all functions and enter programming mode
  {

    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, NUM_LEDS /*Traversal_Number*/, CRGB::Black);
    Application_SmartRobotCarxxx0.Motion_Control = stop_it;
    Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
  }
}

/*
  N21:command
  CMD mode：The ultrasonic module receives and feeds back the status and ranging data according to the control command of APP terminal.
  Input：
*/
void ApplicationFunctionSet::CMD_UltrasoundModuleStatus_xxx0(uint8_t is_get) {
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&UltrasoundData_cm /*out*/);  //Ultrasonic data
  UltrasoundDetectionStatus = function_xxx(UltrasoundData_cm, 0, ObstacleDetection);
  if (1 == is_get)  //ultrasonic sensor  is_get Start     true：has obstacle / false: no obstable
  {
    if (true == UltrasoundDetectionStatus) {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    } else {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }
  } else if (2 == is_get)  //ultrasonic sensor is_get data
  {
    char toString[10];
    sprintf(toString, "%d", UltrasoundData_cm);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
  }
}
/*
  N22:command
  CMD mode：Tracking module receives and feeds back tracing status and data according to the control command of APP terminal
  Input：
*/
void ApplicationFunctionSet::CMD_TraceModuleStatus_xxx0(uint8_t is_get) {
  char toString[10];
  if (0 == is_get) /*Get left IR sensor status*/
  {
    sprintf(toString, "%d", TrackingData_L);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
    /*
    if (true == TrackingDetectionStatus_L)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }*/
  } else if (1 == is_get) /*Get middle IR sensor status*/
  {
    sprintf(toString, "%d", TrackingData_M);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
    /*
    if (true == TrackingDetectionStatus_M)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }*/
  } else if (2 == is_get) /*Get right IR sensor status*/
  {
    sprintf(toString, "%d", TrackingData_R);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
    /*
        if (true == TrackingDetectionStatus_R)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }*/
  }
  Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
}

/* 
 * End:CMD
 * Graphical programming and command control module
 $ Elegoo & SmartRobot & 2020-06
 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*Key command*/
void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void) {
  uint8_t get_keyValue;
  static uint8_t temp_keyValue = keyValue_Max;
  AppKey.DeviceDriverSet_key_Get(&get_keyValue);

  if (temp_keyValue != get_keyValue) {
    temp_keyValue = get_keyValue;  //Serial.println(get_keyValue);
    switch (get_keyValue) {
      case /* constant-expression */ 1:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
        break;
      case /* constant-expression */ 2:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
        break;
      case /* constant-expression */ 3:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
        break;
      case /* constant-expression */ 4:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
      default:

        break;
    }
  }
}
/*Infrared remote control*/
void ApplicationFunctionSet::ApplicationFunctionSet_IRrecv(void) {
  uint8_t IRrecv_button;
  static bool IRrecv_en = false;
  if (AppIRrecv.DeviceDriverSet_IRrecv_Get(&IRrecv_button /*out*/)) {
    IRrecv_en = true;
    //Serial.println(IRrecv_button);
  }
  if (true == IRrecv_en) {
    switch (IRrecv_button) {
      case /* up */ 1:
        /* code */
        Application_SmartRobotCarxxx0.Motion_Control = Forward;
        break;
      case /* down */ 2:
        /* code */
        Application_SmartRobotCarxxx0.Motion_Control = Backward;
        break;
      case /* left */ 3:
        /* code */
        Application_SmartRobotCarxxx0.Motion_Control = Left;
        break;
      case /* right */ 4:
        /* code */
        Application_SmartRobotCarxxx0.Motion_Control = Right;
        break;
      case /* ok */ 5:
        /* code */
        {
          Application_SmartRobotCarxxx0.Motion_Control = stop_it;
          AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
          Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        }
        break;
      case /* (1) */ 6:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
        /*{
          if (Application_SmartRobotCarxxx0.Functional_Mode == Standby_mode) {
            Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
          } else if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode) {
            Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
          } else if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode) {
            Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
          } else if (Application_SmartRobotCarxxx0.Functional_Mode == Follow_mode) {
            Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
          }
        }*/
        break;
      case /* (2) */ 7:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
        break;
      case /* (3) */ 8:
        /* code */
        Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
        break;
      case /* (4) */ 9:
        /* code */
        {
          if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode) {
            if (TrackingDetection_S > 30) {
              TrackingDetection_S -= 10;
            }
          } else if (Rocker_CarSpeed > 50) {
            Rocker_CarSpeed -= 5;
            Application_SmartRobotCarxxx0.current_Speed = Rocker_CarSpeed;
            //EEPROM.update(/*address=*/0, /*value=*/Rocker_CarSpeed);
          }
        }
        break;
      case /* (5) */ 10:
        /* code */
        {
          if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode) {
            TrackingDetection_S = 250;
          } else {
            Rocker_CarSpeed = 250;
            Application_SmartRobotCarxxx0.current_Speed = Rocker_CarSpeed;
            //EEPROM.update(/*address=*/0, /*value=*/Rocker_CarSpeed);
          }
        }
        break;
      case /* (6) */ 11:
        /* code */
        {
          if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)  //Adjust the threshold of the line tracking module to adapt the actual environment
          {
            if (TrackingDetection_S < 600) {
              TrackingDetection_S += 10;
            }
          } else if (Rocker_CarSpeed < 255) {
            Rocker_CarSpeed += 5;
            Application_SmartRobotCarxxx0.current_Speed = Rocker_CarSpeed;
            //EEPROM.update(/*address=*/0, /*value=*/Rocker_CarSpeed);

            //uint8_t cur = EEPROM.read(/*address=*/0);
            //if (cur < 250 && cur > 0) {
            //  EEPROM.update(/*address=*/0,/*value=*/Rocker_CarSpeed);
            //} else {
            //EEPROM.write(/*address=*/0,/*value=*/Rocker_CarSpeed);
            //}
          }
        }
        break;

      case /* (7) */ 12:
        /*{
          if (Rocker_CarSpeed < 255) {
            Rocker_CarSpeed += 5;
          }
        }*/
        break;
      case /* (8) */ 13:
        /*{
          Rocker_CarSpeed = 250;
        }*/
        {
          unsigned int bright = FastLED.getBrightness();
          if (bright < 226) {
            FastLED.setBrightness(bright + 25);
            FastLED.show();
          }
        }
        break;
      case /* (9) */ 14:
        /*{
          if (Rocker_CarSpeed > 50) {
            Rocker_CarSpeed -= 5;
          }
        }*/
        break;
      case /* (*) */ 15:
        {
          if (FastLED.getBrightness() > 0) {
            FastLED.setBrightness(0);
            FastLED.show();
          } else {
            FastLED.setBrightness(250);
            FastLED.show();
          }
        }
        break;
      case /* (0) */ 16:
        {
          unsigned int bright = FastLED.getBrightness();
          if (bright > 24) {
            FastLED.setBrightness(bright - 25);
            FastLED.show();
          }
        }
        break;
      case /* (#) */ 17:
        {
          AppServo.DeviceDriverSet_Servo2_control();  // Light control on
        }
        break;
      default:
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        break;
    }
    /*achieve time-limited control on movement direction part*/
    if (IRrecv_button < 5)  //4 || Application_SmartRobotCarxxx0.Functional_Mode == Standby_mode
    {
      Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;
      if (millis() - AppIRrecv.IR_PreMillis > 300) {
        IRrecv_en = false;
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        AppIRrecv.IR_PreMillis = millis();
      }
    } else {
      IRrecv_en = false;
      AppIRrecv.IR_PreMillis = millis();
    }
  }
}

/*Data analysis on serial port*/
void ApplicationFunctionSet::ApplicationFunctionSet_SerialPortDataAnalysis(void) {
  String SerialPortData;
  uint8_t c = 0;
  //uint8_t l = 0;
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      c = Serial.read();
      if (c == '{') {
        SerialPortData = "{";
        Serial.println("");
        if (SerialPortData += Serial.readStringUntil('}')) {
          SerialPortData += "}";
          c = 1;
          Serial.println(SerialPortData);
          break;
        } else {
          //Serial.println("Error Command without Close Tag -}");
          //Serial.println(SerialPortData);
        }
      }
      SerialPortData = "";
      Serial.print(c);
      c = 0;
    }
    while (Serial.available() > 0) {
      Serial.read();
    }
    if (true == SerialPortData.equals("{Factory}") || true == SerialPortData.equals("{default}") || true == SerialPortData.equals("{WA_NO}") || true == SerialPortData.equals("{WA_OK}")) {
      SerialPortData = "";
      c = 0;
    }
    //while (c != '{' && Serial.available() > 0) {
    //  c = Serial.read();
    // }
    // if (c == '{') {
    //  SerialPortData += Serial->readStringUntil('}');
    //while (c != '}' && Serial.available() > 0) {
    //  c = Serial.read();
    //  SerialPortData += (char)c;
    //}
    // c = '}';
    // SerialPortData += (char)c;
    // }
  }
  if (c == 1) {
    StaticJsonDocument<200> doc;                                        //Declare a JsonDocument object
    DeserializationError error = deserializeJson(doc, SerialPortData);  //Deserialize JSON data from the serial data buffer
    SerialPortData = "";
    if (error) {
      Serial.println("error:deserializeJson");
    } else if (!error)  //Check if the deserialization is successful
    {
      int control_mode_N = doc["N"];
      //Get the serial number of the new command
      char *temp = doc["H"];
      CommandSerialNumber = temp;
      /*Please view the following code blocks in conjunction with the Communication protocol for Smart Robot Car.pdf*/
      switch (control_mode_N) {
        case 1: /*<Command：N 1> motor control mode */
          {
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_MotorControl;
            CMD_is_MotorSelection = doc["D1"];
            CMD_is_MotorSpeed = doc["D2"];
            CMD_is_MotorDirection = doc["D3"];
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
            break;
          }
        case 2:
          {
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_CarControl_TimeLimit; /*<Command：N 2> Car movement direction and speed control：Time limited mode*/
            CMD_is_CarDirection = doc["D1"];
            CMD_is_CarSpeed = doc["D2"];
            CMD_is_CarTimer = doc["T"];
            Application_SmartRobotCarxxx0.CMD_CarControl_Millis = millis();
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
            break;
          }
        case 3: /*<Command：N 3> Car movement direction and speed control：No time limited mode*/
          {
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_CarControl_NoTimeLimit;
            CMD_is_CarDirection = doc["D1"];
            CMD_is_CarSpeed = doc["D2"];
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
            break;
          }
        case 4: /*<Command：N 4> motor control:Control motor speed mode*/
          {
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_MotorControl_Speed;
            CMD_is_MotorSpeed_L = doc["D1"];
            CMD_is_MotorSpeed_R = doc["D2"];
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
            break;
          }
        case 5: /*<Command：N 5> servo motor control */
          {
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_ServoControl;
            CMD_is_Servo = doc["D1"];
            CMD_is_Servo_angle = doc["D2"];
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
            break;
          }
        case 7: /*<Command：N 7> Lighting control:Time limited mode. Lighting (Left, front, right, back and center)*/
          {
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_LightingControl_TimeLimit;
            CMD_is_LightingSequence = doc["D1"];
            CMD_is_LightingColorValue_R = doc["D2"];
            CMD_is_LightingColorValue_G = doc["D3"];
            CMD_is_LightingColorValue_B = doc["D4"];
            CMD_is_LightingTimer = doc["T"];
            Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = millis();
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
            break;
          }
        case 8: /*<Command：N 8> Lighting control:No time limited mode. Lighting (Left, front, right, back and center) */
          {
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_LightingControl_NoTimeLimit;
            CMD_is_LightingSequence = doc["D1"];
            CMD_is_LightingColorValue_R = doc["D2"];
            CMD_is_LightingColorValue_G = doc["D3"];
            CMD_is_LightingColorValue_B = doc["D4"];
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
            break;
          }
        case 21: /*<Command：N 21>：ultrasonic sensor: detect obstacle distance */
          {
            CMD_UltrasoundModuleStatus_xxx0(doc["D1"]);
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
            break;
          }
        case 22: /*<Command：N 22>：IR sensor：for line tracking mode */
          {
            CMD_TraceModuleStatus_xxx0(doc["D1"]);
#if _is_print
            Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
            break;
          }
#if _is_print
        case 23: /*<Command：N 23>：Check if the car leaves the ground */
          {
            if (true == Car_LeaveTheGround) {
              Serial.print('{' + CommandSerialNumber + "_false}");
            } else if (false == Car_LeaveTheGround) {
              Serial.print('{' + CommandSerialNumber + "_true}");
            }
            break;
          }
#endif
        case 110: /*<Command：N 110> Clear all function:Enter programming mode */
          {
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Programming_mode;
            break;
          }
        case 100: /*<Command：N 100> Clear all function:Enter standby mode*/
          {
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Standby_mode;
            // Application_SmartRobotCarxxx0.Motion_Control = stop_it;
            // Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
            break;
          }
        case 101: /*<Command：N 101> :remote control to switch the car mode*/
          {
            if (1 == doc["D1"]) {
              Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
            } else if (2 == doc["D1"]) {
              Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
            } else if (3 == doc["D1"]) {
              Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
            }
            break;
          }
        case 105: /*<Command：N 105> :FastLED brightness adjustment control command*/
          {
            if (1 == doc["D1"] && (CMD_is_FastLED_setBrightness < 250)) {
              CMD_is_FastLED_setBrightness += 50;
              if (CMD_is_FastLED_setBrightness > 200 && CMD_is_FastLED_setBrightness < 250) {
                CMD_is_FastLED_setBrightness = 250;
              }
              FastLED.setBrightness(CMD_is_FastLED_setBrightness);
            } else if (2 == doc["D1"] && (CMD_is_FastLED_setBrightness > 50)) {
              CMD_is_FastLED_setBrightness -= 50;
              //if (CMD_is_FastLED_setBrightness > 0 && CMD_is_FastLED_setBrightness < 50) {
              //CMD_is_FastLED_setBrightness = 0;
              //}
              FastLED.setBrightness(CMD_is_FastLED_setBrightness);
            } else if (3 == doc["D1"]) {
              AppServo.DeviceDriverSet_Servo2_control();
            } else if (4 == doc["D1"]) {
              if (FastLED.getBrightness() > 0) {
                FastLED.setBrightness(0);
                FastLED.show();
              } else {
                FastLED.setBrightness(250);
                FastLED.show();
              }
            }
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Standby_mode;
            break;
          }
        case 106: /*<Command：N 106> */
          {
            uint8_t temp_Set_Servo = doc["D1"];
            if (temp_Set_Servo > 5 || temp_Set_Servo < 1)
              return;
            ApplicationFunctionSet_Servo(temp_Set_Servo);
            //Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Standby_mode;
            break;
          }
        case 102: /*<Command：N 102> :Rocker control mode command*/
          {
            Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;
            Rocker_temp = doc["D1"];
            Rocker_CarSpeed = doc["D2"];
            if (Rocker_CarSpeed <= 250 && Rocker_CarSpeed > 0) {
              Application_SmartRobotCarxxx0.current_Speed = Rocker_CarSpeed;
              //EEPROM.update(/*address=*/0, /*value=*/Rocker_CarSpeed);
            }
            switch (Rocker_temp) {
              case 1:
                {
                  Application_SmartRobotCarxxx0.Motion_Control = Forward;
                  break;
                }
              case 2:
                {
                  Application_SmartRobotCarxxx0.Motion_Control = Backward;
                  break;
                }
              case 3:
                {
                  Application_SmartRobotCarxxx0.Motion_Control = Left;
                  break;
                }
              case 4:
                {
                  Application_SmartRobotCarxxx0.Motion_Control = Right;
                  break;
                }
              case 5:
                {
                  Application_SmartRobotCarxxx0.Motion_Control = LeftForward;
                  break;
                }
              case 6:
                {
                  Application_SmartRobotCarxxx0.Motion_Control = LeftBackward;
                  break;
                }
              case 7:
                {
                  Application_SmartRobotCarxxx0.Motion_Control = RightForward;
                  break;
                }
              case 8:
                {
                  Application_SmartRobotCarxxx0.Motion_Control = RightBackward;
                  break;
                }
              case 9:
                {
                  Application_SmartRobotCarxxx0.Motion_Control = stop_it;
                  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
                  break;
                }
              default:
                {
                  //Application_SmartRobotCarxxx0.Motion_Control = stop_it;
                  //Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
                  Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Standby_mode;
                  break;
                }
            }
            break;
          }
        default:
          {
            // Application_SmartRobotCarxxx0.Motion_Control = stop_it;
            // Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
            Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Standby_mode;
            break;
          }
      }
      //Serial.println("control_mode_N:END Not ERROR");
    }  // if not error
    //Serial.println("control_mode_N:END C");
  }  // if c
  //Serial.println("control_mode_N:END");
}
