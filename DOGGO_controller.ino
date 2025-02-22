//#include <cQueue.h>
#include "IK.h"
#include "LSCSerial.h"
#include <math.h>

// Serial Port on Teensy 4 for communication with Hiwonder Servo Bus Controller (for Teensy 4, Serial1 = pin0 (RX); pin1 (TX))
LSCSerial servoController(Serial1);
// Serial Port on Teensy 4 for communication with Airis LLM, connected to PC or to Raspberry PI
#define SERIAL_PORT Serial

// IK constructor
IK frontLeftLeg, frontRightLeg, backLeftLeg, backRightLeg;

// LX-16A Servo Data
// you must pre-calibrate each Servo central position 500 +/- deviation at 90 degrees.
// you must limit each Servo min/max range angle in HW controller to avoid motor latch or legs damage. 
// SERVO_FACTOR 0.004188790205  convertion factor from Radians to Servo Steps, assuming Angles 0-240 degrees, servo has 1000 steps 
const float SERVO_FACTOR = 0.004188790205f;

// Leg Geometry Data, In my geometry COXA is just z_offset
#define COXA   48.0
#define FEMUR  110.0
#define TIBIA  143.0

// Leg offset from the body center in case global coordinates will be used
#define XOFF 95.0
#define YOFF 70.0
#define ZOFF 0.0

#define SPEED_MIN 100 
#define SPEED_MAX 1200
uint32_t poseMakeSpeed = 500;

unsigned long lastMoveTime = 0;
bool isMoving = false;
int stepIndex = 0;
int stepIndex2 = 1;

enum Status {WALK, MOVEL, MOVER, TURNL, TURNR, MOVEBACK, SIT, STAND, STANDH, LIEDOWN, PAWLEFT, PAWRIGHT, GETBAT};
//initial FSM controller status
Status currentStatus = STAND;

boolean doneMovement = false; 
// Переменные для хранения параметров команд
int walkStepsCount = 50;
int walkSpeed = 200;
int walkStepsDone;



void setup() {
    SERIAL_PORT.begin(115200);
    servoController.begin(9600); // Инициализация связи с контроллером на 9600 бод

    // Инициализация ног
    frontLeftLeg.begin(COXA, FEMUR, TIBIA, XOFF,  YOFF, ZOFF, LegType::FRONT_LEFT);
    frontRightLeg.begin(COXA, FEMUR, TIBIA, XOFF, -YOFF, ZOFF, LegType::FRONT_RIGHT);
    backLeftLeg.begin(COXA, FEMUR, TIBIA, -XOFF,  YOFF, ZOFF, LegType::BACK_LEFT);
    backRightLeg.begin(COXA, FEMUR, TIBIA, -XOFF, -YOFF, ZOFF, LegType::BACK_RIGHT);

    SERIAL_PORT.println("Servo controller initialized!");
    // Читаем напряжение батареи контроллера
    SERIAL_PORT.println("Reading battery voltage...");
    uint16_t voltage = servoController.getBatteryVoltage();
    SERIAL_PORT.print("Battery voltage: ");
    SERIAL_PORT.print(voltage);
    SERIAL_PORT.println(" mV");
    
    stepIndex = 0;
    stepIndex2 = 2;
    poseStand(1000);
    // inital delay for all sensor to boot
    delay(1000);
    //currentStatus = WALK;
  
}

void loop() {
  // FSM Controller
  switch(currentStatus){
    case WALK:
      walkTrotForward(walkStepsCount, walkSpeed);     
      break;
    case STAND:
      poseStand(poseMakeSpeed);
      break;
    case STANDH:
      poseStandHigh(poseMakeSpeed);
      break;
    case SIT:
      poseSit(poseMakeSpeed);
      break;
    case LIEDOWN:
      poseLieDown(poseMakeSpeed);
      break;
    case GETBAT:
      // stand before measurement to avoid falling after walking
      poseStand(poseMakeSpeed);
      doneMovement = false;
      getBat();
      break;
    case TURNL:
      break;
    case TURNR:
      break;
    case MOVEL:
      walkTrotLeft(walkStepsCount, walkSpeed); 
      break;
    case MOVER:
      walkTrotRight(walkStepsCount, walkSpeed); 
      break;
    case MOVEBACK:
      break;
    case PAWLEFT:
      poseSit(500);
      doneMovement = false;
      wavePaw(poseMakeSpeed, "LEFT", 15);
      break;
    case PAWRIGHT:
      poseSit(500);
      doneMovement = false;
      wavePaw(poseMakeSpeed, "RIGHT", 15);
      break;  
      
    default:
      // todo: default status
      
      break;   
  }


    handleSerialCommands();

}

void wavePaw(uint32_t makeSpeed, String paw, uint8_t cycles){
    float pawSequence[2][3] = {
        {105.0, 0.0, 180.0},  
        {105.0, 0.0, 200.0},      
    };
   if (doneMovement) return;  // if movement completed return
 
   if (paw == "LEFT"){
      while (cycles>0){
          auto anglesFL = frontLeftLeg.getLegAnglesLocal(pawSequence[0][0], pawSequence[0][1], pawSequence[0][2]);
          // FL Leg servos TIBIA FEMUR COXA
          int pos7 = 500 + int(anglesFL[0]/SERVO_FACTOR);
          int pos8 = 500 + int(-anglesFL[1]/SERVO_FACTOR);
          int pos9 = 500 + int(anglesFL[2]/SERVO_FACTOR);
          // FL Leg servos TIBIA FEMUR COXA
          servoController.moveServo(7, pos7, makeSpeed);
          servoController.moveServo(8, pos8, makeSpeed);
          servoController.moveServo(9, pos9, makeSpeed);
          delay(makeSpeed+20);
          anglesFL = frontLeftLeg.getLegAnglesLocal(pawSequence[1][0], pawSequence[1][1], pawSequence[1][2]);
          // FL Leg servos TIBIA FEMUR COXA
          pos7 = 500 + int(anglesFL[0]/SERVO_FACTOR);
          pos8 = 500 + int(-anglesFL[1]/SERVO_FACTOR);
          pos9 = 500 + int(anglesFL[2]/SERVO_FACTOR);
          // FL Leg servos TIBIA FEMUR COXA
          servoController.moveServo(7, pos7, makeSpeed);
          servoController.moveServo(8, pos8, makeSpeed);
          servoController.moveServo(9, pos9, makeSpeed);
          delay(makeSpeed+20);  
          cycles = cycles - 1;      
      }
      currentStatus = SIT;


   }
   else if (paw == "RIGHT"){
      while (cycles>0){
          auto anglesFR = frontRightLeg.getLegAnglesLocal(pawSequence[0][0], pawSequence[0][1], pawSequence[0][2]);
          int pos1 = 500 + int(-anglesFR[0]/SERVO_FACTOR);
          int pos2 = 500 + int(anglesFR[1]/SERVO_FACTOR);
          int pos3 = 500 + int(anglesFR[2]/SERVO_FACTOR);
          // FR Leg servos TIBIA FEMUR COXA
          servoController.moveServo(1, pos1, makeSpeed);
          servoController.moveServo(2, pos2, makeSpeed);
          servoController.moveServo(3, pos3, makeSpeed);
          delay(makeSpeed+20);
          anglesFR = frontRightLeg.getLegAnglesLocal(pawSequence[1][0], pawSequence[1][1], pawSequence[1][2]);
          pos1 = 500 + int(-anglesFR[0]/SERVO_FACTOR);
          pos2 = 500 + int(anglesFR[1]/SERVO_FACTOR);
          pos3 = 500 + int(anglesFR[2]/SERVO_FACTOR);
          // FR Leg servos TIBIA FEMUR COXA
          servoController.moveServo(1, pos1, makeSpeed);
          servoController.moveServo(2, pos2, makeSpeed);
          servoController.moveServo(3, pos3, makeSpeed);
          delay(makeSpeed+20);          
          cycles = cycles - 1;      
      }
      currentStatus = SIT;    
   }
   
}


void poseSit(uint32_t makeSpeed) {
    float sitPose[4][3] = {
        {20.0, 0.0, 280.0},  // Front Left
        {20.0, 0.0, 280.0},  // Front Right
        {35.0, 0.0, 190.0},  // Back Left
        {35.0, 0.0, 190.0}   // Back Right
    };
    moveToPose(sitPose, makeSpeed);
}

void poseStand(uint32_t makeSpeed) {
    float standPose[4][3] = {
        {0.0, 0.0, 180.0},   // Front Left
        {0.0, 0.0, 180.0},   // Front Right
        {-50.0, 0.0, 200.0},   // Back Left
        {-50.0, 0.0, 200.0}    // Back Right
    };
    moveToPose(standPose, makeSpeed);
}

void poseStandHigh(uint32_t makeSpeed) {
    float standPose[4][3] = {
        {0.0, 0.0, 270.0},   // Front Left
        {0.0, 0.0, 270.0},   // Front Right
        {-30.0, 0.0, 280.0},   // Back Left
        {-30.0, 0.0, 280.0}    // Back Right
    };
    moveToPose(standPose, makeSpeed);
}

void poseLieDown(uint32_t makeSpeed) {
    float standPose[4][3] = {
        {75.0, 0.0, 150.0},   // Front Left
        {75.0, 0.0, 150.0},   // Front Right
        {75.0, 0.0, 150.0},   // Back Left
        {75.0, 0.0, 150.0}    // Back Right
    };
    moveToPose(standPose, makeSpeed);
}


  void walkTrotForward(uint32_t walkStepsCount, uint32_t walkSpeed) {
      if (!isMoving) {
          isMoving = true;
          walkStepsDone = walkStepsCount;
          lastMoveTime = millis(); // Запоминаем время старта движения
      }
  
      if (millis() - lastMoveTime >= walkSpeed) {
          lastMoveTime = millis(); // Обновляем время старта нового движения
  
          // Координаты для разных позиций
          float fStep[4][3] = {0.0,0.0,240.0,
                               -10.0,0.0,225.0,
                               30.0,0.0,225.0,
                               -5.0,0.0,240.0};

          float xi = fStep[stepIndex][0];
          float yi = fStep[stepIndex][1];
          float zi = fStep[stepIndex][2];

          auto anglesFL = frontLeftLeg.getLegAnglesLocal(xi+50.0,yi,zi-30.0);

          int pos7 = 500 + int(anglesFL[0]/SERVO_FACTOR);
          int pos8 = 500 + int(-anglesFL[1]/SERVO_FACTOR);
          int pos9 = 500 + int(anglesFL[2]/SERVO_FACTOR);

          auto anglesBR = backRightLeg.getLegAnglesLocal(xi-50.0,yi,zi+15.0);

          int pos4 = 500 + int(-anglesBR[0]/SERVO_FACTOR);
          int pos5 = 500 + int(anglesBR[1]/SERVO_FACTOR);
          int pos12 = 500 + int(-anglesBR[2]/SERVO_FACTOR);
          
          xi = fStep[stepIndex2][0];
          yi = fStep[stepIndex2][1];
          zi = fStep[stepIndex2][2];

          auto anglesBL = backLeftLeg.getLegAnglesLocal(xi-50.0,yi,zi+15.0);

          int pos10 = 500 + int(anglesBL[0]/SERVO_FACTOR);
          int pos11 = 500 + int(-anglesBL[1]/SERVO_FACTOR);
          int pos6 = 500 + int(-anglesBL[2]/SERVO_FACTOR);
      
          auto anglesFR = frontRightLeg.getLegAnglesLocal(xi+50.0,yi,zi-30.0);   
     
          int pos1 = 500 + int(-anglesFR[0]/SERVO_FACTOR);
          int pos2 = 500 + int(anglesFR[1]/SERVO_FACTOR);
          int pos3 = 500 + int(anglesFR[2]/SERVO_FACTOR);


      
          // Moving all servos according legs group for trot
          servoController.moveServo(1, pos1, walkSpeed);
          servoController.moveServo(2, pos2, walkSpeed);
          servoController.moveServo(3, pos3, walkSpeed);
          servoController.moveServo(7, pos7, walkSpeed);
          servoController.moveServo(8, pos8, walkSpeed);
          servoController.moveServo(9, pos9, walkSpeed);

          servoController.moveServo(10, pos10, walkSpeed);
          servoController.moveServo(11, pos11, walkSpeed);
          servoController.moveServo(6, pos6, walkSpeed);
          servoController.moveServo(4, pos4, walkSpeed);
          servoController.moveServo(5, pos5, walkSpeed);
          servoController.moveServo(12, pos12, walkSpeed);
            
          stepIndex = (stepIndex + 1) % 4;    // Переключаем шаги
          stepIndex2 = (stepIndex2 + 1) % 4;  // Переключаем шаги 
          
          //SERIAL_PORT.println(walkStepsDone);
          if (walkStepsDone<=0){
            doneMovement = false;
            isMoving = false;
            currentStatus = STAND;
          }
          walkStepsDone = walkStepsDone - 1;
      }
  }


  void walkTrotLeft(uint32_t walkStepsCount, uint32_t walkSpeed) {
      if (!isMoving) {
          isMoving = true;
          walkStepsDone = walkStepsCount;
          lastMoveTime = millis(); // Запоминаем время старта движения
      }
  
      if (millis() - lastMoveTime >= walkSpeed + 50) {
          lastMoveTime = millis(); // Обновляем время старта нового движения
  
          // Координаты для разных позиций
          float fStep[4][3] = 
                   {0.0, 20.0, 220.0,     
                    0.0, -20.0, 220.0,     
                    0.0, -20.0, 200.0,    
                    0.0, 20.0, 200.0};   

          float xi = fStep[stepIndex][0];
          float yi = fStep[stepIndex][1];
          float zi = fStep[stepIndex][2];

          auto anglesFL = frontLeftLeg.getLegAnglesLocal(xi,yi,zi);

          int pos7 = 500 + int(anglesFL[0]/SERVO_FACTOR);
          int pos8 = 500 + int(-anglesFL[1]/SERVO_FACTOR);
          int pos9 = 500 + int(anglesFL[2]/SERVO_FACTOR);

          auto anglesBR = backRightLeg.getLegAnglesLocal(xi,yi,zi);

          int pos4 = 500 + int(-anglesBR[0]/SERVO_FACTOR);
          int pos5 = 500 + int(anglesBR[1]/SERVO_FACTOR);
          int pos12 = 500 + int(-anglesBR[2]/SERVO_FACTOR);
          
          xi = fStep[stepIndex2][0];
          yi = fStep[stepIndex2][1];
          zi = fStep[stepIndex2][2];

          auto anglesBL = backLeftLeg.getLegAnglesLocal(xi,yi,zi);

          int pos10 = 500 + int(anglesBL[0]/SERVO_FACTOR);
          int pos11 = 500 + int(-anglesBL[1]/SERVO_FACTOR);
          int pos6 = 500 + int(-anglesBL[2]/SERVO_FACTOR);
      
          auto anglesFR = frontRightLeg.getLegAnglesLocal(xi,yi,zi);   
     
          int pos1 = 500 + int(-anglesFR[0]/SERVO_FACTOR);
          int pos2 = 500 + int(anglesFR[1]/SERVO_FACTOR);
          int pos3 = 500 + int(anglesFR[2]/SERVO_FACTOR);


      
          // Moving all servos according legs group for trot
          servoController.moveServo(1, pos1, walkSpeed);
          servoController.moveServo(2, pos2, walkSpeed);
          servoController.moveServo(3, pos3, walkSpeed);
          servoController.moveServo(7, pos7, walkSpeed);
          servoController.moveServo(8, pos8, walkSpeed);
          servoController.moveServo(9, pos9, walkSpeed);

          servoController.moveServo(10, pos10, walkSpeed);
          servoController.moveServo(11, pos11, walkSpeed);
          servoController.moveServo(6, pos6, walkSpeed);
          servoController.moveServo(4, pos4, walkSpeed);
          servoController.moveServo(5, pos5, walkSpeed);
          servoController.moveServo(12, pos12, walkSpeed);
            
          stepIndex = (stepIndex + 1) % 4;    // Переключаем шаги
          stepIndex2 = (stepIndex2 + 1) % 4;  // Переключаем шаги 
          
          //SERIAL_PORT.println(walkStepsDone);
          if (walkStepsDone<=0){
            doneMovement = false;
            currentStatus = STAND;
          }
          walkStepsDone = walkStepsDone - 1;
      }
  }


  void walkTrotRight(uint32_t walkStepsCount, uint32_t walkSpeed) {
      if (!isMoving) {
          isMoving = true;
          walkStepsDone = walkStepsCount;
          lastMoveTime = millis(); // Запоминаем время старта движения
      }
  
      if (millis() - lastMoveTime >= walkSpeed + 50) {
          lastMoveTime = millis(); // Обновляем время старта нового движения
  
          // Координаты для разных позиций
          float fStep[4][3] = 
                   {0.0, -20.0, 220.0,     
                    0.0, 20.0, 220.0,     
                    0.0, 20.0, 200.0,    
                    0.0, -20.0, 200.0};   

          float xi = fStep[stepIndex][0];
          float yi = fStep[stepIndex][1];
          float zi = fStep[stepIndex][2];

          auto anglesFL = frontLeftLeg.getLegAnglesLocal(xi,yi,zi);

          int pos7 = 500 + int(anglesFL[0]/SERVO_FACTOR);
          int pos8 = 500 + int(-anglesFL[1]/SERVO_FACTOR);
          int pos9 = 500 + int(anglesFL[2]/SERVO_FACTOR);

          auto anglesBR = backRightLeg.getLegAnglesLocal(xi,yi,zi);

          int pos4 = 500 + int(-anglesBR[0]/SERVO_FACTOR);
          int pos5 = 500 + int(anglesBR[1]/SERVO_FACTOR);
          int pos12 = 500 + int(-anglesBR[2]/SERVO_FACTOR);
          
          xi = fStep[stepIndex2][0];
          yi = fStep[stepIndex2][1];
          zi = fStep[stepIndex2][2];

          auto anglesBL = backLeftLeg.getLegAnglesLocal(xi,yi,zi);

          int pos10 = 500 + int(anglesBL[0]/SERVO_FACTOR);
          int pos11 = 500 + int(-anglesBL[1]/SERVO_FACTOR);
          int pos6 = 500 + int(-anglesBL[2]/SERVO_FACTOR);
      
          auto anglesFR = frontRightLeg.getLegAnglesLocal(xi,yi,zi);   
     
          int pos1 = 500 + int(-anglesFR[0]/SERVO_FACTOR);
          int pos2 = 500 + int(anglesFR[1]/SERVO_FACTOR);
          int pos3 = 500 + int(anglesFR[2]/SERVO_FACTOR);


      
          // Moving all servos according legs group for trot
          servoController.moveServo(1, pos1, walkSpeed);
          servoController.moveServo(2, pos2, walkSpeed);
          servoController.moveServo(3, pos3, walkSpeed);
          servoController.moveServo(7, pos7, walkSpeed);
          servoController.moveServo(8, pos8, walkSpeed);
          servoController.moveServo(9, pos9, walkSpeed);

          servoController.moveServo(10, pos10, walkSpeed);
          servoController.moveServo(11, pos11, walkSpeed);
          servoController.moveServo(6, pos6, walkSpeed);
          servoController.moveServo(4, pos4, walkSpeed);
          servoController.moveServo(5, pos5, walkSpeed);
          servoController.moveServo(12, pos12, walkSpeed);
            
          stepIndex = (stepIndex + 1) % 4;    // Переключаем шаги
          stepIndex2 = (stepIndex2 + 1) % 4;  // Переключаем шаги 
          
          //SERIAL_PORT.println(walkStepsDone);
          if (walkStepsDone<=0){
            doneMovement = false;
            currentStatus = STAND;
          }
          walkStepsDone = walkStepsDone - 1;
      }
  }


void handleSerialCommands() {
    if (SERIAL_PORT.available() > 0) {
        String command = SERIAL_PORT.readStringUntil('\n'); 
        command.trim();

        int firstSpace = command.indexOf(' ');
        String baseCommand = (firstSpace == -1) ? command : command.substring(0, firstSpace);
        String params = (firstSpace == -1) ? "" : command.substring(firstSpace + 1);

        // Обработка команд
        if (baseCommand == "WALK") {
            if (params.length() > 0) {
                int spacePos = params.indexOf(' ');
                if (spacePos != -1) {
                    walkStepsCount = params.substring(0, spacePos).toInt();
                    walkSpeed = params.substring(spacePos + 1).toInt();
                } 
                else {
                    walkStepsCount = params.toInt();
                }
            }
            else {walkSpeed = 200; walkStepsCount = 50;} //set default minumum 6 cycles for walking = 4 steps, default speed
            isMoving = false;
            stepIndex = 0;
            stepIndex2 = 2;
            currentStatus = WALK;
        }

        else if (baseCommand == "MOVEL") {
            if (params.length() > 0) {
                int spacePos = params.indexOf(' ');
                if (spacePos != -1) {
                    walkStepsCount = params.substring(0, spacePos).toInt();
                    walkSpeed = params.substring(spacePos + 1).toInt();
                } 
                else {
                    walkStepsCount = params.toInt();
                }
            }
            else {walkSpeed = 300; walkStepsCount = 40;} //
            isMoving = false;
            stepIndex = 0;
            stepIndex2 = 2;
            currentStatus = MOVEL;
        }

        else if (baseCommand == "MOVER") {
            if (params.length() > 0) {
                int spacePos = params.indexOf(' ');
                if (spacePos != -1) {
                    walkStepsCount = params.substring(0, spacePos).toInt();
                    walkSpeed = params.substring(spacePos + 1).toInt();
                } 
                else {
                    walkStepsCount = params.toInt();
                }
            }
            else {walkSpeed = 300; walkStepsCount = 16;} //set default minumum 6 cycles for walking = 4 steps, default speed
            isMoving = false;
            stepIndex = 0;
            stepIndex2 = 2;
            currentStatus = MOVER;
        }
        
        else if (baseCommand == "STAND") {
            poseMakeSpeed = (params.length() > 0) ? params.toInt() : 1000;
            if (poseMakeSpeed<SPEED_MIN){poseMakeSpeed = SPEED_MIN;}
            if (poseMakeSpeed>SPEED_MAX){poseMakeSpeed = SPEED_MAX;}
            doneMovement = false;
            currentStatus = STAND;
        }

        else if (baseCommand == "STANDH") {
            poseMakeSpeed = (params.length() > 0) ? params.toInt() : 1000;
            if (poseMakeSpeed<SPEED_MIN){poseMakeSpeed = SPEED_MIN;}
            if (poseMakeSpeed>SPEED_MAX){poseMakeSpeed = SPEED_MAX;}
            doneMovement = false;
            currentStatus = STANDH;
        }
        else if (baseCommand == "SIT") {
            poseMakeSpeed = (params.length() > 0) ? params.toInt() : 1000;
            if (poseMakeSpeed<SPEED_MIN){poseMakeSpeed = SPEED_MIN;}
            if (poseMakeSpeed>SPEED_MAX){poseMakeSpeed = SPEED_MAX;}
            doneMovement = false;
            currentStatus = SIT;
        }
        else if (baseCommand == "LIEDOWN") {
            poseMakeSpeed = (params.length() > 0) ? params.toInt() : 1000;
            if (poseMakeSpeed<SPEED_MIN){poseMakeSpeed = SPEED_MIN;}
            if (poseMakeSpeed>SPEED_MAX){poseMakeSpeed = SPEED_MAX;}
            doneMovement = false;
            currentStatus = LIEDOWN;
        }
        else if (baseCommand == "GETBAT") {
            poseMakeSpeed = (params.length() > 0) ? params.toInt() : 1000;
            if (poseMakeSpeed<SPEED_MIN){poseMakeSpeed = SPEED_MIN;}
            if (poseMakeSpeed>SPEED_MAX){poseMakeSpeed = SPEED_MAX;}            
            doneMovement = false;
            currentStatus = GETBAT;
        }
        else if (baseCommand == "PAWLEFT") {
            poseMakeSpeed = (params.length() > 0) ? params.toInt() : 100;
            if (poseMakeSpeed<100){poseMakeSpeed = 100;}
            if (poseMakeSpeed>SPEED_MAX){poseMakeSpeed = SPEED_MAX;}
            doneMovement = false;
            currentStatus = PAWLEFT;
        }

        else if (baseCommand == "PAWRIGHT") {
            poseMakeSpeed = (params.length() > 0) ? params.toInt() : 100;   
            if (poseMakeSpeed<100){poseMakeSpeed = 100;}
            if (poseMakeSpeed>SPEED_MAX){poseMakeSpeed = SPEED_MAX;}
            doneMovement = false;
            currentStatus = PAWRIGHT;
        }
    }
}


void getBat(){
  if (doneMovement == false){
    uint16_t voltage = servoController.getBatteryVoltage();
    SERIAL_PORT.println(voltage);
    doneMovement = true;
    // stand after sending measurement
    currentStatus = STAND;
  }
}

void moveToPose(const float legPositions[4][3], uint32_t makeSpeed) {
    if (doneMovement) return;  // if movement completed return

    auto anglesFL = frontLeftLeg.getLegAnglesLocal(legPositions[0][0], legPositions[0][1], legPositions[0][2]);
    auto anglesFR = frontRightLeg.getLegAnglesLocal(legPositions[1][0], legPositions[1][1], legPositions[1][2]);
    auto anglesBL = backLeftLeg.getLegAnglesLocal(legPositions[2][0], legPositions[2][1], legPositions[2][2]);
    auto anglesBR = backRightLeg.getLegAnglesLocal(legPositions[3][0], legPositions[3][1], legPositions[3][2]);

    // acoording the physical ID's of your servos as installed on Legs
    // FL Leg servos TIBIA FEMUR COXA
    int pos7 = 500 + int(anglesFL[0]/SERVO_FACTOR);
    int pos8 = 500 + int(-anglesFL[1]/SERVO_FACTOR);
    int pos9 = 500 + int(anglesFL[2]/SERVO_FACTOR);
    // FR Leg servos TIBIA FEMUR COXA
    int pos1 = 500 + int(-anglesFR[0]/SERVO_FACTOR);
    int pos2 = 500 + int(anglesFR[1]/SERVO_FACTOR);
    int pos3 = 500 + int(anglesFR[2]/SERVO_FACTOR);
    // BL Leg servos TIBIA FEMUR COXA
    int pos10 = 500 + int(anglesBL[0]/SERVO_FACTOR);
    int pos11 = 500 + int(-anglesBL[1]/SERVO_FACTOR);
    int pos6 = 500 + int(-anglesBL[2]/SERVO_FACTOR); 
    // BR Leg servos TIBIA FEMUR COXA 
    int pos4 = 500 + int(-anglesBR[0]/SERVO_FACTOR);
    int pos5 = 500 + int(anglesBR[1]/SERVO_FACTOR);
    int pos12 = 500 + int(-anglesBR[2]/SERVO_FACTOR);
    // Moving servos according the legs grous
    // FR Leg servos TIBIA FEMUR COXA
    servoController.moveServo(1, pos1, makeSpeed);
    servoController.moveServo(2, pos2, makeSpeed);
    servoController.moveServo(3, pos3, makeSpeed);
    // FL Leg servos TIBIA FEMUR COXA
    servoController.moveServo(7, pos7, makeSpeed);
    servoController.moveServo(8, pos8, makeSpeed);
    servoController.moveServo(9, pos9, makeSpeed);
    // BL Leg servos TIBIA FEMUR COXA
    servoController.moveServo(10, pos10, makeSpeed);
    servoController.moveServo(11, pos11, makeSpeed);
    servoController.moveServo(6, pos6, makeSpeed);
    // BR Leg servos TIBIA FEMUR COXA 
    servoController.moveServo(4, pos4, makeSpeed);
    servoController.moveServo(5, pos5, makeSpeed);
    servoController.moveServo(12, pos12, makeSpeed); 

    doneMovement = true;
 
}

