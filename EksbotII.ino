#include <Encoder.h>
#include <EEPROM.h>
#include <Servo.h>

#define BTN_MISSION   52
#define BTN_MISSION_BIRU   50
#define BTN_CALIBRATE 48

#define motRIN1  28
#define motRIN2  30
#define ENA      13
#define motLIN3  26
#define motLIN4  24
#define ENB      4
#define encodRA 19
#define encodRB 18
#define encodLA 2
#define encodLB 3
#define STEP1_IN1  29
#define STEP1_IN2  31
#define STEP1_IN3  33
#define STEP1_IN4  35
#define STEP2_IN1  43
#define STEP2_IN2  45
#define STEP2_IN3  47 
#define STEP2_IN4  49

const float DIAMETER_RODA = 6.65;
const int ENCODER_CPR = 2500;
const float JARAK_ANTAR_RODA = 21.0;
const int STEPS_PER_REV = 2048;
const float MM_PER_REV = 4.0; 
const int STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV; 
const int STEPPER_SPEED_DELAY = 600; 
float Kp_heading = 2000.0;  
float Ki_heading = 0.0; 
float Kd_heading = 250.0;

float Kp_turn = 12.0;   
float Ki_turn = 0.5;   
float Kd_turn = 3.0;   

int MIN_TURN_SPEED = 80;
int MAX_TURN_SPEED = 150;

int BASE_SPEED = 245;
int MAX_SPEED = 255;
int MIN_SPEED = 150;
float DECEL_DISTANCE = 15.0;
float Kp;
float Kd;
float lastError = 0;
float KpA;
float KdA;
float lastErrorA = 0;
Servo myservo, myservo2;

// =====================================================================
// MISI
// =====================================================================
enum TipePerintah {
  MAJU, MUNDUR, PUTAR_KANAN, PUTAR_KIRI, 
  NAIK_DEPAN, NAIK_BELAKANG, TURUN_DEPAN, TURUN_BELAKANG, 
  FOLLOW_DEPAN, FOLLOW_BELAKANG, NAIK_SEMUA, TURUN_SEMUA,
  CAPIT_DEPANTUTUP, CAPIT_DEPANBUKA, CAPIT_BELAKANGTUTUP, CAPIT_BELAKANGBUKA,
  TUNGGU, SELESAI
};
struct LangkahMisi { TipePerintah perintah; float nilai; };

LangkahMisi daftarMisi[] = {

    // {PUTAR_KIRI, 85},
    // {FOLLOW_BELAKANG, 40},
    // {PUTAR_KIRI, 180},
    // {FOLLOW_BELAKANG, 40},
    // {PUTAR_KIRI, 180},
    // {FOLLOW_BELAKANG, 40},
    // {PUTAR_KIRI, 180},
    // {FOLLOW_BELAKANG, 40},
    // {PUTAR_KIRI, 85},

    // {NAIK_SEMUA, 1},
    // {TUNGGU, 100},
    // {TURUN_SEMUA, 1},


//JALAN
    {MAJU, 155},
    {TUNGGU, 100},
    {PUTAR_KANAN, 85},
    {TUNGGU, 200},
    {MAJU, 40},
    {TUNGGU, 100},
    {PUTAR_KANAN, 85},
    {TUNGGU, 200},
    {FOLLOW_BELAKANG, 35},
    {TUNGGU, 100},

//ANGKAT
    {NAIK_BELAKANG, 1.5},
    {TUNGGU, 100},
    {CAPIT_BELAKANGTUTUP},    
    {TUNGGU, 100},

//AMBIL OBJEK SELANJUTNYA
    {PUTAR_KIRI, 180},
    {TUNGGU, 400},
    {MAJU, 10},
    {TUNGGU, 100},
    {FOLLOW_BELAKANG, 38}, // DARI 25
    {TUNGGU, 100},
    {PUTAR_KANAN, 85},
    {TUNGGU, 200},
    {MAJU, 39},
    {TUNGGU, 100},
    {PUTAR_KIRI, 85},
    {TUNGGU, 200},
    {MAJU, 27}, //DARI 24
    {TUNGGU, 100},

//ANGKAT 2
    {NAIK_DEPAN, 1.5},
    {CAPIT_DEPANTUTUP},    
    {NAIK_SEMUA, 1.5},
    {TUNGGU, 100},

// MAJU DIKIT
    {MAJU, 5},
    {TUNGGU, 100},

// NYEBRANG
    {FOLLOW_BELAKANG, 32},
    {TUNGGU, 100},
    {MUNDUR, 134},
    {TUNGGU, 100},
    {PUTAR_KANAN, 85},
    {TUNGGU, 200},
    {MUNDUR, 10},
    {TUNGGU, 100},
    {PUTAR_KIRI, 85},
    {TUNGGU, 200},
    {FOLLOW_BELAKANG, 34},
    {TUNGGU, 100},


//TURUNIN OBJEK PERTAMA
    {CAPIT_BELAKANGBUKA},    
    {TUNGGU, 100},
    {TURUN_BELAKANG, 0.75},
    {TUNGGU, 200},

//JALAN
    {MAJU, 30},
    {TUNGGU, 100},
    {PUTAR_KIRI, 85},
    {TUNGGU, 200},
    {MAJU, 38},
    {TUNGGU, 100},
    {PUTAR_KIRI, 82},
    {TUNGGU, 200},
    {MAJU, 29},
    {TUNGGU, 100},

//TARUH KEDUA
    {CAPIT_DEPANBUKA},    
    {TUNGGU, 100},
    {TURUN_DEPAN, 0.75},
    {TUNGGU, 100},

// //JALAN
    {MUNDUR, 6},
    {TUNGGU, 100},
    {FOLLOW_BELAKANG, 23},
    {TUNGGU, 100},
    {MUNDUR, 160},
    {TUNGGU, 100},
    {PUTAR_KIRI, 80},
    {TUNGGU, 200},
    {MUNDUR, 25},
    {TUNGGU, 100},
    {PUTAR_KANAN, 80},

    {TURUN_SEMUA, 2.25},

    {SELESAI, 0}
};

LangkahMisi daftarMisiBiru[] = {

    // {FOLLOW_BELAKANG, 80},
    // {TUNGGU, 100},
    // {PUTAR_KIRI, 180},
    // {TUNGGU, 400},
    // {FOLLOW_BELAKANG, 80},
    // {TUNGGU, 100},
    // {PUTAR_KANAN, 180},
    // {NAIK_DEPAN, 3},
    // {NAIK_SEMUA, 3},
    // {TURUN_SEMUA, 2.25},


//JALAN
    {MAJU, 155},
    {TUNGGU, 100},
    {PUTAR_KIRI, 82},
    {TUNGGU, 200},
    {MAJU, 37},
    {TUNGGU, 100},
    {PUTAR_KIRI, 82},
    {TUNGGU, 200},
    {FOLLOW_BELAKANG, 35},
    {TUNGGU, 100},

//ANGKAT
    {NAIK_BELAKANG, 1.5},
    {TUNGGU, 100},
    {CAPIT_BELAKANGTUTUP},    
    {TUNGGU, 100},

//AMBIL OBJEK SELANJUTNYA
    {PUTAR_KANAN, 180},
    {TUNGGU, 400},
    {MAJU, 10},
    {TUNGGU, 100},
    {FOLLOW_BELAKANG, 38}, // DARI 25
    {TUNGGU, 100},
    {PUTAR_KIRI, 83},
    {TUNGGU, 200},
    {MAJU, 38},
    {TUNGGU, 100},
    {PUTAR_KANAN, 84},
    {TUNGGU, 200},
    {MAJU, 25}, //DARI 24
    {TUNGGU, 100},

//ANGKAT 2
    {NAIK_DEPAN, 1.5},
    {CAPIT_DEPANTUTUP},    
    {NAIK_SEMUA, 1.5},
    {TUNGGU, 100},

// MAJU DIKIT
    {MAJU, 5},
    {TUNGGU, 100},

// NYEBRANG
    {FOLLOW_BELAKANG, 32},
    {TUNGGU, 100},
    {MUNDUR, 133},
    {TUNGGU, 100},
    {PUTAR_KIRI, 82},
    {TUNGGU, 200},
    {MUNDUR, 10},
    {TUNGGU, 100},
    {PUTAR_KANAN, 82},
    {TUNGGU, 200},
    {FOLLOW_BELAKANG, 32},
    {TUNGGU, 100},

// TURUNIN OBJEK PERTAMA
    {CAPIT_BELAKANGBUKA},    
    {TUNGGU, 100},
    {TURUN_BELAKANG, 0.75},
    {TUNGGU, 200},

//JALAN
    {MAJU, 30},
    {TUNGGU, 100},
    {PUTAR_KANAN, 85},
    {TUNGGU, 200},
    {MAJU, 38},
    {TUNGGU, 100},
    {PUTAR_KANAN, 86},
    {TUNGGU, 200},
    {MAJU, 29},
    {TUNGGU, 100},

//TARUH KEDUA
    {CAPIT_DEPANBUKA},    
    {TUNGGU, 100},
    {TURUN_DEPAN, 0.75},
    {TUNGGU, 100},

//JALAN
    {MUNDUR, 6},
    {TUNGGU, 100},
    {FOLLOW_BELAKANG, 23},
    {TUNGGU, 100},
    {MUNDUR, 160},
    {TUNGGU, 100},
    {PUTAR_KANAN, 80},
    {TUNGGU, 200},
    {MUNDUR, 25},
    {TUNGGU, 100},
    {PUTAR_KIRI, 80},

    {TURUN_SEMUA, 2.25},

    {SELESAI, 0}
};

// =====================================================================
// STEPPER SEQUENCE
// =====================================================================
const int stepSequence[8][4] = {{1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0}, {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}};

// =====================================================================
// GLOBAL VAR
// =====================================================================
Encoder encoderKanan(encodRA, encodRB);
Encoder encoderKiri(encodLA, encodLB);

float x_pos = 0.0, y_pos = 0.0, theta = 0.0, totalJarak = 0.0;
long lastPosKiri = 0, lastPosKanan = 0;

float error_straight = 0, lastError_straight = 0, integral_straight = 0;
float error_turn = 0, lastError_turn = 0, integral_turn = 0;

enum RobotState {DIAM, BERGERAK_LURUS, BERPUTAR, GERAK_STEPPER, KALIBRASI_PUTAR, FOLLOWING_DEPAN, FOLLOWING_BELAKANG};
RobotState currentState = DIAM;
bool missionActive = false;
int currentMissionStep = 0;

float targetDistance = 0.0, distanceMovedInTask = 0.0;
float targetAngle = 0.0, accumulatedRotation = 0.0, lastThetaForRotation = 0.0;
int moveDirection = 1;

long startEncoderKiri = 0, startEncoderKanan = 0;
int currentTurnSpeed = 0;

int currentStepperStep1 = 0, currentStepperStep2 = 0;
float forkliftHeightFront = 0.0, forkliftHeightBack = 0.0; 

int calibrationStep = 0;
long calibrationStartTime = 0;
int misiAktif = 0;

float lockedHeading = 0.0;

int bacaGarisAnalog_Depan[6];
bool bacaGarisDigital_Depan[6];
int maxGaris_Depan[6];
int minGaris_Depan[6];
int medianGaris_Depan[6];
int analogPinGaris_Depan[6] = {A5, A4, A3, A2, A1, A0};
int maxPerbandingan_Depan[6] = {0, 0, 0, 0, 0, 0};
int minPerbandingan_Depan[6] = {1000, 1000, 1000, 1000, 1000, 1000};
int state = 0;
int totalActiveA = 0;
int bobotSensorDepan = 0;
bool statusKalibrasiDepan = 0;

int bacaGarisAnalog_Belakang[6];
bool bacaGarisDigital_Belakang[6];
int maxGaris_Belakang[6];
int minGaris_Belakang[6];
int medianGaris_Belakang[6];
int analogPinGaris_Belakang[6] = {A11, A10, A9, A8, A7, A6};
int maxPerbandingan_Belakang[6] = {0, 0, 0, 0, 0, 0};
int minPerbandingan_Belakang[6] = {1000, 1000, 1000, 1000, 1000, 1000};
bool statusKalibrasiBelakang = 0;
float bobotSensorBelakang = 0;
int totalActive = 0;

// =====================================================================
// KALIBRASI
// =====================================================================
void kalibrasiSensorGaris() {
  kalibrasiGarisDepan();
  kalibrasiGarisBelakang();
}

// =====================================================================
// CAPIT
// =====================================================================
void bukaServoDepan(){
   myservo2.attach(7); 
   myservo2.write(1);             
   delay(1500);
   myservo2.detach();
}

void tutupServoDepan(){
   myservo2.attach(7); 
   myservo2.write(90);              
   delay(1500);
   myservo2.detach();
}

void bukaServoBelakang(){
   myservo.attach(6);  
   myservo.write(1);             
   delay(1500);
   myservo.detach(); 
}

void tutupServoBelakang(){
   myservo.attach(6);  
   myservo.write(90);              
   delay(1500);
   myservo.detach(); 
}

// =====================================================================
// STEPPER CONTROL
// =====================================================================
void setStepperOutput(int stepperNum, int in1, int in2, int in3, int in4) {
  if (stepperNum == 1) {
    digitalWrite(STEP1_IN1, in1);
    digitalWrite(STEP1_IN2, in2);
    digitalWrite(STEP1_IN3, in3);
    digitalWrite(STEP1_IN4, in4);
  } else if (stepperNum == 2) {
    digitalWrite(STEP2_IN1, in1);
    digitalWrite(STEP2_IN2, in2);
    digitalWrite(STEP2_IN3, in3);
    digitalWrite(STEP2_IN4, in4);
  }
}

void stepperStep(int stepperNum, bool clockwise) {
  int &currentStep = (stepperNum == 1) ? currentStepperStep1 : currentStepperStep2;
  if (clockwise) {
    currentStep++;
    if (currentStep >= 8) currentStep = 0;
  } else {
    currentStep--;
    if (currentStep < 0) currentStep = 7;
  }
  setStepperOutput(stepperNum, 
                   stepSequence[currentStep][0],
                   stepSequence[currentStep][1],
                   stepSequence[currentStep][2],
                   stepSequence[currentStep][3]);
}

void stepperOff(int stepperNum) {
  setStepperOutput(stepperNum, 0, 0, 0, 0);
}

void moveForkliftIndividual(int forkliftNum, float targetHeight_cm) {
  float &currentHeight = (forkliftNum == 1) ? forkliftHeightFront : forkliftHeightBack;
  float deltaHeight_cm = targetHeight_cm - currentHeight;
  int steps = abs(deltaHeight_cm * 10 * STEPS_PER_MM); 
  bool naik = (deltaHeight_cm > 0);
  if (forkliftNum == 1) { naik = !naik; }
  for (int i = 0; i < steps; i++) {
    stepperStep(forkliftNum, naik);
    delayMicroseconds(STEPPER_SPEED_DELAY);    
  }
  currentHeight = targetHeight_cm;
  stepperOff(forkliftNum);
}

void moveAllForklift(float targetHeightFront_cm, float targetHeightBack_cm) {
  float deltaFront = targetHeightFront_cm - forkliftHeightFront;
  float deltaBack = targetHeightBack_cm - forkliftHeightBack;
  
  long stepsFront = abs(deltaFront * 10 * STEPS_PER_MM);
  long stepsBack = abs(deltaBack * 10 * STEPS_PER_MM);
  
  bool naikFront = (deltaFront > 0);
  bool dirFront = !naikFront;
  
  bool naikBack = (deltaBack > 0);
  bool dirBack = naikBack;   
  
  long maxSteps = max(stepsFront, stepsBack);
  
  for (long i = 0; i < maxSteps; i++) {
    if (i < stepsFront) {
      stepperStep(1, dirFront);
    }
    
    if (i < stepsBack) {
      stepperStep(2, dirBack);
    }
    delayMicroseconds(STEPPER_SPEED_DELAY);    
  }
  
    forkliftHeightFront = targetHeightFront_cm;
  forkliftHeightBack = targetHeightBack_cm;
  stepperOff(1);
  stepperOff(2);
}

// =====================================================================
// LINE SENSOR
// =====================================================================
void kalibrasiGarisDepan() {
  // while (statusKalibrasiDepan == 0)
  // {
  //   for (int i = 10; i <= 15; i++)
  //   {
  //     maxPerbandingan_Depan[i] = 0;
  //     minPerbandingan_Depan[i] = 0;
  //   }
  //   statusKalibrasiDepan = 1;
  // }

  // for (int i = 0; i <= 5; i++)
  // {
  //   bacaGarisAnalog_Depan[i] = analogRead(analogPinGaris_Depan[i]);
  //   maxGaris_Depan[i] = max(maxPerbandingan_Depan[i], bacaGarisAnalog_Depan[i]);
  //   maxPerbandingan_Depan[i] = maxGaris_Depan[i];

  //   minGaris_Depan[i] = min(minPerbandingan_Depan[i], bacaGarisAnalog_Depan[i]);
  //   minPerbandingan_Depan[i] = minGaris_Depan[i];

  //   if (i == 0)
  //   {
  //     medianGaris_Depan[i] = ((maxGaris_Depan[i] - minGaris_Depan[i]) * 0.5) + minGaris_Depan[i];
  //   }
  //   else if (i == 1)
  //   {
  //     medianGaris_Depan[i] = ((maxGaris_Depan[i] - minGaris_Depan[i]) * 0.62) + minGaris_Depan[i];
  //   }
  //   else if (i == 2)
  //   {
  //     medianGaris_Depan[i] = ((maxGaris_Depan[i] - minGaris_Depan[i]) * 0.6) + minGaris_Depan[i];
  //   }
  //   else if (i == 3)
  //   {
  //     medianGaris_Depan[i] = ((maxGaris_Depan[i] - minGaris_Depan[i]) * 0.6) + minGaris_Depan[i];
  //   }
  //   else if (i == 4)
  //   {
  //     medianGaris_Depan[i] = ((maxGaris_Depan[i] - minGaris_Depan[i]) * 0.81) + minGaris_Depan[i];
  //   } // 200 putih, 170 antara 5 dan 6, 140 hitam semua, 117 antara 4 dan 5 (+-
  //   else if (i == 5)
  //   {
  //     medianGaris_Depan[i] = ((maxGaris_Depan[i] - minGaris_Depan[i]) * 0.875) + minGaris_Depan[i];
  //   } //199 putih, 192 antara hitam putih, 183 hitam semua, 167 antara 5 ama 6


  //   if (medianGaris_Depan[i] < 100)
  //   {
  //     EEPROM.write(i * 3 + 0, 0);
  //     EEPROM.write(i * 3 + 1, (medianGaris_Depan[i] / 10));
  //     EEPROM.write(i * 3 + 2, (medianGaris_Depan[i] - ((medianGaris_Depan[i] / 10) * 10)));
  //   }
  //   else
  //   {
  //     EEPROM.write(i * 3 + 0, (medianGaris_Depan[i] / 100));
  //     EEPROM.write(i * 3 + 1, ((medianGaris_Depan[i] / 10) - ((medianGaris_Depan[i] / 100) * 10)));
  //     EEPROM.write(i * 3 + 2, ((medianGaris_Depan[i]) - ((medianGaris_Depan[i] / 10) * 10)));
  //   }
  // }
}

void membacaGarisDepan() {
  // statusKalibrasiDepan = 0;
  // for (int i = 0; i <= 5; i++)
  // {
  //   bacaGarisAnalog_Depan[i] = analogRead(analogPinGaris_Depan[i]); // baca nilai pwm biasa
  // //   bacaGarisAnalog_Depan[i] = analogRead(analogPinGaris_Depan[i]); // baca nilai pwm biasa
  //   if (bacaGarisAnalog_Depan[i] >= medianGaris_Depan[i])
  //   {
  //     bacaGarisDigital_Depan[i] = 0; // Deteksi warna abu abu
  //   }
  //   else
  //   {
  //     bacaGarisDigital_Depan[i] = 1; // Deteksi warna hitam
  //   }

  //   if (bacaGarisDigital_Depan[3] == 1) {
  //     bacaGarisDigital_Depan[5] = 0;
  //   }
  //   if (bacaGarisDigital_Depan[2] == 1) {
  //     bacaGarisDigital_Depan[4] = 0;
  //     bacaGarisDigital_Depan[5] = 0;
  //   }
  //   for (int i = 0; i <= 5; i++) {
  //     Serial.print(String(bacaGarisDigital_Depan[i]) + " = ");
  //     Serial.print(String(bacaGarisAnalog_Depan[i]) + " / ");
  //     Serial.print(String(medianGaris_Depan[i]) + " || ");
  //   }
  //   Serial.println("\n=====================================\n");
  // }
}

void lineFollowingDepan() {
  // KpA = 8;  //8   8.25
  // KdA = 0;  //5   5
  // bobotSensorDepan = (-6 * bacaGarisDigital_Depan[0]) + (-3.5 * bacaGarisDigital_Depan[1]) + (-0.4 * bacaGarisDigital_Depan[2]) + (0.4 * bacaGarisDigital_Depan[3]) + (3.5 * bacaGarisDigital_Depan[4]) + (6 * bacaGarisDigital_Depan[5]);
  // totalActiveA = (bacaGarisDigital_Depan[0] + bacaGarisDigital_Depan[1] + bacaGarisDigital_Depan[2] + bacaGarisDigital_Depan[3] + bacaGarisDigital_Depan[4] + bacaGarisDigital_Depan[5]);

  // if (totalActiveA == 0)
  // {
  //   totalActiveA = 1; // Mencegah pembagian dengan nilai 0
  // }

  // Serial.println("bobot : " + String(bobotSensorDepan) + "| aktif : " + String(totalActiveA));

  // float errorA = bobotSensorDepan / totalActiveA;
  // float derivativeA = errorA - lastErrorA;
  // float outputA = (KpA * errorA) + (KdA * derivativeA);

  // float baseSpeed = 88; // pertimbangkan nilai minimal speed si robot
  // float leftSpeed = baseSpeed - outputA;
  // float rightSpeed = baseSpeed + outputA;

  // leftSpeed = constrain(leftSpeed, 0, 255);
  // rightSpeed = constrain(rightSpeed, 0, 255);
  // gerakkanMotor(motRIN1, motRIN2, ENA, leftSpeed);
  // gerakkanMotor(motLIN3, motLIN4, ENB, rightSpeed);

  // lastErrorA = errorA;
}

void kalibrasiGarisBelakang(){
  while (statusKalibrasiBelakang == 0)
  {
    for (int i = 10; i <= 15; i++)
    {
      maxPerbandingan_Belakang[i] = 0;
      minPerbandingan_Belakang[i] = 0;
    }
    statusKalibrasiBelakang = 1;
  }

  for (int i = 0; i <= 5; i++)
  {
    bacaGarisAnalog_Belakang[i] = analogRead(analogPinGaris_Belakang[i]);
    maxGaris_Belakang[i] = max(maxPerbandingan_Belakang[i], bacaGarisAnalog_Belakang[i]);
    maxPerbandingan_Belakang[i] = maxGaris_Belakang[i];

    minGaris_Belakang[i] = min(minPerbandingan_Belakang[i], bacaGarisAnalog_Belakang[i]);
    minPerbandingan_Belakang[i] = minGaris_Belakang[i];

    if (i == 1)
    {
      medianGaris_Belakang[i] = ((maxGaris_Belakang[i] - minGaris_Belakang[i]) * 0.725) + minGaris_Belakang[i];
    }
    else if (i == 2) {
      medianGaris_Belakang[i] = ((maxGaris_Belakang[i] - minGaris_Belakang[i]) * 0.75) + minGaris_Belakang[i];
    }
    else{
      medianGaris_Belakang[i] = ((maxGaris_Belakang[i] - minGaris_Belakang[i]) * 0.6) + minGaris_Belakang[i];
    }

    // if (i == 0)
    // {
    //   medianGaris_Belakang[i] = ((maxGaris_Belakang[i] - minGaris_Belakang[i]) * 0.6) + minGaris_Belakang[i];
    // }
    // else if (i == 1) {
    //   medianGaris_Belakang[i] = ((maxGaris_Belakang[i] - minGaris_Belakang[i]) * 0.65) + minGaris_Belakang[i];
    // }
    // else if (i == 2) {
    //   medianGaris_Belakang[i] = ((maxGaris_Belakang[i] - minGaris_Belakang[i]) * 0.75) + minGaris_Belakang[i];
    // }
    // else if (i == 3) {
    //   medianGaris_Belakang[i] = ((maxGaris_Belakang[i] - minGaris_Belakang[i]) * 0.55) + minGaris_Belakang[i];
    // }
    // else if (i == 4)
    // {
    //   medianGaris_Belakang[i] = ((maxGaris_Belakang[i] - minGaris_Belakang[i]) * 0.675) + minGaris_Belakang[i];
    // }
    // else if (i == 5) {
    //   medianGaris_Belakang[i] = ((maxGaris_Belakang[i] - minGaris_Belakang[i]) * 0.475) + minGaris_Belakang[i];
    // }

    if (medianGaris_Belakang[i] < 100)
    {
      EEPROM.write(20 + (i * 3) + 0, 0);
      EEPROM.write(20 + (i * 3) + 1, (medianGaris_Belakang[i] / 10));
      EEPROM.write(20 + (i * 3) + 2, (medianGaris_Belakang[i] - ((medianGaris_Belakang[i] / 10) * 10)));
    }
    else
    {
      EEPROM.write(20 + (i * 3) + 0, (medianGaris_Belakang[i] / 100));
      EEPROM.write(20 + (i * 3) + 1, ((medianGaris_Belakang[i] / 10) - ((medianGaris_Belakang[i] / 100) * 10)));
      EEPROM.write(20 + (i * 3) + 2, ((medianGaris_Belakang[i]) - ((medianGaris_Belakang[i] / 10) * 10)));
    }
  }
}

void membacaGarisBelakang() {
  statusKalibrasiBelakang = 0;
  for (int i = 0; i <= 5; i++)
  {
    bacaGarisAnalog_Belakang[i] = analogRead(analogPinGaris_Belakang[i]); // baca nilai pwm biasa
    if (bacaGarisAnalog_Belakang[i] >= medianGaris_Belakang[i])
    {
      bacaGarisDigital_Belakang[i] = 0; // Deteksi warna abu abu
    }
    else
    {
      bacaGarisDigital_Belakang[i] = 1; // Deteksi warna hitam
    }
    Serial.print(String(bacaGarisDigital_Belakang[i]) + " = ");
    Serial.print(String(bacaGarisAnalog_Belakang[i]) + " / ");
    Serial.print(String(medianGaris_Belakang[i]) + " || ");
  }
  Serial.println("\n=====================================\n");
}

void lineFollowingBelakang(){
  // Kp = 7.5;  //8   8.25
  Kd = 9;  //8
  Kp = 16;   //16
  // Kd = 5;
// kiri -6, -4.5, -3, -0.265, 4.75, 6 kanan

  bobotSensorBelakang = (-6 * bacaGarisDigital_Belakang[0]) + (-4.5 * bacaGarisDigital_Belakang[1]) + (-2.5 * bacaGarisDigital_Belakang[2]) + (0.25 * bacaGarisDigital_Belakang[3]) + (4.75 * bacaGarisDigital_Belakang[4]) + (6 * bacaGarisDigital_Belakang[5]);
  totalActive = (bacaGarisDigital_Belakang[0] + bacaGarisDigital_Belakang[1] + bacaGarisDigital_Belakang[2] + bacaGarisDigital_Belakang[3] + bacaGarisDigital_Belakang[4] + bacaGarisDigital_Belakang[5]);

  if (totalActive == 0)
  {
    totalActive = 1;
  }

  Serial.println("bobot : " + String(bobotSensorBelakang) + "| aktif : " + String(totalActive));

  float error = bobotSensorBelakang / totalActive;
  float derivative = error - lastError;
  float output = (Kp * error) + (Kd * derivative);

  float baseSpeed = 87;
  float leftSpeed = baseSpeed - output;
  float rightSpeed = baseSpeed + output;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  gerakkanMotor(motRIN2, motRIN1, ENA, leftSpeed);
  gerakkanMotor(motLIN4, motLIN3, ENB, rightSpeed);

  lastError = error;
}

// =====================================================================
// MOTOR DC CONTROL FUNCTIONS
// =====================================================================
void gerakkanMotor(int pinIN1, int pinIN2, int pinPWM, int kecepatan) {
    if (kecepatan > 0) {
        digitalWrite(pinIN1, HIGH); 
        digitalWrite(pinIN2, LOW);
    } else if (kecepatan < 0) {
        digitalWrite(pinIN1, LOW); 
        digitalWrite(pinIN2, HIGH);
    } else {
        digitalWrite(pinIN1, LOW); 
        digitalWrite(pinIN2, LOW);
    }
    analogWrite(pinPWM, abs(kecepatan));
}
void berhenti() {
    gerakkanMotor(motRIN1, motRIN2, ENA, 0);
    gerakkanMotor(motLIN3, motLIN4, ENB, 0);
}


// =====================================================================
// PID CONTROL FUNCTIONS
// =====================================================================
void resetPID_straight() {
    error_straight = 0;
    lastError_straight = 0;
    integral_straight = 0;
}

void resetPID_turn() {
    error_turn = 0;
    lastError_turn = 0;
    integral_turn = 0;
}

void resetPID_line() {
  lastErrorA = 0;
  lastError = 0;
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);

  pinMode(BTN_MISSION, INPUT_PULLUP);
  pinMode(BTN_MISSION_BIRU, INPUT_PULLUP);
  pinMode(BTN_CALIBRATE, INPUT_PULLUP);

  for (int i = 0; i <= 5; i++) {
    // Baca EEPROM untuk sensor DEPAN
    medianGaris_Depan[i] = EEPROM.read(0 + (i * 3) + 0);
    medianGaris_Depan[i] = (medianGaris_Depan[i] * 10) + EEPROM.read(0 + (i * 3) + 1);
    medianGaris_Depan[i] = (medianGaris_Depan[i] * 10) + EEPROM.read(0 + (i * 3) + 2);

    // Baca EEPROM untuk sensor BELAKANG
    medianGaris_Belakang[i] = EEPROM.read(20 + (i * 3) + 0);
    medianGaris_Belakang[i] = (medianGaris_Belakang[i] * 10) + EEPROM.read(20 + (i * 3) + 1);
    medianGaris_Belakang[i] = (medianGaris_Belakang[i] * 10) + EEPROM.read(20 + (i * 3) + 2);
  }

  // Setup DC motor
  pinMode(motRIN1, OUTPUT);
  pinMode(motRIN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(motLIN3, OUTPUT);
  pinMode(motLIN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Setup stepper 1 
  pinMode(STEP1_IN1, OUTPUT);
  pinMode(STEP1_IN2, OUTPUT);
  pinMode(STEP1_IN3, OUTPUT);
  pinMode(STEP1_IN4, OUTPUT);
  
  // Setup stepper 2 
  pinMode(STEP2_IN1, OUTPUT);
  pinMode(STEP2_IN2, OUTPUT);
  pinMode(STEP2_IN3, OUTPUT);
  pinMode(STEP2_IN4, OUTPUT);
  
  stepperOff(1);
  stepperOff(2);
  
  encoderKiri.write(0);
  encoderKanan.write(0);

  bukaServoDepan();
  bukaServoBelakang();

}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  // --- ODOMETRY ---
  long posKiri = encoderKiri.read();
  long posKanan = encoderKanan.read();
  long deltaPosKiri = posKiri - lastPosKiri;
  long deltaPosKanan = posKanan - lastPosKanan;
  lastPosKiri = posKiri;
  lastPosKanan = posKanan;
  
  float kelilingRoda = PI * DIAMETER_RODA;
  float deltaJarakKiri = ((float)deltaPosKiri / ENCODER_CPR) * kelilingRoda;
  float deltaJarakKanan = ((float)deltaPosKanan / ENCODER_CPR) * kelilingRoda;
  float deltaJarak = (deltaJarakKiri + deltaJarakKanan) / 2.0;
  float deltaTheta = (deltaJarakKanan - deltaJarakKiri) / JARAK_ANTAR_RODA;
  
  x_pos += deltaJarak * cos(theta);
  y_pos += deltaJarak * sin(theta);
  theta += deltaTheta;

  if (currentState != DIAM) {
    totalJarak += abs(deltaJarak);
  }
  
  // --- STATE MACHINE ---
  switch(currentState) {    
    case DIAM:
      berhenti();

      // Cek tombol Misi MERAH (pin 52)
      if (digitalRead(BTN_MISSION) == LOW && !missionActive) {
        delay(50);
        if (digitalRead(BTN_MISSION) == LOW) {
          missionActive = true;
          currentMissionStep = 0;
          misiAktif = 1;
          delay(500); 
        }
      } 
      
      // Cek tombol Misi BIRU (pin 50)
      if (digitalRead(BTN_MISSION_BIRU) == LOW && !missionActive) {
        delay(50);
        if (digitalRead(BTN_MISSION_BIRU) == LOW) {
          missionActive = true;
          currentMissionStep = 0;
          misiAktif = 2;
          delay(500); 
        }
      } 
      
      // Cek tombol Kalibrasi (pin 48)
      if (digitalRead(BTN_CALIBRATE) == LOW) {
        delay(50);
        if (digitalRead(BTN_CALIBRATE) == LOW) {
          Serial.println("Kalibrasi: Mereset nilai...");
          for (int i = 0; i <= 5; i++) {
            maxPerbandingan_Depan[i] = 0;
            minPerbandingan_Depan[i] = 1023; 
            maxPerbandingan_Belakang[i] = 0;
            minPerbandingan_Belakang[i] = 1023; 
          }
          currentState = KALIBRASI_PUTAR; 
          delay(500); 
        }
      }
      
      if (missionActive) {
        
        LangkahMisi* misiSaatIni;
        int ukuranMisi;

        if (misiAktif == 1) { // Misi MERAH
          misiSaatIni = daftarMisi;
          ukuranMisi = sizeof(daftarMisi)/sizeof(daftarMisi[0]);
        } else if (misiAktif == 2) { // Misi BIRU
          misiSaatIni = daftarMisiBiru;
          ukuranMisi = sizeof(daftarMisiBiru)/sizeof(daftarMisiBiru[0]);
        } else {
          missionActive = false;
          break;
        }

        if (currentMissionStep >= ukuranMisi) {
          missionActive = false;
          misiAktif = 0; // Reset pilihan misi
          break;
        }
        
        LangkahMisi &misi = misiSaatIni[currentMissionStep]; // Ambil misi dari pointer
        
        switch(misi.perintah) {
          case MAJU:
            targetDistance = misi.nilai; distanceMovedInTask = 0.0; moveDirection = 1;
            startEncoderKiri = posKiri; startEncoderKanan = posKanan;
            lockedHeading = theta;
            resetPID_straight(); currentState = BERGERAK_LURUS;
            break;
          case MUNDUR:
            targetDistance = misi.nilai; distanceMovedInTask = 0.0; moveDirection = -1;
            startEncoderKiri = posKiri; startEncoderKanan = posKanan;
            lockedHeading = theta;  
            resetPID_straight(); currentState = BERGERAK_LURUS;
            break;
          case PUTAR_KANAN:
            targetAngle = -misi.nilai; accumulatedRotation = 0.0; lastThetaForRotation = theta;
            resetPID_turn(); currentState = BERPUTAR; currentTurnSpeed = 0;
            break;
          case PUTAR_KIRI:
            targetAngle = misi.nilai; accumulatedRotation = 0.0; lastThetaForRotation = theta;
            resetPID_turn(); currentState = BERPUTAR; currentTurnSpeed = 0;
            break;
          case NAIK_DEPAN: moveForkliftIndividual(1, forkliftHeightFront + misi.nilai); currentMissionStep++; break;
          case NAIK_BELAKANG: moveForkliftIndividual(2, forkliftHeightBack + misi.nilai); currentMissionStep++; break;
          case TURUN_DEPAN: moveForkliftIndividual(1, forkliftHeightFront - misi.nilai); currentMissionStep++; break;
          case TURUN_BELAKANG: moveForkliftIndividual(2, forkliftHeightBack - misi.nilai); currentMissionStep++; break;

          case NAIK_SEMUA: 
            moveAllForklift(forkliftHeightFront + misi.nilai, forkliftHeightBack + misi.nilai); 
            currentMissionStep++; 
            break;
          
          case TURUN_SEMUA: 
            moveAllForklift(forkliftHeightFront - misi.nilai, forkliftHeightBack - misi.nilai); 
            currentMissionStep++; 
            break;

          case FOLLOW_DEPAN:
            targetDistance = misi.nilai;      
            distanceMovedInTask = 0.0;        
            resetPID_line();                  
            currentState = FOLLOWING_DEPAN;
            break;
          case FOLLOW_BELAKANG:
            targetDistance = misi.nilai;      
            distanceMovedInTask = 0.0;        
            resetPID_line();                  
            currentState = FOLLOWING_BELAKANG;
            break;
            
          case CAPIT_DEPANBUKA: bukaServoDepan(); currentMissionStep++; break;
          case CAPIT_DEPANTUTUP: tutupServoDepan(); currentMissionStep++; break;
          case CAPIT_BELAKANGBUKA: bukaServoBelakang(); currentMissionStep++; break;
          case CAPIT_BELAKANGTUTUP: tutupServoBelakang(); currentMissionStep++; break;
            
          case TUNGGU: delay(misi.nilai); currentMissionStep++; break;
          case SELESAI: 
            missionActive = false; 
            misiAktif = 0; // Reset pilihan misi
            break;
        }
        
        if (misi.perintah != TUNGGU && misi.perintah != SELESAI && 
            misi.perintah != NAIK_DEPAN && misi.perintah != NAIK_BELAKANG && 
            misi.perintah != TURUN_DEPAN && misi.perintah != TURUN_BELAKANG &&
            misi.perintah != CAPIT_DEPANBUKA && misi.perintah != CAPIT_DEPANTUTUP &&
            misi.perintah != CAPIT_BELAKANGBUKA && misi.perintah != CAPIT_BELAKANGTUTUP
            )
        {
          currentMissionStep++;
        }
      }
      break;
      
    case BERGERAK_LURUS:
      {
        // 1. Hitung Jarak Tempuh (Odometry Jarak)
        long deltaFromStartKiri = posKiri - startEncoderKiri;
        long deltaFromStartKanan = posKanan - startEncoderKanan;
        float jarakKiri = ((float)abs(deltaFromStartKiri) / ENCODER_CPR) * kelilingRoda;
        float jarakKanan = ((float)abs(deltaFromStartKanan) / ENCODER_CPR) * kelilingRoda;
        distanceMovedInTask = (jarakKiri + jarakKanan) / 2.0;
        
        float sisaJarak = targetDistance - distanceMovedInTask;

        // 2. Cek Selesai
        if (sisaJarak <= 0.3) { 
          currentState = DIAM; 
          berhenti(); // Pastikan berhenti
          break; 
        }
        
        // 3. Base Speed (dengan Deselerasi)
        int baseSpeed = BASE_SPEED;
        if (sisaJarak < DECEL_DISTANCE) {
          baseSpeed = map(sisaJarak * 10, 0, DECEL_DISTANCE * 10, MIN_SPEED, BASE_SPEED);
          baseSpeed = constrain(baseSpeed, MIN_SPEED, BASE_SPEED);
        }
        
        // ============================================================
        // 4. HEADING CORRECTION PID
        // ============================================================
        
        float error_angle = lockedHeading - theta;
        
        if (error_angle > PI) error_angle -= 2*PI;
        if (error_angle < -PI) error_angle += 2*PI;

        float derivative_angle = error_angle - lastError_straight;
        
        float correction = (Kp_heading * error_angle) + (Kd_heading * derivative_angle);
        
        lastError_straight = error_angle;
        
        correction = constrain(correction, -100, 100); 
        
        // Logika: 
        // Jika error_angle POSITIF -> Robot menghadap ke KANAN dari target -> Perlu belok KIRI
        // Kanan dipercepat, Kiri diperlambat.
        
        int speedKanan, speedKiri;
        
        if (moveDirection == 1) { // MAJU
             // Jika miring ke Kiri (theta > locked), error negatif. Correction negatif.
             speedKanan = baseSpeed - correction;
             speedKiri  = baseSpeed + correction;
        } else { // MUNDUR
             speedKanan = baseSpeed + correction;
             speedKiri  = baseSpeed - correction;
        }
        
        speedKanan = constrain(speedKanan, MIN_SPEED, MAX_SPEED);
        speedKiri  = constrain(speedKiri, MIN_SPEED, MAX_SPEED);
        
        gerakkanMotor(motRIN1, motRIN2, ENA, moveDirection * speedKanan);
        gerakkanMotor(motLIN3, motLIN4, ENB, moveDirection * speedKiri);
      }
      break;

    case BERPUTAR:
      {
        // 1. Hitung Perubahan Sudut (Odometry)
        float deltaTheta_rad = theta - lastThetaForRotation;
        
        // Normalisasi delta sudut (agar tidak lompat saat lewat 180 derajat)
        if (deltaTheta_rad > PI) deltaTheta_rad -= 2*PI;
        if (deltaTheta_rad < -PI) deltaTheta_rad += 2*PI;
        
        // Akumulasi total putaran yang sudah dilakukan
        accumulatedRotation += deltaTheta_rad * 180.0 / PI;
        lastThetaForRotation = theta;
        
        // 2. Hitung Error (Target - Aktual)
        // TargetAngle negatif untuk kanan, positif untuk kiri
        error_turn = targetAngle - accumulatedRotation;
        
        // 3. Cek Selesai (Toleransi 1 derajat)
        if (abs(error_turn) <= 1.0) { 
          currentState = DIAM; 
          berhenti(); 
          resetPID_turn(); 
          break; 
        }
        
        // 4. Logika Integral (Hanya aktif saat dekat target < 10 derajat)
        if (abs(error_turn) < 10.0) {
          integral_turn += error_turn;
          integral_turn = constrain(integral_turn, -100, 100); 
        } else {
          integral_turn = 0;
        }
        
        // 5. Hitung PID
        float derivative_turn = error_turn - lastError_turn;
        lastError_turn = error_turn;
        
        float pidOutput = (Kp_turn * error_turn) + 
                          (Ki_turn * integral_turn) + 
                          (Kd_turn * derivative_turn);
        
        // 6. Konversi ke Kecepatan Motor
        int targetSpeed = abs(pidOutput);
        
        if (targetSpeed < MIN_TURN_SPEED) {
           targetSpeed = MIN_TURN_SPEED;
        }
        
        targetSpeed = constrain(targetSpeed, MIN_TURN_SPEED, MAX_TURN_SPEED);
        
        // 7. Akselerasi
        int acceleration_step = 10;
        if (currentTurnSpeed < targetSpeed) {
          currentTurnSpeed += acceleration_step;
          if (currentTurnSpeed > targetSpeed) currentTurnSpeed = targetSpeed;
        } else {
          currentTurnSpeed = targetSpeed;
        }

        // 8. Tentukan Arah Putar
        // Jika error positif (Target > Aktual), putar kiri (CCW)
        // Jika error negatif (Target < Aktual), putar kanan (CW)
        int turnDirection = (error_turn > 0) ? 1 : -1;
        
        // Kiri: Kanan Maju, Kiri Mundur
        // Kanan: Kanan Mundur, Kiri Maju
        gerakkanMotor(motRIN1, motRIN2, ENA, -turnDirection * currentTurnSpeed);
        gerakkanMotor(motLIN3, motLIN4, ENB, turnDirection * currentTurnSpeed);
      }
      break;

    case KALIBRASI_PUTAR:
      {
        int calibration_speed = 100; 

        kalibrasiSensorGaris(); 
        
        gerakkanMotor(motRIN1, motRIN2, ENA, -calibration_speed);
        gerakkanMotor(motLIN3, motLIN4, ENB, calibration_speed); 

        if (digitalRead(BTN_MISSION) == LOW) {
          delay(50); // Debounce
          if (digitalRead(BTN_MISSION) == LOW) {
            
            berhenti();
            Serial.println("Kalibrasi Selesai...");
                        
            currentState = DIAM;
            
            delay(500); 
          }
        }
      }
      break; 

    case FOLLOWING_DEPAN:
      {
        distanceMovedInTask += abs(deltaJarak); 
        
        if (distanceMovedInTask >= targetDistance) {
          currentState = DIAM; 
          berhenti();          
          break;               
        }
        
        membacaGarisDepan();  
        lineFollowingDepan(); 
      }
      break;

    case FOLLOWING_BELAKANG:
      {
        distanceMovedInTask += abs(deltaJarak); 
        
        if (distanceMovedInTask >= targetDistance) {
          currentState = DIAM; 
          berhenti();          
          break;               
        }
        
        membacaGarisBelakang();  
        lineFollowingBelakang(); 
      }
      break;
  }
  
  // if (currentState != GERAK_STEPPER && millis() % 100 < 15) {
  //   float theta_deg = theta * 180.0 / PI;
  //   Serial.print("X:");
  //   Serial.print(x_pos, 2);
  //   Serial.print(" Y:");
  //   Serial.print(y_pos, 2);
  //   Serial.print(" S:");
  //   Serial.print(theta_deg, 2);
  //   Serial.print(" | Step:");
  //   Serial.print(currentMissionStep);
  //   Serial.print(" State:");
  //   Serial.print(currentState);
  //   Serial.println();
  // }
  
  delay(10);
}