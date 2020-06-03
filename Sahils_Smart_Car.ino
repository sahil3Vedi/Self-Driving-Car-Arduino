#include <MatrixMath.h>

mtx_type Weights1[3][10]=
{
  {-0.2,0.7,-0.5,-0.1,1.0,0.0,-0.6,-0.3,-0.2,-0.3},
  {0.6,-0.1,-0.3,0.7,0.2,-0.1,0.2,-0.3,0.5,0.2},
  {0.4,0.6,-0.5,-0.1,0.8,-0.9,-0.4,0.5,-0.0,-0.4}
};

mtx_type Weights2[10][15]=
{
  {0.2,0.2,-0.1,-0.0,0.1,-0.2,0.2,-0.2,-0.2,-0.3,-0.1,0.0,0.5,-0.1,0.1},
  {-0.6,-0.4,1.0,-0.7,0.4,0.1,-0.3,0.2,-0.0,0.1,0.1,-0.4,-0.3,-0.5,-0.7},
  {-0.4,-0.2,-0.3,0.2,-0.5,-0.1,0.1,0.1,-0.2,-0.0,-0.2,-0.4,-0.2,-0.1,0.5},
  {-0.5,-0.1,-0.4,-0.3,0.2,0.2,0.3,-0.1,0.3,-0.3,-0.3,0.3,-0.1,0.0,-0.1},
  {-0.2,-0.0,1.1,-0.9,-0.1,0.1,-0.9,0.8,-0.4,-0.2,-0.2,-0.4,-0.3,-0.3,-0.6},
  {-0.5,-0.5,0.3,-0.1,-0.3,-0.1,-0.5,0.7,-0.3,-0.3,-0.1,0.2,-0.6,0.2,-0.1},
  {0.1,-0.4,0.4,0.0,0.2,-0.5,-0.2,0.3,0.4,-0.4,0.5,-0.3,-0.1,-0.1,-0.3},
  {-0.0,0.3,0.2,-0.3,-0.1,0.2,-0.2,-0.4,-0.4,0.3,-0.0,0.3,-0.1,-0.2,-0.2},
  {-0.4,-0.2,0.0,-0.0,-0.0,-0.2,0.5,-0.1,0.0,0.2,-0.4,0.3,-0.1,-0.0,-0.0},
  {-0.9,-0.3,0.6,0.0,0.1,-0.5,-0.2,0.9,0.1,-0.4,0.0,-0.5,-0.8,-0.4,-0.6}
};

mtx_type Weights3[15][3]=
{
  {0.5,0.3,-0.5},
  {-0.3,0.2,0.2},
  {-0.5,1.2,-0.7},
  {-0.1,0.3,-0.7},
  {-0.5,0.0,0.3},
  {-0.4,0.3,-0.1},
  {-0.1,0.9,-0.1},
  {-0.6,0.6,-0.6},
  {0.4,0.4,-0.2},
  {0.4,0.2,-0.2},
  {-0.2,-0.2,0.3},
  {-0.4,0.0,-0.5},
  {0.2,-0.0,-0.9},
  {-0.4,-0.5,0.4},
  {1.1,0.5,-0.1}
};

mtx_type Bias1[1][10]=
{
  {0.0,0.5,0.0,0.0,0.5,-0.5,0.0,-0.0,0.0,-0.5}
};

mtx_type Bias2[1][15]=
{
  {-0.2,0.0,0.6,-0.5,0.0,0.0,-0.5,0.4,0.0,0.0,0.0,0.0,-0.1,0.0,-0.1}
};

mtx_type Bias3[1][3]=
{
  {-5,6,-5}
};

mtx_type inputActivation[1][3];
mtx_type hiddenActivation_1[1][10];
mtx_type hiddenTemp_1[1][10];
mtx_type hiddenActivation_2[1][15];
mtx_type hiddenTemp_2[1][15];
mtx_type outputActivation[1][3];
mtx_type outputTemp[1][3];


// EEPROM
//#include <EEPROM.h>
//const uint8_t memAddr_LEFT  = 100;
//const uint8_t memAddr_RIGHT = 200;

// SD Data Logging
#include <SPI.h>
#include <SD.h>
File dataLog;
const uint8_t ERROR_LED   = LED_BUILTIN;
const uint8_t SPI_CS      = 10;
const String Casual_Log   = "C_LOG.txt";
const String Training_Log = "T_LOG.txt";

// BLUETOOTH
#include <SoftwareSerial.h>
SoftwareSerial BT(11,12); // RX, TX
const uint8_t BT_Status = 4;
bool BT_STATE;
String BTR;
String LRS_BTR = "Go Straight (Default)."; // Left Right or Straight
// BT Receive
const String REC_MOVE_FORWARD   = "FWRD";
const String REC_STOP_FORWARD   = "!FWRD";

const String REC_MOVE_BACKWARD  = "BWRD";
const String REC_STOP_BACKWARD  = "!BWRD";

const String REC_MOVE_LEFT      = "LEFT";
const String REC_STOP_LEFT      = "!LEFT";

const String REC_MOVE_RIGHT     = "RIGHT";
const String REC_STOP_RIGHT     = "!RIGHT";

const String REC_STOP_ALL       = "!ALL";

const String REC_START_LOGGING  = "LOG";
const String REC_STOP_LOGGING   = "!LOG";
bool LOGData;

const String REC_START_TRAINING = "TRAIN";
const String REC_STOP_TRAINING  = "!TRAIN";
bool TRAINING;
//uint8_t autoLeftCM;
//uint8_t Temp_autoLeftCM;
//uint8_t autoRightCM;
//uint8_t Temp_autoRightCM;

const String REC_START_ADRIVE   = "ADRIVE";
const String REC_STOP_ADRIVE    = "!ADRIVE";
bool AUTO_DRIVE;

// ULTRASONIC DISTANCE Sensor
const uint8_t SF_Trig = A0;
const uint8_t SF_Echo = A1;
const uint8_t SL_Trig = A2;
const uint8_t SL_Echo = A3;
const uint8_t SR_Trig = A4;
const uint8_t SR_Echo = A5;

const float centimeterFormula = 0.017; // 0.034/2

// RELAYS
const uint8_t RL_FWRD  = 5;
const uint8_t RL_BWRD  = 6;
const uint8_t RL_LEFT  = 7;
const uint8_t RL_RIGHT = 8;

// TIMERS
unsigned long currentTime;
unsigned long previousLog;
unsigned long previousCLog;
unsigned long previousTLog;


// SD Functions
void SD_LOG()
{
  if (currentTime - previousCLog < 1000) return;
  previousCLog = currentTime;
  
  dataLog = SD.open(Casual_Log, FILE_WRITE);
  if (dataLog)
  {
    dataLog.println(F(">> Casual Data Log : "));
    dataLog.print(F("#Front Sensor Data : ")); dataLog.print(getSensor_FRONT());  dataLog.println(F(" cm"));
    dataLog.print(F("#Left Sensor Data  : ")); dataLog.print(getSensor_LEFT());  dataLog.println(F(" cm"));
    dataLog.print(F("#Right Sensor Data : ")); dataLog.print(getSensor_RIGHT()); dataLog.println(F(" cm"));
    dataLog.print(F("#User Command Received : ")); dataLog.println(LRS_BTR);
    dataLog.println();

    dataLog.close();
    digitalWrite(ERROR_LED, LOW);
    Serial.println(F("> Casual Data Log Successful!"));
    Serial.println();
  }
  else
  {
    dataLog.close();
    digitalWrite(ERROR_LED, HIGH);
    Serial.println(F("! Casual Data Log Un-successful!"));
    Serial.println();
  }
}

void TRAINING_LOG()
{
  if (currentTime - previousTLog < 200) return;
  previousTLog = currentTime;
  
  dataLog = SD.open(Training_Log, FILE_WRITE);
  if (dataLog)
  {
    dataLog.println(F(">> TRAINING DATA LOG : "));
    dataLog.print(F("#Front Sensor Data : ")); dataLog.print(getSensor_FRONT());  dataLog.println(F(" cm"));
    dataLog.print(F("#Left Sensor Data  : ")); dataLog.print(getSensor_LEFT());  dataLog.println(F(" cm"));
    dataLog.print(F("#Right Sensor Data : ")); dataLog.print(getSensor_RIGHT()); dataLog.println(F(" cm"));
    dataLog.print(F("#User Command Received : ")); dataLog.println(LRS_BTR);
    dataLog.println();

    dataLog.close();
    digitalWrite(ERROR_LED, LOW);
    Serial.println(F("> Training Data Log Successful!"));
    Serial.println();
  }
  else
  {
    dataLog.close();
    digitalWrite(ERROR_LED, HIGH);
    Serial.println(F("! Training Data Log Un-successful!"));
    Serial.println();
  }
}

// BLUETOOTH Function
void handleBluetooth(String data)
{
  if (data == REC_MOVE_LEFT)
  {
    LRS_BTR = F("Turn \"LEFT\".");
  }
  else if (data == REC_MOVE_RIGHT)
  {
    LRS_BTR = F("Turn \"RIGHT\".");
  }
  else
  {
    LRS_BTR = F("Go Straight (Default).");
  }

  if (data == REC_START_LOGGING)
  {
    LOGData = true;

    Serial.println(F("> Casual Data Logging Enabled!"));
    Serial.println();
  }
  else if (data == REC_STOP_LOGGING)
  {
    LOGData = false;

    Serial.println(F("> Casual Data Logging Disabled!"));
    Serial.println();
  }

  if (data == REC_START_TRAINING)
  {
    AUTO_DRIVE = false;
    TRAINING = true;

    moveVehicle(REC_STOP_ALL);
    moveVehicle(REC_MOVE_FORWARD);

    Serial.println(F("> TRAINING ENABLED!"));
    Serial.println();
  }
  else if (data == REC_STOP_TRAINING)
  {
    TRAINING = false;

    moveVehicle(REC_STOP_ALL);
//    if (Temp_autoLeftCM > 0)
//    {
//      autoLeftCM  = Temp_autoLeftCM;
//      EEPROM.update(memAddr_LEFT , autoLeftCM);
//    }
//    if (Temp_autoRightCM > 0)
//    {
//      autoRightCM = Temp_autoRightCM;
//      EEPROM.update(memAddr_RIGHT, autoRightCM);
//    }

    Serial.println(F("> TRAINING DISABLED!"));
    Serial.println();
  }

  if (data == REC_START_ADRIVE)
  {
    TRAINING = false;
    AUTO_DRIVE = true;

    Serial.println(F("> AUTO DRIVE ENABLED!"));
    Serial.println();
  }
  else if (data == REC_STOP_ADRIVE)
  {
    AUTO_DRIVE = false;

    moveVehicle(REC_STOP_ALL);

    Serial.println(F("> AUTO DRIVE DISABLED!"));
    Serial.println();
  }
}

// ULTRASONIC Sensor Functions
long getSensor_FRONT()
{
  digitalWrite(SF_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(SF_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(SF_Trig, LOW);

  long echoTime = pulseIn(SF_Echo, HIGH);
  
  return echoTime*centimeterFormula;
}

long getSensor_LEFT()
{
  digitalWrite(SL_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(SL_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(SL_Trig, LOW);

  long echoTime = pulseIn(SL_Echo, HIGH);
  
  return echoTime*centimeterFormula;
}

long getSensor_RIGHT()
{
  digitalWrite(SR_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(SR_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR_Trig, LOW);

  long echoTime = pulseIn(SR_Echo, HIGH);
  
  return echoTime*centimeterFormula;
}

void autoDrive_Calc(int SF, int SL, int SR)
{
  inputActivation[0][0] = SF;
  inputActivation[0][1] = SL;
  inputActivation[0][2] = SR;

  // Hidden Layer 1
  Matrix.Multiply((mtx_type*)inputActivation, (mtx_type*)Weights1 ,1,3,10, (mtx_type*)hiddenTemp_1);
  Matrix.Add((mtx_type*)hiddenTemp_1, (mtx_type*)Bias1, 1,10, (mtx_type*)hiddenActivation_1);

  // Hidden Layer 2
  Matrix.Multiply((mtx_type*)hiddenActivation_1, (mtx_type*)Weights2 ,1,10,15, (mtx_type*)hiddenTemp_2);
  Matrix.Add((mtx_type*)hiddenTemp_2, (mtx_type*)Bias2, 1,15, (mtx_type*)hiddenActivation_2);

  // Output Layer
  Matrix.Multiply((mtx_type*)hiddenActivation_2, (mtx_type*)Weights3 ,1,15,3, (mtx_type*)outputTemp);
  Matrix.Add((mtx_type*)outputTemp, (mtx_type*)Bias3, 1,3, (mtx_type*)outputActivation);
}

// Control Function
//void autoDrive(uint8_t SL, uint8_t SF, uint8_t SR)
//{
//  static bool autoStop = false;
//  if (SF < 50)
//  {
//    if (autoStop == false)
//    {
//      moveVehicle(REC_MOVE_BACKWARD);
//      delay(50);
//      moveVehicle(REC_STOP_ALL);
//      autoStop = true;
//    }
//    return;
//  }
//  if (autoStop)
//  {
//    moveVehicle(REC_MOVE_FORWARD);
//    autoStop = false;
//  }
//
//  static bool autoLeft  = false;
//  static bool autoRight = false;
//
//  if (SL <= autoLeftCM)
//  {
//    moveVehicle(REC_MOVE_LEFT);
//    autoLeft = true;
//  }
//  else if (SL > autoLeftCM && autoLeft == true)
//  {
//    moveVehicle(REC_STOP_LEFT);
//    autoLeft = false;
//  }
//}

void moveVehicle(String Direction)
{
  // FORWARD-BACKWARD
  if (Direction == REC_MOVE_FORWARD)
  {
    digitalWrite(RL_BWRD, HIGH);
    digitalWrite(RL_FWRD, LOW);
    
    Serial.println(F("> Going Forward!"));
    Serial.println();
  }
  else if (Direction == REC_STOP_FORWARD)
  {
    digitalWrite(RL_FWRD, HIGH);
    
    Serial.println(F("> Not Going Forward!"));
    Serial.println();
  }

  if (Direction == REC_MOVE_BACKWARD)
  {
    digitalWrite(RL_FWRD, HIGH);
    digitalWrite(RL_BWRD, LOW);
    
    Serial.println(F("> Going Backward!"));
    Serial.println();
  }
  else if (Direction == REC_STOP_BACKWARD)
  {
    digitalWrite(RL_BWRD, HIGH);
    
    Serial.println(F("> Not Going Backward!"));
    Serial.println();
  }

  
  // LEFT-RIGHT
  if (Direction == REC_MOVE_LEFT)
  {
    digitalWrite(RL_RIGHT, HIGH);
    digitalWrite(RL_LEFT,  LOW);
    
    Serial.println(F("> Turning Left!"));
    Serial.println();
  }
  else if (Direction == REC_STOP_LEFT)
  {
    digitalWrite(RL_LEFT, HIGH);
    
    Serial.println(F("> Not Turning Left!"));
    Serial.println();
  }

  if (Direction == REC_MOVE_RIGHT)
  {
    digitalWrite(RL_LEFT,  HIGH);
    digitalWrite(RL_RIGHT, LOW);
    
    Serial.println(F("> Turning Right!"));
    Serial.println();
  }
  else if (Direction == REC_STOP_RIGHT)
  {
    digitalWrite(RL_RIGHT, HIGH);
    
    Serial.println(F("> Not Turning Right!"));
    Serial.println();
  }

  // STOP ALL
  if (Direction == REC_STOP_ALL)
  {
    digitalWrite(RL_FWRD,  HIGH);
    digitalWrite(RL_BWRD,  HIGH);
    digitalWrite(RL_LEFT,  HIGH);
    digitalWrite(RL_RIGHT, HIGH);
    
    Serial.println(F("> Stopped ALL!"));
    Serial.println();
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println();

//  autoLeftCM  = EEPROM.read(memAddr_LEFT);
//  autoRightCM = EEPROM.read(memAddr_RIGHT);

  pinMode(ERROR_LED, OUTPUT);
  pinMode(SPI_CS,    OUTPUT);
  if (SD.begin(SPI_CS))
  {
    digitalWrite(ERROR_LED, LOW);
    Serial.println(F("> SD Card Initialized!"));
  }
  else
  {
    digitalWrite(ERROR_LED, HIGH);
    Serial.println(F("! Couldn't Initialize the SD Card, Check the Connection!"));
  }
  Serial.println();

// CASUAL LOG FILE
  if (SD.exists(Casual_Log))
  {
    if (SD.remove(Casual_Log))
    {
      digitalWrite(ERROR_LED, LOW);
      Serial.println(F("> Existing Casual Log File Deleted!"));
    }
  }

  if (!SD.exists(Casual_Log))
  {
    Serial.println(F("> Creating New Casual Log File!"));
    dataLog = SD.open(Casual_Log, FILE_WRITE);
    if (dataLog)
    {
      dataLog.println("##### SMART CAR DATA-LOG #####");
      dataLog.println();
      dataLog.close();

      digitalWrite(ERROR_LED, LOW);
      Serial.print(F("> New Log File Created with Name "));
      Serial.println(Casual_Log);
    }
    else
    {
      digitalWrite(ERROR_LED, HIGH);
      Serial.println(F("! Failed Creating Casual Log File."));
    }
  }
  Serial.println();

// TRAINING LOG FILE
  if (SD.exists(Training_Log))
  {
    if (SD.remove(Training_Log))
    {
      digitalWrite(ERROR_LED, LOW);
      Serial.println(F("> Existing Training Log File Deleted!"));
    }
  }

  if (!SD.exists(Training_Log))
  {
    Serial.println(F("> Creating New Training Log File!"));
    dataLog = SD.open(Training_Log, FILE_WRITE);
    if (dataLog)
    {
      dataLog.println("##### SMART CAR DATA-LOG #####");
      dataLog.println();
      dataLog.close();

      digitalWrite(ERROR_LED, LOW);
      Serial.print(F("> New Log File Created with Name "));
      Serial.println(Training_Log);
    }
    else
    {
      digitalWrite(ERROR_LED, HIGH);
      Serial.println(F("! Failed Creating Training Log File."));
    }
  }

  Serial.println();

  BT.begin(38400);
  pinMode(BT_Status, INPUT);

  pinMode(SF_Trig, OUTPUT);
  pinMode(SF_Echo, INPUT);
  pinMode(SL_Trig, OUTPUT);
  pinMode(SL_Echo, INPUT);
  pinMode(SR_Trig, OUTPUT);
  pinMode(SR_Echo, INPUT);

  pinMode(RL_FWRD,  OUTPUT);
  pinMode(RL_BWRD,  OUTPUT);
  pinMode(RL_LEFT,  OUTPUT);
  pinMode(RL_RIGHT, OUTPUT);

  digitalWrite(RL_FWRD,  HIGH);
  digitalWrite(RL_BWRD,  HIGH);
  digitalWrite(RL_LEFT,  HIGH);
  digitalWrite(RL_RIGHT, HIGH);

  moveVehicle(REC_MOVE_LEFT);
  delay(100);
  moveVehicle(REC_MOVE_RIGHT);
  delay(100);
  moveVehicle(REC_MOVE_LEFT);
  delay(100);
  moveVehicle(REC_MOVE_RIGHT);
  delay(100);
  moveVehicle(REC_STOP_ALL);

  Serial.println();
  Serial.println(F(">> Smart Car is Ready!"));
  Serial.println();

  autoDrive_Calc(getSensor_FRONT(), getSensor_LEFT(), getSensor_RIGHT());
  for (uint8_t i = 0; i < 3; i++)
  {
    Serial.print(outputActivation[0][i]);
    Serial.print(F(" "));
  }
  Serial.println();
}

void loop()
{
  if (digitalRead(BT_Status))
  {
    if (BT_STATE == LOW)
    {
      Serial.println(F("> BT Connected!"));
      BT_STATE = HIGH;
    }
  }
  else
  {
    if (BT_STATE == HIGH)
    {
      Serial.println(F("> BT Dis-Connected!"));
      LRS_BTR = F("Go Straight (Default).");
      TRAINING = false;
      AUTO_DRIVE = false;
      moveVehicle(REC_STOP_ALL);
      dataLog.close();
      BT_STATE = LOW;
    }
    return;
  }
  
  if (Serial.available())
  {
    uint8_t data = Serial.read();
    if (data == '0' && LOGData == true)
    {
      LOGData = false;
    }
    else if (data == '1' && LOGData == false)
    {
      LOGData = true;
    }
  }
  while(BT.available())
  {
    char RBT = BT.read();
    BTR += RBT;
    if (BTR == REC_MOVE_FORWARD || BTR == REC_STOP_FORWARD || BTR == REC_MOVE_BACKWARD || BTR == REC_STOP_BACKWARD || BTR == REC_MOVE_LEFT || BTR == REC_STOP_LEFT || BTR == REC_MOVE_RIGHT || BTR == REC_STOP_RIGHT || BTR == REC_STOP_ALL || BTR == REC_START_LOGGING || BTR == REC_STOP_LOGGING || BTR == REC_START_TRAINING || BTR == REC_STOP_TRAINING || BTR == REC_START_ADRIVE || BTR == REC_STOP_ADRIVE)
    {
      break;
    }
    delay(1);
  }

  if (BTR.length())
  {
    Serial.println();
    Serial.println(BTR);
    handleBluetooth(BTR);
    moveVehicle(BTR);
    BTR = "";
  }
  
  //  F("Turn \"LEFT\".") F("Turn \"RIGHT\".") F("Go Straight (Default).")

  if (TRAINING)
  {
//    if (getSensor_LEFT() > 5 && getSensor_LEFT() <= 255)
//    {
//      if (LRS_BTR == F("Turn \"LEFT\"."))
//      {
//        Temp_autoLeftCM = getSensor_LEFT();
//      }
//    }
//
//    if (getSensor_RIGHT() > 5 && getSensor_RIGHT() <= 255)
//    {
//      if (LRS_BTR == F("Turn \"RIGHT\"."))
//      {
//        Temp_autoLeftCM = getSensor_RIGHT();
//      }
//    }
     
    AUTO_DRIVE = false;
    TRAINING_LOG();
  }
  else if (AUTO_DRIVE)
  {
    TRAINING = false;
//    autoDrive(getSensor_LEFT(), getSensor_FRONT(), getSensor_RIGHT());

    autoDrive_Calc(getSensor_FRONT(), getSensor_LEFT(), getSensor_RIGHT());
    
  }

  currentTime = millis();
  if (!LOGData || TRAINING) return;
  
  SD_LOG();
    
  if (currentTime - previousLog >= 1000)
  { 
    Serial.print(F("#Front Sensor Data : ")); Serial.print(getSensor_FRONT()); Serial.println(F(" cm"));
    Serial.print(F("#Left Sensor Data  : ")); Serial.print(getSensor_LEFT()); Serial.println(F(" cm"));
    Serial.print(F("#Right Sensor Data : ")); Serial.print(getSensor_RIGHT()); Serial.println(F(" cm"));
    Serial.print(F("#User Command Received : ")); Serial.println(LRS_BTR);
    Serial.println();

    previousLog = currentTime;
  }
}
