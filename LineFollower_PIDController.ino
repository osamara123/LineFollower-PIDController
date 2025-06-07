#include <SoftwareSerial.h>
const int LM1 = 6, LM2 = 7;
const int RM1 = 4, RM2 = 5;
const int ENL = 11, ENR = 10;

const int SL = A0;
const int SC = A1;
const int SR = A2;

SoftwareSerial BT(12, 13);

int NORMAL = 150;
int EXTRA = 190;
float P = 0.3;    
float I = 0.02;   
float D = 1;    
float NOTHING = 0.05;
int SLEW = 5;


const unsigned lost = 650;
const unsigned deb = 70;
const int t_delay = 400;
const byte READS = 5;

const float filter = 0.70;
const float reading = 0.30;
const float history_error = 0.2;

unsigned long time_last_seen = 0;
int last_direction = 0;
bool at = false;
float fErr = 0.0;
int pwmL = 0, pwmR = 0;
float positionHistory = 0.0;

float error = 0;
float previous_error = 0;
float integral = 0;
float derivative = 0;
float PID_value = 0;

void setup() {
  pinMode(LM1, OUTPUT); pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT); pinMode(RM2, OUTPUT);
  pinMode(ENL, OUTPUT); pinMode(ENR, OUTPUT);

  pinMode(SL, INPUT); pinMode(SC, INPUT); pinMode(SR, INPUT);

  Serial.begin(9600);  // ← ADD THIS LINE
  BT.begin(9600);
  delay(2000);
  
  Serial.println("Serial monitor ready!");  // Confirmation message
  BT.println("AT+BAUD?");  // Check current baud rate

  delay(1500);
  delay(1500);
}

bool general(int pin) {
  byte sum = 0;
  for (byte i = 0; i < READS; ++i) {
    sum += !digitalRead(pin);
    delayMicroseconds(150);
  }
  return sum >= (READS + 1) / 2;
}

void loop() {



   if (BT.available()) {
  String cmd = BT.readStringUntil('\n');
  cmd.trim(); // Remove whitespace
  
  Serial.print("Command received: '");
  Serial.print(cmd);
  Serial.println("'");
  
  if (cmd.length() > 1) {
    char type = cmd.charAt(0);
    Serial.print("First character: '");
    Serial.print(type);
    Serial.println("'");
    
    // Print ASCII value to check for hidden characters
    Serial.print("ASCII value: ");
    Serial.println((int)type);
    
    int value = cmd.substring(1).toInt();
    Serial.print("Value part: ");
    Serial.println(value);
    
    if (value >= 0 && value <= 255) {
      Serial.println("Value in range!");
      
      if (type == 'B') {
        Serial.println("TYPE MATCHED 'B'!");
        NORMAL = value;
        Serial.print("NORMAL set to "); 
        Serial.println(NORMAL);
      } 
      else {
        Serial.print("Type did not match 'B', got: ");
        Serial.println(type);
      }
    }
    else {
      Serial.println("Value out of range 0-255");
    }
  }
}


  bool L = general(SL);
  bool C = general(SC);
  bool R = general(SR);

  static unsigned long tT = 0;
  static bool allSensorsActive = false;

  if (L && C && R) {
    if (!allSensorsActive) {
      tT = millis();
      allSensorsActive = true;
    } else if (millis() - tT > deb) {
      at = true;
    }
  } else {
    allSensorsActive = false;
  }

  if (C) {
    int rawErr = (int)R - (int)L;
    fErr = filter * fErr + reading * rawErr;
    positionHistory = positionHistory * history_error + rawErr * (1 - history_error);
    
    error = fErr;
    
    integral += error;
    derivative = error - previous_error;
    PID_value = P * error + I * integral + D * derivative;
    previous_error = error;
    
    float corr = (fabs(error) < NOTHING) ? 0 : PID_value * 255.0;
    
    setDrive(NORMAL - corr, NORMAL + corr);

    if (rawErr != 0) last_direction = rawErr;
    time_last_seen = millis();
    at = false;
  }
  else if (L ^ R) {
    float sideValue = L ? -1.0 : 1.0;
    positionHistory = positionHistory * history_error + sideValue * (1 - history_error);
    
    error = sideValue * 1.5; 
    integral += error;
    derivative = error - previous_error;
    previous_error = error;

    if (L) setDrive(NORMAL * 0.2, NORMAL * 1.0);
    else setDrive(NORMAL * 1.0, NORMAL * 0.2);

    last_direction = L ? -1 : 1;
    time_last_seen = millis();
  }
  else if (at) {
    integral = 0;
    
    setDrive(NORMAL, NORMAL);
    delay(100);

    digitalWrite(LM1, LOW); digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW); digitalWrite(RM2, LOW);
    analogWrite(ENL, 0);
    analogWrite(ENR, 0);
    delay(50);

    bool leftSeen = general(SL);
    bool centerSeen = general(SC);
    bool rightSeen = general(SR);

    if (centerSeen) {
      setDrive(NORMAL, NORMAL);
      delay(t_delay);
    }
  

    else if (leftSeen && !rightSeen) {
      Left();
    }
    else if (rightSeen && !leftSeen) {
      Right();
    }
    else {
      spiral();
    }

    at = false;
  }
  else {
    integral *= 0.5; 
      spiral();
  }

  delayMicroseconds(500);
}

void setDrive(float tgtL, float tgtR) {
  static float smoothTargetL = 0, smoothTargetR = 0;
  smoothTargetL = 0.85 * smoothTargetL + 0.15 * tgtL;
  smoothTargetR = 0.85 * smoothTargetR + 0.15 * tgtR;

  smoothTargetL = constrain(smoothTargetL, 0, 255);
  smoothTargetR = constrain(smoothTargetR, 0, 255);

  pwmL += constrain(smoothTargetL - pwmL, -SLEW, SLEW);
  pwmR += constrain(smoothTargetR - pwmR, -SLEW, SLEW);

  digitalWrite(LM1, LOW); digitalWrite(LM2, HIGH);
  digitalWrite(RM1, HIGH); digitalWrite(RM2, LOW);
  analogWrite(ENL, pwmL); analogWrite(ENR, pwmR);
}

void spin(int dir) {
  if (abs(positionHistory) > 0.3) {
    dir = (positionHistory > 0) ? 1 : -1;
  }

  static int currentSpinPower = 0;
  currentSpinPower = min(EXTRA, currentSpinPower + SLEW / 2);

  if (dir < 0) {
    digitalWrite(LM1, HIGH); digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH); digitalWrite(RM2, LOW);
  } else {
    digitalWrite(LM1, LOW); digitalWrite(LM2, HIGH);
    digitalWrite(RM1, LOW); digitalWrite(RM2, HIGH);
  }

  analogWrite(ENL, currentSpinPower);
  analogWrite(ENR, currentSpinPower);
}

void spiral() {
  static byte currentPower = 90;
  static float spiralSpeed = 90;
  static unsigned long lastDirectionChange = 0;
  static int spiralDir = 1;

  spiralDir = (positionHistory > 0) ? 1 : -1;

  unsigned long now = millis();
  if (now - lastDirectionChange > 1200) {
    spiralDir = -spiralDir;
    lastDirectionChange = now;
    spiralSpeed = max(90.0, spiralSpeed * 0.8);
  }

  spiralSpeed = min(180.0, spiralSpeed + 0.15);
  currentPower += constrain(spiralSpeed - currentPower, -SLEW / 2, SLEW / 2);

  int innerPower = currentPower * 0.85;
  int outerPower = currentPower;

  if (spiralDir < 0) {
    digitalWrite(LM1, HIGH); digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH); digitalWrite(RM2, LOW);
    analogWrite(ENL, innerPower);
    analogWrite(ENR, outerPower);
  } else {
    digitalWrite(LM1, LOW); digitalWrite(LM2, HIGH);
    digitalWrite(RM1, LOW); digitalWrite(RM2, HIGH);
    analogWrite(ENL, outerPower);
    analogWrite(ENR, innerPower);
  }
}

void Left() {
  digitalWrite(LM1, HIGH); digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW); digitalWrite(RM2, HIGH);
  analogWrite(ENL, EXTRA);
  analogWrite(ENR, EXTRA);
  delay(100);
}

void Right() {
  digitalWrite(LM1, LOW); digitalWrite(LM2, HIGH);
  digitalWrite(RM1, HIGH); digitalWrite(RM2, LOW);
  analogWrite(ENL, EXTRA);
  analogWrite(ENR, EXTRA);
  delay(100);
}