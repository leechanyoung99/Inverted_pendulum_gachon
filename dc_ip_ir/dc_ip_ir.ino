const int motor_pwm = 8;
const int motor_dir1 = 9;
const int motor_dir2 = 10;

// bool debugging = false;
bool debugging = true;

static unsigned long timer = 0;
unsigned long interval = 500;

double KP = 99.8;
double KI = 0.0;
double KD = 2.2;
double KP_none;
double KI_none;
double KD_none;
double error_integral = 0.0;
double error_differential = 0.0;
double error_prev = 0.0;
double output = 0.0;
double dt, t, t_prev;

const int encoderPinA = 2; // 엔코더 핀 A white
const int encoderPinB = 3; // 엔코더 핀 B black
long encoderPos = 0; // 엔코더 초기화
long encoderRot = 0; //
int encoderDeg = 0; // 엔코더 각도

double pwmValue;

int LastIR = 0;
double Dz_factor = 0;

void doEncoderA() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) // 같으면
    encoderPos++; // 정회전
  else // 다르면
    encoderPos--; // 역회전
}

void doEncoderB() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) // 같으면
    encoderPos--; // 역회전
  else // 다르면
    encoderPos++; // 정회전
}

int IR_distance() {
  const float smoothingFactor = 0.7;
  int volt = map(analogRead(A0), 0, 1023, 0, 5000); // 아날로그 입력값을 전압으로 변환
  static float filteredVolt = 0;
  filteredVolt = (smoothingFactor * volt) + ((1 - smoothingFactor) * filteredVolt);
  int distance = (27.61 / (filteredVolt - 0.1696)) * 1000; // 거리값 계산
  
  if (distance > 40 || distance < 7) {
    return 0; // 부정확한 영역
  } else {
    return distance;
  }
}

int Deadzone_Detector(){
  int IR = IR_distance(); // 거리 측정
  // Serial.println(IR); // 거리값 출력

  if (IR != 0 && IR <= 9){  // 현재 거리 10cm 이하 후진
    //Serial.println("backward");
    return -1;
  }
  else if (IR > 31 && IR > LastIR) { // 현재 거리 20cm 이상이며 거리가 멀어지는 중이면 전진
    //Serial.println("forward");
    return 1;
  }
  else {
    //Serial.println("neutral"); // 아니면 가만히 있기
    return 0;
  }
  LastIR=IR; // 다음을 위해 기록
  //return IR-20;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hold");
  delay(3000);
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
  delay(1000);
  Serial.println("Initialized");
  delay(1000);
  Serial.println("System on");
  pinMode(motor_pwm, OUTPUT);
  pinMode(motor_dir1, OUTPUT);
  pinMode(motor_dir2, OUTPUT);
}

void loop() {
  encoderDeg = 18000 - (encoderPos * 36000 / 14400) % 36000;
  Dz_factor = Deadzone_Detector(); //데드존 접근 거리에 따라 -1, 0 ,1
  double angle = encoderDeg / 100.0 + Dz_factor*1.5;
  PID(angle);
  if (abs(angle) > 90.0) {
    digitalWrite(motor_dir1, LOW);
    digitalWrite(motor_dir2, LOW);
  } else if (abs(angle) > 0.01) {
    Motor(output);
  } else {
    digitalWrite(motor_dir1, LOW);
    digitalWrite(motor_dir2, LOW);
  }
  
  delay(10);
  if (millis() - timer >= interval) {
    timer = millis();
    KP = analogRead(A1) / 1024.0 * 100.0;
    KI = analogRead(A2)/1024.0*0.1;
    KD = analogRead(A3) / 1024.0 * 10.0;
    
    if (debugging) {
    Serial.print("dT :");
    Serial.print(dt); Serial.print("\t");
    Serial.print("Output :");
    Serial.print(output); Serial.print("\t");
    Serial.print("KP :");
    Serial.print(KP); Serial.print("\t");
    Serial.print("KI :");
    Serial.print(KI); Serial.print("\t");
    Serial.print("KD :");
    Serial.print(KD); Serial.print("\t");
    Serial.print("PWM :");
    Serial.print(pwmValue); Serial.println("\t");
    }
  }
}

void Motor(double vel) {
  // double pwmValue = map(abs(vel),0,500,120,255);

  pwmValue = abs(vel)+140;
  if (pwmValue >255){
    pwmValue=255;
  }
  if (vel < 0.0) {
    digitalWrite(motor_dir1, HIGH);
    digitalWrite(motor_dir2, LOW);
  } else if (vel > 0.0) {
    digitalWrite(motor_dir1, LOW);
    digitalWrite(motor_dir2, HIGH);
  }
  analogWrite(motor_pwm, pwmValue);
}

void PID(double angle) {
  // if (angle<-1){
  //   angle=angle+1;
  // }
  // else if (angle>1){
  //   angle=angle-1;
  // }
  // dt = (millis() - t_prev) / 1000.0;
  dt = 0.01;
  error_integral += angle * dt;
  error_differential = (angle - error_prev) / dt;
  output = KP * angle + KI * error_integral + KD * error_differential;
  error_prev = angle;
  t_prev = millis();
  // if (output >= 200) {
  //   output = 200;
  // } else if (output <= -200) {
  //   output = -200;
  // }
}