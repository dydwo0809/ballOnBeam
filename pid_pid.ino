#include <Servo.h> // [2972] 서보 헤더파일 포함


/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 // [1234] LED를 아두이노 GPIO 9번 핀에 연결
                  // [2345] ...하면 개선
#define PIN_SERVO 10 // [2951] servo moter를 아두이노 GPIO 10번 핀에 연결
#define PIN_IR A0      // [2961] 적외선센서를 아두이노 A0핀에 연결

// Framework setting
#define _DIST_TARGET 255 //[2967] 탁구공 위치까지의 거리를 255로 고정
         // [2952] 탁구공의 목표 위치를 25.5cm로 설정
#define _DIST_MIN 100 // [2972] 거리 센서가 인식 가능하게 설정한 최소 거리
#define _DIST_MAX 410 // [2972] 거리 센서가 인식 가능하게 설정한 최대 거리

// Distance sensor
#define _DIST_ALPHA 0.7   // [2959] ema 필터에 적용할 알파값

// Servo range
#define _DUTY_MIN 1115  // [2952] 서보의 최소 각도값
#define _DUTY_NEU 1381 // [2952] 서보의 중간 각도값
#define _DUTY_MAX 1640 // [1691] 서보의 최대 각도

// Servo speed control
//#define _SERVO_ANGLE 30  //[2967] 서보 각도 설정
#define _SERVO_SPEED 300  //[2959] 서보의 속도 설정

// Event periods
#define _INTERVAL_DIST 20  //[2959] 센서의 거리측정 인터벌값
#define _INTERVAL_SERVO 20 //[2967] 서보 INTERVAL값 설정
#define _INTERVAL_SERIAL 100  //[2959] 시리얼 모니터/플로터의 인터벌값 설정

// PID parameters
#define _KP 1.5 // [2961] 비례이득
#define _KD 88  // 미분 이득  
#define _KI 0.009 // 적분 이득              

#define a 88  //100
#define b 146 //150
#define c 187 //200
#define d 223 //245
#define e 235 //255
#define f 246 //265
#define g 277 //300
#define h 407 //400

#define num 9
float sensorValues[num];
float temp[num];
int avg = num / 2;

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  // [2972] 서보 정의

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;    // [2961] dist_raw : 적외선센서로 얻은 거리를 저장하는 변수
                             // [2961] dist_ema : 거리를 ema필터링을 한 값을 저장하는 변수
float alpha;                 // [2959] ema의 알파값을 저장할 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;// [2957] last_sampling_time_dist : 거리센서 측정 주기
                                                                                           // [2957] last_sampling_time_servo : 서보 위치 갱신 주기
                                                                                           // [2957] last_sampling_time_serial : 제어 상태 시리얼 출력 주기
bool event_dist, event_servo, event_serial;                                                // [2957] 각각의 주기에 도달했는지를 불리언 값으로 저장하는 변수

// Servo speed control
int duty_chg_per_interval;                                                                 // [2952] 한 주기당 변화할 서보 활동량을 정의
int duty_target, duty_curr;                                                                // [2961] 목표위치, 서보에 입력할 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;



void setup() {
  // initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);                                                               // [2952] LED를 GPIO 9번 포트에 연결
                                                                                          // [2957] LED를 출력 모드로 설정
  myservo.attach(PIN_SERVO);                                                              // [2952] 서보 모터를 GPIO 10번 포트에 연결

  // initialize global variables
  alpha = _DIST_ALPHA;   // [2959] ema의 알파값 초기화
  dist_ema = 99.9;          // [2959] dist_ema 초기화
  dist_target = _DIST_TARGET;
  dterm = 0;

  // move servo to neutral position
  duty_curr = _DUTY_NEU;

  // initialize serial port
  Serial.begin(57600); // [2952] 시리얼 포트를 57600의 속도로 연결

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);  // [2959] 한 주기마다 이동할 양
                                                                                                          // [2974] INTERVAL -> _INTERVAL_SERVO 로 수정

                                                   // [2974] 이벤트 변수 초기화
  last_sampling_time_dist = 0;                     // [2974] 마지막 거리 측정 시간 초기화
  last_sampling_time_servo = 0;                    // [2974] 마지막 서보 업데이트 시간 초기화
  last_sampling_time_serial = 0;                   // [2974] 마지막 출력 시간 초기화
  event_dist = event_servo = event_serial = false; // [2974] 각 이벤트 변수 false로 초기화

  for (int i = 0; i < num ; i ++){
      sensorValues[i] = 100;
     }
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
  unsigned long time_curr = millis();  // [2964] event 발생 조건 설정
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true; // [2957] 거리 측정 주기에 도달했다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true; // [2957] 서보모터 제어 주기에 도달했다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true; // [2957] 출력주기에 도달했다는 이벤트 발생
  }

////////////////////
// Event handlers //
////////////////////

  // get a distance reading from the distance sensor
  
  if(event_dist) { 
     event_dist = false;
     dist_raw = ir_distance_filtered();   // [2959] dist_raw에 필터링된 측정값 저장
     float val = ir_distance();
     if (dist_ema == 99.9){                  // [2959] 맨 처음
       dist_ema = dist_raw;               // [2959] 맨 처음 ema값 = 필터링된 측정값
     }                                    // [2963] dist_ema를 dist_raw로 초기화
     else{
       dist_ema = alpha * dist_raw + (1-alpha) * dist_ema;   // [2959] ema 구현
     }
     for (int i = 0 ; i < num - 1 ; i++){
       sensorValues[i] = sensorValues[i+1];
     }
     sensorValues[num - 1] = dist_ema;

     for (int i = 0; i < num ; i ++){
      temp[i] = sensorValues[i];
     }
     
     for (int i = 0; i < num - 1; i++){
       for (int j = 0; j < num - 1; j++){
         if (temp[j] > temp[j+1]){
           float buff = temp[j];
           temp[j] = temp[j+1];
           temp[j+1] = buff;
         }
       }
     }
     dist_ema = (temp[avg - 1] + temp[avg] + temp[avg + 1]) / 3.0;
     
    
  // PID control logic
    error_curr = dist_target - dist_ema;
    
    pterm = _KP * error_curr;             // [2959] PID의 P

    if (error_curr - error_prev > -0.7 && error_curr - error_prev < 0.7){
      dterm = 0;
    }
    else{
      dterm = _KD * (error_curr - error_prev);
    }
    if (230 < dist_ema && dist_ema < 280){
      iterm += _KI * error_curr;
    }
    else{
      iterm = 0;
    }
    control = pterm + dterm + iterm;     // [2959] 중간에서 얼마나 움직일 것인가

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }
    if(duty_target > _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }

    //update error_prev
    error_prev = error_curr;

    if(duty_target > duty_curr) {                             // [2964] 현재 서보 각도 읽기
      duty_curr += duty_chg_per_interval;                     // [2961] duty_curr은 주기마다 duty_chg_per_interval만큼 증가
      if(duty_curr > duty_target) {duty_curr = duty_target;}  // [2956] duty_target 지나쳤을 경우, duty_target에 도달한 것으로 duty_curr값 재설정
    }
    else {
      duty_curr -= duty_chg_per_interval;                     // [2961] duty_curr은 주기마다 duty_chg_per_interval만큼 감소
      if (duty_curr < duty_target) {duty_curr = duty_target;} // [2956] duty_target 지나쳤을 경우, duty_target에 도달한 것으로 duty_curr값 재설정
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr);                     // [2964] 서보 움직임 조절

  }
  
  if(event_serial) {
    event_serial = false; // [2974] 출력 이벤트 실행 후, 다음 주기까지 이벤트 종료 
    Serial.print("IR:");
    Serial.print(dist_ema);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));   
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));    
    Serial.print(",DTT:"); 
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:"); 
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
}
float ir_distance(void){ // [2959] 센서가 측정한 거리를 리턴해주는 함수
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0; 
  return val;                             
}                                              

float ir_distance_filtered(void){                     // [2959] 센서가 측정한 거리를 필터링한 값을 리턴
  float val = ir_distance();
  if (val < b){
    return 100 + 50.0 * (val - a) / (b - a);
  }
  else if (val < c){
    return 150 + 50 * (val - b) / (c - b);
  }
  else if (val < d){
    return 200 + 45 * (val - c) / (d - c);           
  }
  else if (val < e){
    return 245 + 10 * (val - d) / (e - d);           
  }
  else if (val < f){
    return 255 + 10 * (val - e) / (f - e);           
  }
  else if (val < g){
    return 265 + 45 * (val - f) / (g - f);           
  }
  else {
    return 300 + 100 * (val - g) / (h - g);           
  }
                                                          
}
