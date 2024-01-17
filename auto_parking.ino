#include <Wire.h>
#include <ESP32Servo.h>

#define In1 13
#define In2 12
#define In3 14
#define In4 27

#define EnableA 26
#define EnableB 25

#define echoPin1 32
#define trigPin1 33

#define echoPin2 17
#define trigPin2 5

#define servopin 18

#define ledpin 21

#define MAX_MOTOR_SPEED 255

int sum1 = 0;
int sum2 = 0;

const int frontAngle = 80;
const int leftAngle = 180;

int currentServoPosition = 80;

bool phase1 = false;
bool phase2 = false;
bool phase3 = false;
bool phase4 = false;

int speed1 = 110;
int speed2 = 110;

Servo servo;

long duration1, distance1;
long duration2, distance2;
long duration3, distance3;

bool reachedBarrier = false;

bool rotatted = false;

bool slot = false;

bool adjusted = false;

bool end = false;


void setup() {
  Serial.begin(115200);

  // the motors
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnableA, OUTPUT);
  pinMode(EnableB, OUTPUT);
  analogWrite(EnableA, speed1);
  analogWrite(EnableB, speed2);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  servo.attach(servopin);
  servo.write(frontAngle);
  delay(150);

  pinMode(ledpin,OUTPUT);

}

void scanNetworks() {
  int numberOfNetworks = WiFi.scanNetworks();

  if (numberOfNetworks == 0) {
    Serial.println("No networks found");
  } else {
    Serial.print("Number of networks found: ");
    Serial.println(numberOfNetworks);

    for (int i = 0; i < numberOfNetworks; ++i) {
      // Print SSID and RSSI for each network
      Serial.print("Network: ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" | RSSI: ");
      Serial.println(WiFi.RSSI(i));
    }
  }
}

void measureDistanceLeft() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1 / 58.2;
  Serial.print("Left distance: ");
  Serial.println(distance1);

}

void measureDistancFront() {
  //Move the servo back to the front if needed
  if (currentServoPosition != frontAngle) {
    servo.write(frontAngle);
    delay(400); // Allow the servo to reach the position
    currentServoPosition = frontAngle;
  }

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 / 58.2;
  Serial.print("the front distance: ");
  Serial.println(distance2);

}
void measureDistanceLeftServo() {
  // Move the servo back to the front if needed
  if (currentServoPosition != leftAngle) {
    servo.write(leftAngle);
    delay(400); // Allow the servo to reach the position
    currentServoPosition = leftAngle;
  }

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  duration3 = pulseIn(echoPin2, HIGH);
  distance3 = duration3 / 58.2;
  Serial.print("left servo distance: ");
  Serial.println(distance3);  
}

void turnRightReverse() {
  // Serial.println("turn right reverse...");
  // startTime = millis(); 
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  analogWrite(EnableA, speed1-50  );
  analogWrite(EnableB, 0);
  // printElapsedTime();
}

void moveReverse() {
  // Serial.println("Moving reverse...");
  // startTime = millis();  // Record the start time
  analogWrite(EnableA, speed1);
  analogWrite(EnableB, speed2);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  
  // printElapsedTime();
}

//moveReverse
void moveForward() {
  // Serial.println("Moving forward...");
  //startTime = millis();
  analogWrite(EnableA, speed1);
  analogWrite(EnableB, speed2);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  //printElapsedTime();
}

void turnRightForward() {
  // Serial.println("turn right forward...");
  // startTime = millis();
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  
  analogWrite(EnableA, speed1 - 50);
  analogWrite(EnableB, 0);

  // printElapsedTime();
}

void rotateRight() {
  // Serial.println("Rotating right...");
  // startTime = millis();
  analogWrite(EnableA, speed1);
  analogWrite(EnableB, speed2);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  // printElapsedTime();
}

void rotateLeft() {
  // Serial.println("Rotating left...");
  // startTime = millis();
  analogWrite(EnableA, speed1 );
  analogWrite(EnableB, speed2);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  // printElapsedTime();
}

void stopMotors() {
  //Serial.println("Stopping motors...");
  // printElapsedTime();  // Print elapsed time before stopping
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}

// void printElapsedTime() {
//   unsigned long elapsedTime = millis() - startTime;
//   Serial.print("Elapsed time: ");
//   Serial.print(elapsedTime);
//   Serial.println(" ms");
// }

void ForwadrdTillReachingBarrier(int lim){
  for(int i = 0; i < 3 ; i++){
    measureDistancFront();
    sum1 += distance2;
    delay(10);
  }
  sum1 /= 3;
  if(sum1 < lim){
    reachedBarrier = true;
    return;
  }
  else{
    moveForward();
  }
  sum1 = 0;
}
void rotateParallelToBarrier(){
  measureDistanceLeft();
  //sum1 += distance1;
  delay(10);
  measureDistanceLeftServo();
  //sum2 += distance3;
  delay(10);
  // for(int i = 0; i < 3 ; i++){
  //   measureDistanceLeft();
  //   sum1 += distance1;
  //   delay(10);
  // }
  // sum1 /= 3;
  // for(int i = 0; i < 3 ; i++){
    
  //   measureDistanceLeftServo();
  //   sum2 += distance3;
  //   delay(10);
  // }
  // sum2 /= 3;
  if(abs(distance1 - distance2 ) <= 2){
    rotatted = true;
  }
  else{
    rotateRight();
  }
  Serial.print("distance diff: ");
  Serial.println(abs(distance1 - distance3 ));
}

void findAvalibleSlot(){
  measureDistanceLeft();
  delay(10);
  measureDistanceLeftServo();
  delay(10);

  if(distance1 >= 20 && distance3 >= 20 ){
      slot = true;
  }
  else{
    moveForward();
  }

}
void adjustToSlot(){
  measureDistanceLeft();
  sum1 += distance1;
  delay(10);
  measureDistanceLeftServo();
  sum2 += distance3;
  delay(10);
  // for(int i = 0; i < 5 ; i++){
  //   measureDistanceLeft();
  //   sum1 += distance1;
  //   delay(10);
  // }
  // sum1 /= 5;
  // for(int i = 0; i < 5 ; i++){
    
  //   measureDistanceLeftServo();
  //   sum2 += distance3;
  //   delay(10);
  // }
  // sum2 /= 5;
  if(abs(distance1 - distance3 ) <= 2){
    adjusted = true;
  }
  else{
    rotateLeft();
  }
  Serial.print("distance diff: ");
  Serial.println(abs(distance1 - distance3 ));
}

void loop() {
  if(!reachedBarrier){
    if(!reachedBarrier){
      if(!phase4){
        ForwadrdTillReachingBarrier(17);
      }
      else{
        ForwadrdTillReachingBarrier(11);
      }
    }
    
    if(reachedBarrier){
      if(phase4){
        rotatted = false;
        phase2 = false;
      }
        phase1 = true;
        // Serial.println("reached barrier");
        // delay(2000);
        rotateRight();
        delay(1000);
        stopMotors();
        delay(250);
        sum1 = 0;
    }
  }
  if(!rotatted && phase1 ){
    if(!rotatted){
      rotateParallelToBarrier();
    }
    if(rotatted){
      if(phase4){
      digitalWrite(ledpin, HIGH);
      }
      else{
      digitalWrite(ledpin, LOW);
      }
      phase2 = true;

      stopMotors();
      delay(1500);
    }
  }

  if(!slot && phase2){
    if(!slot){
      findAvalibleSlot();
    }
    if(slot){
      moveForward();
      delay(50);
      rotateLeft();
      delay(1400);
      moveForward();
      delay(70);
      stopMotors();
      phase3 = true;
      // Serial.println("found slot");
      // delay(2000);
    }
  }
  if(!adjusted && phase3){
    if(!adjusted){
      adjustToSlot();
    }
    if(adjusted){
      phase4 = true;
      phase1 = false;
      reachedBarrier = false;
      stopMotors();
    }
  }

  // if(phase4){
  //   measureDistancFront();
  //   delay(10);
  //   measureDistanceLeft();
  //   delay(10);
  // }

  if(Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'w':
        moveForward();
        break;
      case 's':
        moveReverse();
        break;
      case 'a':
        rotateLeft();
        break;
      case 'd':  
        rotateRight();
        break;
      case ' ':
        stopMotors();
        break;
      default:
        // Handle other cases if needed
        break;
    }
  }

  // if(!park){

  //     moveForward();
  //     delay(2590);
  //     stopMotors();
  //     delay(1000);
  //     rotateLeft();
  //     delay(1235);
  //     stopMotors();
  //     delay(1000);
  //     moveReverse();
  //     delay(1008);
  //     stopMotors();
  //     delay(1000);
  //     rotateRight();
  //     delay(1425);
  //     stopMotors();
  //     delay(1000);
  //     // adjusting
  //     rotateLeft();
  //     delay(507 );
  //     stopMotors();
  //     delay(1000);
  //     moveReverse();
  //     delay(546);
  //     stopMotors();
  //     delay(1000);
  //     rotateRight();
  //     delay(654);
  //     stopMotors();
  //     delay(1000);
  //     moveForward();
  //     delay(468);
  //     stopMotors();
  //     delay(1000);

  //     stopMotors();
  //     digitalWrite(ledpin,HIGH);
  //     park = true;
  // }
  
}
