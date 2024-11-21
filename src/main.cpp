#include <Arduino.h>
#include <Servo.h>

#define B_BUTTON_PIN 17
#define RESET_BUTTON_PIN 15
#define SERVO_PIN 27

Servo myservo;
int yellow_goal = 5;
int yellow_pressed = 0;

void setup() {
  Serial.begin(9600);
  pinMode(B_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  myservo.attach(SERVO_PIN);

}


void loop() {
  int yellowbuttonState = digitalRead(B_BUTTON_PIN);
  int resetbuttonState = digitalRead(RESET_BUTTON_PIN);
  if (yellowbuttonState == LOW) {
    ++yellow_pressed;
    Serial.print("button is LOW\n");
    delay(1000);
  }
  
  if (yellow_pressed > yellow_goal)
    yellow_pressed = yellow_goal;
  
  if (resetbuttonState == LOW) {
    yellow_pressed = 0;
  }

  int servoNum = map(yellow_pressed, 0, yellow_goal, 0, 179);
  myservo.write(servoNum);
  Serial.print(servoNum);
  Serial.print('\n');
  
}