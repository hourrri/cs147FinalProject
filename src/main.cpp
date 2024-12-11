#include <Arduino.h>
#include <Servo.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <Wire.h>

#include "SparkFunLSM6DSO.h"
#include <cmath>

LSM6DSO myIMU;

float value;
float prevValue;

int pickups = 0;

#define B_BUTTON_PIN 17
#define RESET_BUTTON_PIN 15
#define SERVO_PIN 27
#define TIMER_BUTTON_PIN 13

// from here (testing heartbeat + temp)
#define TEMP_PIN 36 // ADC0
#define HEARTBEAT_PIN 39 // ADC3

int heart_signal;
int BPM;
int beat_threshold = 2000; // maybe 500 idk
// to here (testing heartbeat + temp)

Servo myservo;
int blue_goal = 5;
int blue_pressed = 0;
int reset_pressed = 0;
int hit_goal = 0;
double temperature = 0;

bool tracked;

const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n"\
"MIIEtjCCA56gAwIBAgIQCv1eRG9c89YADp5Gwibf9jANBgkqhkiG9w0BAQsFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
"MjAeFw0yMjA0MjgwMDAwMDBaFw0zMjA0MjcyMzU5NTlaMEcxCzAJBgNVBAYTAlVT\n" \
"MR4wHAYDVQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xGDAWBgNVBAMTD01TRlQg\n" \
"UlMyNTYgQ0EtMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMiJV34o\n" \
"eVNHI0mZGh1Rj9mdde3zSY7IhQNqAmRaTzOeRye8QsfhYFXSiMW25JddlcqaqGJ9\n" \
"GEMcJPWBIBIEdNVYl1bB5KQOl+3m68p59Pu7npC74lJRY8F+p8PLKZAJjSkDD9Ex\n" \
"mjHBlPcRrasgflPom3D0XB++nB1y+WLn+cB7DWLoj6qZSUDyWwnEDkkjfKee6ybx\n" \
"SAXq7oORPe9o2BKfgi7dTKlOd7eKhotw96yIgMx7yigE3Q3ARS8m+BOFZ/mx150g\n" \
"dKFfMcDNvSkCpxjVWnk//icrrmmEsn2xJbEuDCvtoSNvGIuCXxqhTM352HGfO2JK\n" \
"AF/Kjf5OrPn2QpECAwEAAaOCAYIwggF+MBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYD\n" \
"VR0OBBYEFAyBfpQ5X8d3on8XFnk46DWWjn+UMB8GA1UdIwQYMBaAFE4iVCAYlebj\n" \
"buYP+vq5Eu0GF485MA4GA1UdDwEB/wQEAwIBhjAdBgNVHSUEFjAUBggrBgEFBQcD\n" \
"AQYIKwYBBQUHAwIwdgYIKwYBBQUHAQEEajBoMCQGCCsGAQUFBzABhhhodHRwOi8v\n" \
"b2NzcC5kaWdpY2VydC5jb20wQAYIKwYBBQUHMAKGNGh0dHA6Ly9jYWNlcnRzLmRp\n" \
"Z2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RHMi5jcnQwQgYDVR0fBDswOTA3\n" \
"oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0R2xvYmFsUm9v\n" \
"dEcyLmNybDA9BgNVHSAENjA0MAsGCWCGSAGG/WwCATAHBgVngQwBATAIBgZngQwB\n" \
"AgEwCAYGZ4EMAQICMAgGBmeBDAECAzANBgkqhkiG9w0BAQsFAAOCAQEAdYWmf+AB\n" \
"klEQShTbhGPQmH1c9BfnEgUFMJsNpzo9dvRj1Uek+L9WfI3kBQn97oUtf25BQsfc\n" \
"kIIvTlE3WhA2Cg2yWLTVjH0Ny03dGsqoFYIypnuAwhOWUPHAu++vaUMcPUTUpQCb\n" \
"eC1h4YW4CCSTYN37D2Q555wxnni0elPj9O0pymWS8gZnsfoKjvoYi/qDPZw1/TSR\n" \
"penOgI6XjmlmPLBrk4LIw7P7PPg4uXUpCzzeybvARG/NIIkFv1eRYIbDF+bIkZbJ\n" \
"QFdB9BjjlA4ukAg2YkOyCiB8eXTBi2APaceh3+uBLIgLk8ysy52g2U3gP7Q26Jlg\n" \
"q/xKzj3O9hFh/g==\n" \
"-----END CERTIFICATE-----\n";

//format for SAS token : "SharedAccessSignature sr=147HubProject.azure-devices.net%2Fdevices%2F147esp32&sig=7QEm3hsSUR9PGfl4ZekPU%2BEb6j8KwWPnDar%2B5V36vwU%3D&se=1732913822";

const char* iothubName = "147HubProject";
const char* deviceName = "147esp32";
const char* sasToken = "SharedAccessSignature sr=147HubProject.azure-devices.net%2Fdevices%2F147esp32&sig=myHq5cdmUjTcLDjrmIzFhOUure5kCNtze5Hpa2TkXuM%3D&se=1733180612";

const char* ssid = "Houri (2)";
const char* password = "zxcvbnma";


void sendData(int blueButton, int resetButton, int hitGoal, int numOfPickups, int BPM, double temp) {
  HTTPClient http;

  String url = "https://" + String(iothubName) + 
               ".azure-devices.net/devices/" + String(deviceName) + 
               "/messages/events?api-version=2016-11-14";

  String payload ="{\"blueButton\": " + String(blueButton) + 
                   ", \"resetButton\": " + String(resetButton) + 
                   ", \"hitGoal\": " + String(hitGoal) + 
                   ", \"numOfPickups\": " + String(numOfPickups) + 
                   ", \"tempurature\": " + String(temp) + 
                   ", \"BPM\": " + String(BPM) + "}";

  http.begin(url, root_ca); 
  http.addHeader("Authorization", sasToken);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(payload);

  Serial.print("Sending data to: ");
  Serial.println(url);
  Serial.print("Payload: ");
  Serial.println(payload);

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Error in HTTP request: ");
    Serial.println(http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

void setup() {
  Serial.begin(9600);
  pinMode(B_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIMER_BUTTON_PIN, INPUT_PULLUP);

  myservo.attach(SERVO_PIN);

  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");

  // for accelerometer
  Wire.begin();
  delay(10);
  if( myIMU.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");

  // for heartbeat sensor
  tracked = 0;
  reset_pressed = 0;
}


void loop() {
  int bluebuttonState = digitalRead(B_BUTTON_PIN);
  int resetbuttonState = digitalRead(RESET_BUTTON_PIN);
  int timerbuttonState = digitalRead(TIMER_BUTTON_PIN);
  

  int BUDUMP = 0;

  if (bluebuttonState == LOW ) {
    ++blue_pressed;
    sendData(blue_pressed, reset_pressed, hit_goal, pickups, BPM, temperature);
  }
  
  if (blue_pressed >= blue_goal) { // changed from > to = so multiple presses after goal reached dont increment hit_goal DOUBLE CHECK HERE PLS
    if(!tracked){
      ++hit_goal;
      tracked = 1;
    }
    blue_pressed = blue_goal;
    sendData(blue_pressed, reset_pressed, hit_goal, pickups, BPM, temperature);
  }
  
  if (resetbuttonState == LOW) {
    ++reset_pressed;
    blue_pressed = 0;
    tracked = 0;
    sendData(blue_pressed, reset_pressed, hit_goal, pickups, BPM, temperature);
  }

  if (timerbuttonState == LOW) {
    Serial.println("------TIMER START------");
    unsigned long starttime = millis();
    unsigned long endtime = starttime;
    
    int min_temp = 0;
    int max_temp = 0;
    bool first_run = 1;
    int temp_reading = 0;

    while ((endtime - starttime) <= 15000) {
      heart_signal = analogRead(HEARTBEAT_PIN);
      Serial.println(heart_signal);
      if (heart_signal > beat_threshold)
        ++BUDUMP;
      endtime = millis();

      temp_reading = analogRead(TEMP_PIN);
      if (first_run) {
        max_temp = temp_reading;
        min_temp = temp_reading;
        first_run = 0;
      }

      if (temp_reading > max_temp)
        max_temp = temp_reading;
      if (temp_reading < min_temp)
        min_temp = temp_reading;

      delay(550);
    }

    BPM = BUDUMP * 4;
    Serial.println("BPM: ");
    Serial.println(BPM);

    temp_reading = analogRead(TEMP_PIN);
    temperature = map(temp_reading, min_temp, max_temp, 9700, 9900);
    temperature = temperature / 100;

    Serial.println("TEMP MAPPED: ");
    Serial.println(temperature);

    sendData(blue_pressed, reset_pressed, hit_goal, pickups, BPM, temperature);
  }


  prevValue = value;
  value = myIMU.readFloatAccelX();

  // Serial.println("\nvalue: ");
  // Serial.println(value);

  if (abs((prevValue - value)) > .2) {
    pickups++;
    sendData(blue_pressed, reset_pressed, hit_goal, pickups, BPM, temperature);
  }

  // Serial.println("Number of pickups: ");
  // Serial.println(pickups);

  int servoNum = map(blue_pressed, 0, blue_goal, 0, 179);
  myservo.write(servoNum);
  // Serial.print(servoNum);
  // Serial.print('\n');
  // sendData(blue_pressed, reset_pressed, hit_goal, pickups, BPM, temperature);
}
