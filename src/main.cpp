#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <HX711.h>
#include <NewPing.h>
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SDA0_Pin 40   // select ESP32  I2C pins
#define SCL0_Pin 41
const int LOADCELL_DOUT_PIN = 39;
const int LOADCELL_SCK_PIN = 38;
// Ultrasonic Sensor Pins
#define TRIG_PIN 5
#define ECHO_PIN 6

// IR Sensor Pins
#define IR_SENSOR_1 15
#define IR_SENSOR_2 7

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
AsyncWebServer server(80);

const char *ssid = "haquann";
const char *password = "quanhoangdz";
const int TURN45 = 500;
unsigned long previousMillis = 0;
const long interval = 3000; // Blinking interval in milliseconds
bool eyesClosed = false;
bool mouthSmiling = false;
int reading;
int ENA_pin = 13;
int IN1 = 12;
int IN2 = 11;
int ENB_pin = 8;
int IN3 = 46;
int IN4 = 3;
HX711 scale;
HardwareSerial gpsSerial(2);
Adafruit_GPS GPS(&gpsSerial);
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;
}


void displayInfo(int weight);
void setupGPS() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 17, 18);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
}

void clearGPS() {
  while (!GPS.newNMEAreceived()) {
    char c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());

  while (!GPS.newNMEAreceived()) {
    char c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
}

String getGPSData() {
  String gpsData = "";

  clearGPS();

  while (!GPS.newNMEAreceived()) {
    char c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());

  gpsData += "Time: " + String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "." + String(GPS.milliseconds) + "<br>";
  gpsData += "Date: " + String(GPS.day) + "/" + String(GPS.month) + "/20" + String(GPS.year) + "<br>";
  gpsData += "Fix: " + String(GPS.fix) + " quality: " + String(GPS.fixquality) + "<br>";
  gpsData += "Satellites: " + String(GPS.satellites) + "<br>";

  if (GPS.fix) {
    gpsData += "Location: " + String(GPS.latitude, 4) + GPS.lat + ", " + String(GPS.longitude, 4) + GPS.lon + "<br>";
    gpsData += "Google Maps location: " + String(GPS.latitudeDegrees, 4) + ", " + String(GPS.longitudeDegrees, 4) + "<br>";
    gpsData += "Speed (knots): " + String(GPS.speed) + "<br>";
    gpsData += "Heading: " + String(GPS.angle) + "<br>";
    gpsData += "Altitude: " + String(GPS.altitude) + "<br>";
  }

  return gpsData;
}
void rotateLeft(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA_pin, 255);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB_pin, 150);
}
void rotateRight(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA_pin, 150);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB_pin, 255);
}
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA_pin, 255);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB_pin, 255);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA_pin, 255);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB_pin, 255);
}



void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA_pin, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB_pin, 0);
}
void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA_pin, 200);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB_pin, 0);
  delay(TURN45);
  stopMotors();
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA_pin, 0);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB_pin, 200);
  delay(TURN45);
  stopMotors();
}

void handleMotorControl(AsyncWebServerRequest *request) {
  String motorAction = request->arg("action");

  if (motorAction == "forward") {
    moveForward();
  } else if (motorAction == "backward") {
    moveBackward();
  } else if (motorAction == "left") {
    turnLeft();
  } else if (motorAction == "right") {
    turnRight();
  } else if (motorAction == "stop") {
    stopMotors();
  }

  request->send(200, "text/plain", "Motor control executed: " + motorAction);
}
String currentMode = "manual";

void handleModeChange(AsyncWebServerRequest *request) {
  String mode = request->arg("mode");
  if (mode == "manual" || mode == "autonomous") {
    currentMode = mode;
    request->send(200, "text/plain", "Mode set to: " + mode);
  } else {
    request->send(400, "text/plain", "Invalid mode");
  }
}
void handleGPSData(AsyncWebServerRequest *request) {
  String gpsData = getGPSData();
  request->send(200, "text/html", gpsData);
}
void handleWeightData(AsyncWebServerRequest *request) {
  request->send(200, "text/plain", String(reading));
}
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Smart Suitcase Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      background-color: #f4f4f4;
      margin: 0;
      padding: 0;
    }

    h2 {
      color: #333;
    }

    .container {
      display: flex;
      flex-direction: column;
      align-items: center;
      margin-top: 20px;
    }

    .motor-container, .gps-container {
      background-color: #fff;
      border: 2px solid #ddd;
      border-radius: 10px;
      padding: 20px;
      margin: 10px;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
    }

    .button-container {
      display: flex;
      justify-content: space-around;
      margin-top: 20px;
    }

    .control-button, .stop-button {
      padding: 15px 25px;
      font-size: 18px;
      background-color: #4CAF50;
      color: #fff;
      border: none;
      border-radius: 50%;
      cursor: pointer;
      transition: background-color 0.3s ease;
    }

    .stop-button {
      background-color: #e74c3c;
    }

    .control-button:hover, .stop-button:hover {
      background-color: #45a049;
    }

    #gpsData {
      margin-top: 20px;
    }

    #weightBox {
      background-color: #f9f9f9;
      border: 2px solid #ddd;
      border-radius: 10px;
      padding: 20px;
      margin-top: 20px;
    }
  </style>
</head>
<body>
  <h2>Smart Suitcase Control</h2>

  <div class="container">
  <div>
    <button onclick="setMode('manual')">Manual Mode</button>
    <button onclick="setMode('autonomous')">Autonomous Mode</button>
  </div>
    <div class="motor-container">
      <h3>Motor Control</h3>
      <div class="button-container">
        <button class="control-button" onclick="sendMotorCommand('forward')">Foward</button>
      </div>
      <div class="button-container">
        <button class="control-button" onclick="sendMotorCommand('left')">Left</button>
        <button class="stop-button" onclick="sendMotorCommand('stop')">Stop</button>
        <button class="control-button" onclick="sendMotorCommand('right')">Right</button>
      </div>
      <div class="button-container">
        <button class="control-button" onclick="sendMotorCommand('backward')">Backward</button>
      </div>
    </div>

    <div class="gps-container">
      <h3>GPS Data</h3>
      <div id="gpsData"></div>
    </div>

    <div id="weightBox">
      <h3>Weight</h3>
      <div id="weightValue"></div>
    </div>
  </div>

  <script>
    function sendMotorCommand(action) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/motor?action=" + action, true);
      xhr.send();
    }

    function updateGPSData() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (xhr.readyState == 4 && xhr.status == 200) {
          document.getElementById("gpsData").innerHTML = xhr.responseText;
        }
      };
      xhr.open("GET", "/gps", true);
      xhr.send();
    }

    function updateWeight() {
      var xhr = new XMLHttpRequest();
      xhr.onreadystatechange = function() {
        if (xhr.readyState == 4 && xhr.status == 200) {
          document.getElementById("weightValue").innerHTML = xhr.responseText + " g";
        }
      };
      xhr.open("GET", "/weight", true);
      xhr.send();
    }
     function setMode(mode) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/mode?mode=" + mode, true);
    xhr.send();
  }

    setInterval(updateGPSData, 1000);
    setInterval(updateWeight, 1000);
  </script>
</body>
</html>
)rawliteral";



void setup() {
  Wire.begin(SDA0_Pin, SCL0_Pin);
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(102.79);
  scale.tare();  
  delay(1000);
// Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);  // Ensure trigger pin is low
  pinMode(ECHO_PIN, INPUT);

  // IR Sensors
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  pinMode(ENA_pin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB_pin, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(ENA_pin, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  analogWrite(ENB_pin, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  setupGPS();
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }

  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/motor", HTTP_GET, handleMotorControl);
  server.on("/gps", HTTP_GET, handleGPSData);
  server.on("/weight", HTTP_GET, handleWeightData);
  server.on("/mode", HTTP_GET, handleModeChange);
  server.begin();
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
    Serial.println(F("SSD1306 allocation OK"));

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  display.clearDisplay();
}

void loop() {
  Serial.print("one reading:\t");
  reading = scale.get_units();
  Serial.print(reading);
  if(reading < 0){
    reading = 0.00;
  }
  Serial.print("g");
  displayInfo(reading);
  scale.power_down();         
  scale.power_up();
  long distance = measureDistance();
  int rightIRSensorValue = digitalRead(IR_SENSOR_1);
  int leftIRSensorValue = digitalRead(IR_SENSOR_2);
  Serial.print("IR Sensor 1: ");
  Serial.println(rightIRSensorValue);
  Serial.print("IR Sensor 2: ");
  Serial.println(leftIRSensorValue);
  Serial.print("Distance: ");
  Serial.println(distance);

  delay(1000); // Delay a second before next reading
  if (currentMode == "autonomous"){
    if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
     rotateRight();
  }
  //If left sensor detects hand, then turn left. We increase right motor speed and decrease the left motor speed to turn towards left
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateLeft();
  }
  //If distance is between min and max then go straight
  else if ((distance >= 0 && distance <= 30)&& rightIRSensorValue == HIGH && leftIRSensorValue == HIGH)
  {
    moveForward();
  }
  //stop the motors
  else 
  {
    stopMotors();
  }
  }
  else {
    rightIRSensorValue = 1;
    leftIRSensorValue = 1;
    distance = 31;
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });
  }
}
void displayInfo(int weight){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Anh Quan Hoang");
  display.println("4072A Seneca");
  display.println("800-400-5678");
  display.println("Weight:");
  display.display();
  display.println(weight);
  display.display();  
}
