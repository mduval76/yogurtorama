#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PID_v1.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <littlefs.h>
#include <WebSocketsServer.h>
#include <stack>

#define PIN_SENSOR A0
#define PIN_HEATER D1

#define PERIOD_2MIN 12000
#define PERIOD_5MIN 30000
#define PERIOD_SECOND 1000

#define MAX_TEMP 50

ESP8266WebServer httpd(80);
WebSocketsServer webSocket = WebSocketsServer(81);

bool isWithinTargetRange = false;
bool isConnected = false;

double set_point = 43.0;
double input;
double output;

double agg_kp=4, agg_ki=0.2, agg_kd=1;
double cons_kp=2, cons_ki=0.1, cons_kd=0.5;

double min_temp2=0, max_temp2=0, min_temp5=0, max_temp5=0;
std::deque<double> windowed2MinTempReadings;
std::deque<double> windowed5MinTempReadings;

unsigned long previous_time = 0;
unsigned long time_in_range_start = 0;
unsigned long time_in_range = 0;

PID myPID(&input, &output, &set_point, agg_kp, agg_ki, agg_kd, DIRECT);

double readTemperature(int pin) {
  int value = analogRead(pin);
  double voltage = value * (3.3 / 1023.0);
  double resistance = 10000.0 * voltage / (3.3 - voltage);
  double temp_kelvin = 1.0 / (1.0 / 298.15 + 1.0 / 3950.0 * log(resistance / 10000.0));
  double temp_celsius = temp_kelvin - 273.15;
  return temp_celsius;
}

String getContentType(String filename) {
  struct Mime {
    String ext, type;
  } 
  
  mime_types[] = {
    {".html", "text/html"},
    {".js", "application/javascript"},
    {".css", "text/css"},
    {".wasm", "application/wasm"}
  };

  for (unsigned int i = 0; i < sizeof(mime_types) / sizeof(Mime); ++i) {
    if (filename.endsWith(mime_types[i].ext))
      return mime_types[i].type;
  }

  return "application/octet-stream";
}

void handleMinMaxTimeRangedValues() {
  windowed2MinTempReadings.push_front(input);
  windowed5MinTempReadings.push_front(input);

  if (input < min_temp2) min_temp2 = input;
  if (input > max_temp2) max_temp2 = input;
  if (input < min_temp5) min_temp5 = input;
  if (input > max_temp5) max_temp5 = input;

  int removed2MinValue = windowed2MinTempReadings.back();
  if (removed2MinValue == min_temp2 || removed2MinValue == max_temp2 || min_temp2 == 0) {
    min_temp2 = *std::min_element(windowed2MinTempReadings.begin(), windowed2MinTempReadings.end());
    max_temp2 = *std::max_element(windowed2MinTempReadings.begin(), windowed2MinTempReadings.end());
  }

  int removed5MinValue = windowed2MinTempReadings.back();
  if (removed5MinValue == min_temp5 || removed5MinValue == max_temp5 || min_temp5 == 0) {
    min_temp5 = *std::min_element(windowed2MinTempReadings.begin(), windowed2MinTempReadings.end());
    max_temp5 = *std::max_element(windowed2MinTempReadings.begin(), windowed2MinTempReadings.end());
  }
  
  if (windowed2MinTempReadings.size() >= PERIOD_2MIN / PERIOD_SECOND) {
    windowed2MinTempReadings.pop_back();
  }

  if (windowed5MinTempReadings.size() > PERIOD_5MIN / PERIOD_SECOND) {
    windowed5MinTempReadings.pop_back(); 
  }
}

void handleTimeWithinRange(unsigned long current_time) {
  if (isWithinTargetRange) {
    if (time_in_range_start == 0) {
      time_in_range_start = current_time;
    }
    time_in_range = current_time - time_in_range_start;
  } 
  else {
    time_in_range_start = 0;
    time_in_range = 0;
  }
}

void handleTemperatureBroadcast(unsigned long current_time) {
  input = readTemperature(PIN_SENSOR);

  if (myPID.GetMode() == MANUAL) {
    output = 0;
  } 
  else {
    (input >= 41 && input <= 45) ? isWithinTargetRange = true : isWithinTargetRange = false;

    handleTimeWithinRange(current_time);

    if (input > MAX_TEMP) {
      myPID.SetTunings(0, 0, 0);
      return;
    }
    
    // SET TUNINGS
    double gap = abs(set_point - input);
    if (gap < 10) {
      myPID.SetTunings(cons_kp, cons_ki, cons_kd);
    } else {
      myPID.SetTunings(agg_kp, agg_ki, agg_kd);
    }

    myPID.Compute();
  }

  double gap = abs(set_point - input);
  if (gap < 10) {
      myPID.SetTunings(cons_kp, cons_ki, cons_kd);
  } 
  else {
      myPID.SetTunings(agg_kp, agg_ki, agg_kd);
  }

  myPID.Compute();
  int heater_power = map(output, 0, 100, 0, 1023);
  analogWrite(PIN_HEATER, heater_power);

  unsigned long hours = time_in_range / 3600000;
  unsigned long minutes = (time_in_range % 3600000) / 60000;
  unsigned long seconds = (time_in_range % 60000) / 1000;

  char json[256];
  snprintf(json, sizeof(json), 
    "{\"temperature\":%.2f, \"heat_percent\":%.2f, \"min_temp2\":%.2f, \"max_temp2\":%.2f, \"min_temp5\":%.2f, \"max_temp5\":%.2f, \"time_in_range\":\"%02lu:%02lu:%02lu\"}", 
    input, output, min_temp2, max_temp2, min_temp5, max_temp5, hours, minutes, seconds);
  webSocket.broadcastTXT(json);

  Serial.print(input);
  Serial.print("°C\t\t");
  Serial.print(output);
  Serial.println("%");
}

void setup() {
  Serial.begin(115200);  
  Serial.println("\nConfiguring access point...");

  WiFi.softAP("mathieu", "!yogourt!"); 
  Serial.println("Connected to: " + WiFi.softAPIP().toString());
  LittleFS.begin();

  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_HEATER, OUTPUT);

  httpd.onNotFound([]() {
    String filename = httpd.uri();
    if (filename.endsWith("/")) filename += "index.html";
    if (LittleFS.exists(filename)) {
      File file = LittleFS.open(filename, "r");
      httpd.streamFile(file, getContentType(filename));
      file.close();
    } 
    else {
      httpd.send(404, "text/plain", "404: Not Found");
    }
  });

  httpd.begin();
  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_CONNECTED) {
      isConnected = true;
      Serial.println("Client CONNECTED");
      webSocket.sendTXT(num, "{\"temperature\": 0.00}"); 
      webSocket.sendTXT(num, "{\"heat_percent\": 0.0}"); 
      webSocket.sendTXT(num, "{\"min_temp2\": 0.00}"); 
      webSocket.sendTXT(num, "{\"max_temp2\": 0.00}"); 
      webSocket.sendTXT(num, "{\"min_temp5\": 0.00}"); 
      webSocket.sendTXT(num, "{\"max_temp5\": 0.00}");
      webSocket.sendTXT(num, "{\"time_in_range\": 0:00:00}");
    } 
    else if (type == WStype_DISCONNECTED) {
      isConnected = false;
      Serial.println("Client DISCONNECTED");
    } 
    else if (type == WStype_TEXT) {
      Serial.printf("Received message: %s\n", payload);
      payload[length] = '\0';
      if (strcmp((char *)payload, "ON") == 0) {
          myPID.SetMode(AUTOMATIC);
          Serial.println("Heater turned ON");
      } 
      else if (strcmp((char *)payload, "OFF") == 0) {
          analogWrite(PIN_HEATER, 0);
          myPID.SetTunings(0, 0, 0);
          myPID.SetMode(MANUAL);
          Serial.println("Heater turned OFF");
      }
    }
  });

  myPID.SetMode(AUTOMATIC);
}

void loop() {
  httpd.handleClient();
  webSocket.loop();

  unsigned long current_time = millis();
  if (current_time - previous_time >= PERIOD_SECOND) {
    handleTemperatureBroadcast(current_time);
    handleMinMaxTimeRangedValues();
    previous_time = current_time;
  }
}