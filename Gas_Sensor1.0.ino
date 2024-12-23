#include <WiFi.h>
#include <WebServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Preferences.h>

// --- Constants and Pin Definitions ---
const char *apSSID = "Gas_Sensor";
const char *apPassword = "1234567890";
const char *serverIP = "88.222.241.250";  // AWS Server IP
const int serverPort = 9999;           // Server Port

const int ledPin = 2;       // Onboard LED Pin
const int trigPin = 12;     // Ultrasonic Sensor Trigger Pin
const int echoPin = 14;     // Ultrasonic Sensor Echo Pin
const int gasSensorPin = 32; // MQ5 Gas Sensor Pin
const int buzzerPin = 13;   // Buzzer Pin
const int bootButtonPin = 0; // BOOT button Pin

const float R0 = 10.0;      // Resistance in clean air (kOhms)
const int maxDistance = 500; // Maximum distance (cm)
const unsigned long sendInterval = 30000; // Data send interval (ms)

// --- Global Variables ---
String homeSSID = "";
String homePassword = "";
String macAddress;
int packetCount = 0;
unsigned long lastSentTime = 0;

// Wi-Fi and Web Server
WebServer server(80);
WiFiClient client;
Preferences preferences;

// NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 30000); // UTC+5:30 (IST)

// --- Function Declarations ---
void configureWiFi();
void handleWebServer();
int measureDistance();
float readGasValue();
String readGasType();
String getFormattedDateTime();
void sendDataToServer(String mac, String protocolVersion, String packetType, int packetCount, int distance, int rssi, float downlinkFrequency, float temperature, int batteryPercentage, String uplinkType, String dateTime, String gasType, float gasValue);
String toHex(const String &input);
String getUplinkType();
void resetWiFiCredentials();

// --- Setup Function ---
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting ESP32...");

  // Initialize pins
  pinMode(ledPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(gasSensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(bootButtonPin, INPUT_PULLUP);

  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);

  // Initialize Preferences
  preferences.begin("wifi-creds", true);
  homeSSID = preferences.getString("ssid", "");
  homePassword = preferences.getString("password", "");
  preferences.end();

  if (homeSSID != "" && homePassword != "") {
    Serial.println("Found saved Wi-Fi credentials.");
    WiFi.begin(homeSSID.c_str(), homePassword.c_str());
    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(homeSSID);

    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20) {
      delay(500);
      Serial.print(".");
      timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to Wi-Fi!");
      macAddress = WiFi.macAddress();

      // Blink LED to indicate success
      for (int i = 0; i < 6; i++) {
        digitalWrite(ledPin, i % 2 == 0 ? HIGH : LOW);
        delay(200);
      }
      digitalWrite(ledPin, LOW);

      timeClient.begin();
    } else {
      Serial.println("\nFailed to connect to Wi-Fi. Starting Access Point.");
      WiFi.softAP(apSSID, apPassword);
      configureWiFi();
    }
  } else {
    Serial.println("No saved Wi-Fi credentials. Starting Access Point.");
    WiFi.softAP(apSSID, apPassword);
    configureWiFi();
  }

  server.begin();
} 

// --- Loop Function ---
void loop() {
  server.handleClient(); // Handle Wi-Fi configuration page requests

  unsigned long currentTime = millis();
  int distance = measureDistance();
  float gasValue = readGasValue();
  String gasType = readGasType();

  if (digitalRead(bootButtonPin) == LOW) {
    delay(1000); // Debounce delay
    if (digitalRead(bootButtonPin) == LOW) {
      resetWiFiCredentials();
    }
  }

  if (currentTime - lastSentTime >= sendInterval) {
    lastSentTime = currentTime;

    int rssi = WiFi.RSSI();
    int wifiChannel = WiFi.channel();
    float downlinkFrequency = 2407 + (wifiChannel * 5);
    float temperature = random(20, 36);
    int batteryPercentage = random(10, 101);
    String uplinkType = getUplinkType();
    String protocolVersion = "1.0";
    String packetType = "telemetry";
    packetCount++;

    // Trigger buzzer if hazardous gas is detected
    if (gasType == "LPG" || gasType == "Methane" || gasType == "Hydrogen") {
      digitalWrite(buzzerPin, HIGH);
    } else {
      digitalWrite(buzzerPin, LOW);
    }

    timeClient.update();
    String dateTime = getFormattedDateTime();

    sendDataToServer(macAddress, protocolVersion, packetType, packetCount, distance, rssi, downlinkFrequency, temperature, batteryPercentage, uplinkType, dateTime, gasType, gasValue);
  }

  delay(1000); // A short delay for the loop to avoid continuous execution
}

// --- Function Definitions ---
void configureWiFi() {
  server.on("/", HTTP_GET, []() {
    String html = "<html><body>"
                  "<h2>Enter Wi-Fi Credentials</h2>"
                  "<form action='/submit' method='POST'>"
                  "SSID: <input type='text' name='ssid'><br>"
                  "Password: <input type='password' name='password'><br>"
                  "<input type='submit' value='Submit'>"
                  "</form>"
                  "</body></html>";
    server.send(200, "text/html", html);
  });

  server.on("/submit", HTTP_POST, []() {
    if (server.hasArg("ssid") && server.hasArg("password")) {
      homeSSID = server.arg("ssid");
      homePassword = server.arg("password");

      preferences.begin("wifi-creds", false);
      preferences.putString("ssid", homeSSID);
      preferences.putString("password", homePassword);
      preferences.end();

      Serial.println("Restarting ESP32 to connect with new credentials...");
      ESP.restart();
    }
  });
}

void resetWiFiCredentials() {
  Serial.println("Resetting Wi-Fi credentials...");
  preferences.begin("wifi-creds", false);
  preferences.clear();
  preferences.end();

  Serial.println("Wi-Fi credentials cleared. Restarting ESP32...");
  delay(1000);
  ESP.restart();
}

int measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;

  int distance = duration * 0.034 / 2;
  if (distance < 0 || distance > maxDistance) return -1;
  return distance;
}

float readGasValue() {
  int sensorValue = analogRead(gasSensorPin);
  float voltage = sensorValue * (3.3 / 4095.0);
  float RS = ((3.3 * R0) / voltage) - R0;
  return RS / R0;
}

String readGasType() {
  float ratio = readGasValue();
  if (ratio < 1.5) return "LPG";
  if (ratio < 3.0) return "Methane";
  if (ratio < 6.0) return "Hydrogen";
  return "Unknown Gas";
}

String getFormattedDateTime() {
  time_t now = timeClient.getEpochTime();
  struct tm* timeinfo = gmtime(&now);
  char buffer[20];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d", 
          1900 + timeinfo->tm_year,
          1 + timeinfo->tm_mon,
          timeinfo->tm_mday,
          timeinfo->tm_hour,
          timeinfo->tm_min,
          timeinfo->tm_sec);
  return String(buffer);
}

void sendDataToServer(String mac, String protocolVersion, String packetType, int packetCount, int distance, int rssi, float downlinkFrequency, float temperature, int batteryPercentage, String uplinkType, String dateTime, String gasType, float gasValue) {
  if (client.connect(serverIP, serverPort)) {
    Serial.println("Connected to server, preparing data...");

    const int noiseFloor = -90;
    int snr = rssi - noiseFloor;
    String binsensor = "Gas Sensor";

    // Prepare the JSON payload
    String payload = "{";
    payload += "\"protocol_version\": \"" + protocolVersion + "\", ";
    payload += "\"packet_type\": \"" + packetType + "\", ";
    payload += "\"packet_count\": " + String(packetCount) + ", ";
    payload += "\"snr\": " + String(snr) + ", ";
    payload += "\"distance\": " + String(distance) + ", ";
    payload += "\"sensor_type\": \"" + binsensor + "\", ";
    payload += "\"rssi\": " + String(rssi) + ", ";
    payload += "\"downlink_frequency\": " + String(downlinkFrequency) + ", ";
    payload += "\"temperature\": " + String(temperature) + ", ";
    payload += "\"battery_percentage\": " + String(batteryPercentage) + ", ";
    payload += "\"uplink_type\": \"" + uplinkType + "\", ";
    payload += "\"timestamp\": \"" + dateTime + "\"";
    payload += "}";

    // Convert the entire payload to hexadecimal
    String hexPayload = toHex(payload);

    // Prepare the final data with some fields as plain text
    String data = "{";
    data += "\"uuid\": \"" + mac + "\", ";                // Plain text
    data += "\"gas_type\": \"" + gasType + "\", ";        // Plain text
    data += "\"gas_value\": \"" + String(gasValue, 2) + "\", "; // Plain text
    data += "\"data_size\": " + String(hexPayload.length() / 2) + ", "; // Each hex pair represents 1 byte
    data += "\"payload\": \"" + hexPayload + "\"";        // Hexadecimal
    data += "}";

    // Send the data to the server
    client.println(data);
    client.stop();

    Serial.println("Data sent with mixed encoding: " + data);
  } else {
    Serial.println("Connection to server failed");
  }
}

String toHex(const String &input) {
  String hex = "";
  for (size_t i = 0; i < input.length(); ++i) {
    if (input[i] < 16) hex += '0';
    hex += String(input[i], HEX);
  }
  return hex;
}

String getUplinkType() {
  return "Wi-Fi"; // Example uplink type
}
