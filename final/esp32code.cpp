
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <DHT.h>
#include "MPU6050_light.h"
#include <time.h>
#include "HX711.h" // enable HX711
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

Adafruit_MPU6050 mpu;
// ----------------- CONFIG -----------------
// WiFi
const char* WIFI_SSID = "ZTE Blade A55";
const char* WIFI_PASS = "12121212";

// HiveMQ Cloud
const char* MQTT_HOST = "47009d7316b04ed9a5120c96c6b40fe6.s1.eu.hivemq.cloud"; // e.g. mycluster.hivemq.cloud
const uint16_t MQTT_PORT = 8883; // TLS
const char* MQTT_USER = "newiot";
const char* MQTT_PASS = "iotPass123321@#";

// Device identifiers
const char* TRUCK_ID = "truck01"; // change per device
const char* BASE_TOPIC = "fleet";  // topics like fleet/truck01/gps

// OpenRouteService (used by dashboard, included here for completeness)
const char* ORS_KEY = "YOUR_OPENROUTESERVICE_KEY"; // used in dashboard only

// Pins (change to your wiring)
const int GPS_RX_PIN = 16; // Neo-6M TX -> ESP32 RX2 (GPIO16)
const int GPS_TX_PIN = 17; // Neo-6M RX -> ESP32 TX2 (GPIO17)

const int ULTRASONIC_TRIG = 5;
const int ULTRASONIC_ECHO = 18;

const int DHT_PIN = 4; // DHT11 data pin
#define DHTTYPE DHT11

// MPU6050 on default I2C (SDA 21, SCL 22 on many ESP32 dev boards)


const int HX711_DOUT = 15;
const int HX711_SCK = 14;
HX711 scale;


const unsigned long INTERVAL_GPS = 2000;
const unsigned long INTERVAL_ULTRA = 3000;
const unsigned long INTERVAL_DHT = 10000;
const unsigned long INTERVAL_MPU = 500;
const unsigned long INTERVAL_PUBLISH_KEEPALIVE = 60000;

// Flip detection threshold (deg)
const float FLIP_PITCH_THRESHOLD = 120.0; // tweak for bin flip detection
const float FLIP_ROLL_THRESHOLD  = 120.0;
bool mpu_ok = false;
// ----------------- GLOBALS -----------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2

WiFiClientSecure net;
PubSubClient mqtt(net);

DHT dht(DHT_PIN, DHTTYPE);
MPU6050 mpu(Wire);

const unsigned long INTERVAL_HX711 = 5000; // every 5s
float lastWeightKg = NAN;
unsigned long lastHX711 = 0;

//HX711 scale; // optional

// timers
unsigned long lastGPS = 0;
unsigned long lastUltra = 0;
unsigned long lastDHT = 0;
unsigned long lastMPU = 0;
unsigned long lastKeepalive = 0;

// last-known values
double lastLat = 0.0, lastLon = 0.0;
float lastSpeedKmph = 0.0;
int lastSatellites = 0;
long lastDistanceCm = -1;
float lastTemp = NAN, lastHum = NAN;
bool lastFlipState = false;
float lastPitch = 0.0, lastRoll = 0.0;

// ----------------- UTIL: time / ISO8601 -----------------
void ensureTime() {
  // Configure NTP (UTC)
  const char* ntpServer = "pool.ntp.org";
  const long  gmtOffset_sec = 0; // store UTC timestamps; you can add offset
  const int   daylightOffset_sec = 0;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

String isoTimeNow() {
  time_t now;
  time(&now);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return String("1970-01-01T00:00:00Z");
  }
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo); // UTC format
  return String(buf);
}

// ----------------- WIFI & MQTT -----------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("Connecting to WiFi %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 40) {
    delay(500); Serial.print(".");
    retry++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // No control topics in this example, but you can implement commands here
  Serial.print("MQTT recv: "); Serial.println(topic);
}

void connectMQTT() {
  if (mqtt.connected()) return;
  Serial.print("Connecting to MQTT...");
  // For testing only — accepts server certs without verification
  net.setInsecure();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  String clientId = String(TRUCK_ID) + String("_") + String(random(0xffff), HEX);
  int tries = 0;
  while (!mqtt.connected() && tries < 10) {
    Serial.print(".");
    if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
      Serial.println("\nMQTT connected");
      // subscribe to control topic if needed:
      // String ctrlTopic = String("control/") + TRUCK_ID + "/#";
      // mqtt.subscribe(ctrlTopic.c_str());
      break;
    } else {
      Serial.print(" failed rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying ...");
      delay(2000);
    }
    tries++;
  }
  if (!mqtt.connected()) {
    Serial.println("MQTT connection failed");
  }
}

void publishJson(const char* topic, String payload) {
  if (!mqtt.connected()) {
    connectMQTT();
  }
  if (mqtt.connected()) {
    mqtt.publish(topic, payload.c_str());
    // Also retain last published for late dashboard joins? optional
    // mqtt.publish(topic, payload.c_str(), true); // retained
  } else {
    Serial.println("Publish failed: not connected");
  }
}

// ----------------- GPS HANDLER -----------------
void setupGPS() {
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS UART started");
}

void loopGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
    lastSpeedKmph = gps.speed.kmph();
    lastSatellites = gps.satellites.value();

    // build payload
    String payload = "{";
    payload += "\"lat\":" + String(lastLat, 6) + ",";
    payload += "\"lon\":" + String(lastLon, 6) + ",";
    payload += "\"speed_kmh\":" + String(lastSpeedKmph, 2) + ",";
    payload += "\"satellites\":" + String(lastSatellites) + ",";
    payload += "\"ts\":\"" + isoTimeNow() + "\"";
    payload += "}";
    String topic = String(BASE_TOPIC) + "/" + TRUCK_ID + "/gps";
    publishJson(topic.c_str(), payload);
    Serial.println("GPS published: " + payload);
  }
}

// ----------------- ULTRASONIC HANDLER -----------------
long readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // pulseIn timeout to avoid blocking forever (max 25ms -> ~4m)
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL); // 30ms timeout
  if (duration == 0) return -1;
  long cm = (long)(duration * 0.0343 / 2.0);
  return cm;
}

void setupUltrasonic() {
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
}

void loopUltrasonic() {
  long d = readUltrasonicCM(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
  lastDistanceCm = d;
  String payload = "{";
  payload += "\"bin_id\":\"BIN_K01\","; // change per sensor mounting
  payload += "\"distance_cm\":" + String(d) + ",";
  payload += "\"ts\":\"" + isoTimeNow() + "\"";
  payload += "}";
  String topic = String(BASE_TOPIC) + "/" + TRUCK_ID + "/ultrasonic";
  publishJson(topic.c_str(), payload);
  Serial.println("Ultrasonic published: " + payload);
}

// ----------------- DHT HANDLER -----------------
void setupDHT() {
  dht.begin();
}

void loopDHT() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    Serial.println("DHT read failed");
    return;
  }
  lastTemp = t; lastHum = h;
  String payload = "{";
  payload += "\"temp_c\":" + String(t, 2) + ",";
  payload += "\"hum_pct\":" + String(h, 2) + ",";
  payload += "\"ts\":\"" + isoTimeNow() + "\"";
  payload += "}";
  String topic = String(BASE_TOPIC) + "/" + TRUCK_ID + "/dht";
  publishJson(topic.c_str(), payload);
  Serial.println("DHT published: " + payload);
}

// ----------------- MPU6050 (Flip) HANDLER -----------------
void setupMPU() {
  Wire.begin(21,22); // ESP32 default SDA/SCL
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found ⚠️");
    mpu_ok = false;
    return;
  }

  mpu_ok = true;
  Serial.println("MPU6050 ready ✅");

  // set ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100); // let it settle
}

bool detectFlipEvent(float &pitch, float &roll) {
  if (!mpu_ok) return false;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // compute roll/pitch from accel (same math you had)
  roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  pitch = atan2(-a.acceleration.x, 
                 sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) 
          * 180.0 / PI;

  if (pitch < -180) pitch += 360;
  if (roll  < -180) roll  += 360;

  // check thresholds
  if (abs(pitch) > FLIP_PITCH_THRESHOLD || abs(roll) > FLIP_ROLL_THRESHOLD) {
    return true;
  }

  return false;
}

void loopMPU() {
  float pitch = 0.0, roll = 0.0;
  bool flip = detectFlipEvent(pitch, roll);
  lastPitch = pitch; lastRoll = roll;

  if (flip && !lastFlipState) { // rising edge only
    lastFlipState = true;
    String payload = "{";
    payload += "\"flip\":true,";
    payload += "\"pitch\":" + String(pitch, 2) + ",";
    payload += "\"roll\":" + String(roll, 2) + ",";
    payload += "\"bin_id\":\"BIN_K01\","; // optional
    payload += "\"ts\":\"" + isoTimeNow() + "\"";
    payload += "}";
    String t0 = String(BASE_TOPIC) + "/" + TRUCK_ID + "/mpu";
    String tAlert = String("alerts/flip");
    publishJson(t0.c_str(), payload);
    publishJson(tAlert.c_str(), payload); // broadcast alert
    Serial.println("Flip published: " + payload);
  } else if (!flip) {
    lastFlipState = false;
  }
}

// ----------------- HX711 (weight) placeholder -----------------
void loopHX711() {
  if (scale.is_ready()) {
    float weight = scale.get_units(10); // avg 10 samples
    lastWeightKg = weight;

    String payload = "{";
    payload += "\"bin_id\":\"BIN_K01\","; // change per sensor mounting
    payload += "\"weight_kg\":" + String(weight, 2) + ",";
    payload += "\"ts\":\"" + isoTimeNow() + "\"";
    payload += "}";

    String topic = String(BASE_TOPIC) + "/" + TRUCK_ID + "/hx711";
    publishJson(topic.c_str(), payload);
    Serial.println("HX711 published: " + payload);
  } else {
    Serial.println("HX711 not ready");
  }
}


// ----------------- SETUP & LOOP -----------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  randomSeed(analogRead(0));

  connectWiFi();
  ensureTime(); // NTP

  setupGPS();
  setupUltrasonic();
  setupDHT();
  setupMPU();
  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(2280.f);   // ⚖️ calibration factor (adjust!)
  scale.tare();              // zero out the scale
  Serial.println("HX711 scale initialized");
  connectMQTT();

  Serial.println("Setup done.");
}

void loop() {
  unsigned long now = millis();

  // Keep mqtt alive
  if (now - lastKeepalive > INTERVAL_PUBLISH_KEEPALIVE) {
    if (mqtt.connected()) {
      // publish a small heartbeat
      String hb = "{\"ts\":\"" + isoTimeNow() + "\"}";
      String hbTopic = String(BASE_TOPIC) + "/" + TRUCK_ID + "/heartbeat";
      mqtt.publish(hbTopic.c_str(), hb.c_str());
    }
    lastKeepalive = now;
  }

  // handle mqtt client loop
  if (mqtt.connected()) mqtt.loop();
  else connectMQTT();

  // GPS interval
  if (now - lastGPS >= INTERVAL_GPS) {
    loopGPS();
    lastGPS = now;
  }

  // Ultrasonic interval
  if (now - lastUltra >= INTERVAL_ULTRA) {
    loopUltrasonic();
    lastUltra = now;
  }

  // DHT interval
  if (now - lastDHT >= INTERVAL_DHT) {
    loopDHT();
    lastDHT = now;
  }

  // MPU interval
  if (now - lastMPU >= INTERVAL_MPU) {
    loopMPU();
    lastMPU = now;
  }
    // HX711 interval
  if (now - lastHX711 >= INTERVAL_HX711) {
    loopHX711();
    lastHX711 = now;
  }

  // light sleep / small pause to avoid starving CPU
  delay(10);
}
