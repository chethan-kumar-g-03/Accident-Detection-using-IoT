#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <WiFi.h>
#include <WiFiClient.h>
#define BLYNK_TEMPLATE_ID "<Insert ID Here>"
#define BLYNK_TEMPLATE_NAME "<Insert Template Name here>"
#define BLYNK_AUTH_TOKEN "<Insert Authentication token Here>"
#include <BlynkSimpleEsp32.h>
#define PI 22/7

Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

float ax, ay, az, gx, gy, gz, mx, my, mz;
float yaw=0.0, pitch=0.0, roll=0.0;
float yawp=0.0, pitchp=0.0, rollp=0.0;
int buzz = 19;
int crash = 18;
char ssid[] = "POCO F4";
char pass[] = "00000000";
char auth[] = BLYNK_AUTH_TOKEN;

void initializeHardware() {
    pinMode(buzz, OUTPUT);
    pinMode(crash, INPUT);
}

void connectToWiFi() {
    WiFi.begin(ssid, pass);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
}

void initializeSensors() {
    Serial.println("Initializing sensors...");
    mpu.begin();
    compass.init();
    Serial.println("Done!");
}

void readSensors() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    compass.read();
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;
    mx = compass.getX();
    my = compass.getY();
    mz = compass.getZ();
}

void computeAngles()
{
    pitch = atan2(ay, sqrt((ax * ax) + (az * az)));
    roll = atan2(-ax, sqrt((ay * ay) + (az * az)));
    float Yh = (my * cos(roll)) - (mz * sin(roll));
    float Xh = (mx * cos(pitch)) + (my * sin(roll) * sin(pitch)) + (mz * cos(roll) * sin(pitch));
    yaw = atan2(Yh, Xh);
    roll = roll * 65.5;
    pitch = pitch * 65.5;
    yaw = yaw * 65.5;
}

void sendDataToBlynk()
{
  Blynk.virtualWrite(V0, pitch); 
  Blynk.virtualWrite(V1, yaw);
  Blynk.virtualWrite(V2, roll);
  Blynk.virtualWrite(V3, "Safe");
}

void checkAccident()
{
  if ( digitalRead(crash) == LOW || pitch > 60 || roll > 60 || roll < -60 || pitch < -60 )   
    {
      Serial.println("Accident has occurred");
      Blynk.virtualWrite(V3, "ACCIDENT HAS OCCURED");
      Blynk.logEvent("accident_detect", "Accident_Detect");
      analogWrite(buzz, 400);
      delay(200);
      analogWrite(buzz,0);
    }
    yawp=yaw;
    rollp=roll;
    pitchp=pitch;
}

void printSensorData()
{
    Serial.print("A X: ");
    Serial.print(ax);
    Serial.print(" Y: ");
    Serial.print(ay);
    Serial.print(" Z: ");
    Serial.print(az);
    Serial.print("  G X: ");
    Serial.print(gx);
    Serial.print(" Y: ");
    Serial.print(gy);
    Serial.print(" Z: ");
    Serial.println(gz);
    Serial.print("M X: ");
    Serial.print(0.0001 * mx);
    Serial.print(" Y: ");
    Serial.print(0.0001 * my);
    Serial.print(" Z: ");
    Serial.print(0.0001 * mz);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(", Roll: ");
    Serial.print(roll);
    Serial.print(", Yaw: ");
    Serial.println(yaw);
}

void setup()
{
    Serial.begin(115200);
    Blynk.begin(auth, ssid, pass);
    //Blynk.virtualWrite(V3, "Safe");
    initializeHardware();
    connectToWiFi();
    initializeSensors();
}

void loop() {
    Blynk.run();
    readSensors();
    computeAngles();
    sendDataToBlynk();
    printSensorData();
    checkAccident();
    delay(100);
}
