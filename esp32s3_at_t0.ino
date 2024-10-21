/*
For ESP32S3 UWB AT Demo


Use 2.0.0   Wire
Use 1.11.7   Adafruit_GFX_Library
Use 1.14.4   Adafruit_BusIO
Use 2.0.0   SPI
Use 2.5.7   Adafruit_SSD1306

*/

// User config          ------------------------------------------

#define UWB_INDEX 0

#define TAG

#define FREQ_850K

#define UWB_TAG_COUNT 64

// User config end       ------------------------------------------

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <MPU9250_asukiaaa.h>

#define SERIAL_LOG Serial
#define SERIAL_AT mySerial2

HardwareSerial SERIAL_AT(2);

#define RESET 16

#define IO_RXD2 18
#define IO_TXD2 17

#define I2C_SDA 39
#define I2C_SCL 38
#define IMU_SDA 6
#define IMU_SCL 5


MPU9250_asukiaaa mySensor;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// WiFi
const char *ssid = "Frentzen"; // Enter your Wi-Fi name
const char *password = "123456abc";  // Enter Wi-Fi password

// MQTT Broker
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "uwb/location";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

unsigned long lastPublishTime = 0; // Global variable to track the last publish time
const unsigned long publishInterval = 500; // 5-second interval between publishes
float gZ, yawGyro, yaw;
float gZOffset = 0;
unsigned long lastTime, currentTime;
float dt;

WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
    Serial.begin(115200);

    //IMU Setup
    Wire.begin(IMU_SDA, IMU_SCL);
    mySensor.setWire(&Wire);
    mySensor.beginGyro();

    // Perform calibration (add code to show at oled)
    calibrateSensor();
    lastTime = millis();

    // Connecting to a WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");
    //connecting to a mqtt broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public EMQX MQTT broker connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    // Publish and subscribe
    client.publish(topic, "Hi, I'm ESP32 ^^");
    client.subscribe(topic);

    // Original
    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, HIGH);

    SERIAL_LOG.begin(115200);

    SERIAL_LOG.print(F("Hello! ESP32-S3 AT command V1.0 Test"));
    SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);

    SERIAL_AT.println("AT");
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(1000);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3C for 128x32
        SERIAL_LOG.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    display.clearDisplay();

    logoshow();

    sendData("AT?", 2000, 1);
    sendData("AT+RESTORE", 5000, 1);

    sendData(config_cmd(), 2000, 1);
    sendData(cap_cmd(), 2000, 1);

    sendData("AT+SETRPT=1", 2000, 1);
    sendData("AT+SAVE", 2000, 1);
    sendData("AT+RESTART", 2000, 1);
}

long int runtime = 0;

String response = "";
String rec_head = "AT+RANGE";
String output;

void loop()
{
    client.loop();
    // put your main code here, to run repeatedly:
    getIMUData();
    while (SERIAL_LOG.available() > 0)
    {
        SERIAL_AT.write(SERIAL_LOG.read());
        yield();
    }
    while (SERIAL_AT.available() > 0)
    {
        char c = SERIAL_AT.read();

        if (c == '\r')
            continue;
        else if (c == '\n' || c == '\r')
        {   
            response += ",yaw:(" + String(yaw) + ")"; // Append yaw to command
            Serial.println("This is the response: ");
            Serial.println(response.c_str());
            SERIAL_LOG.println(response);

            // Only publish if the time since the last publish exceeds the interval
            if (millis() - lastPublishTime >= publishInterval) {
                client.publish(topic, response.c_str());  // Convert String to C-string before publishing
                lastPublishTime = millis();  // Update the last publish time
            }


            response = "";
        }
        else
            response += c;
    }
}


void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}

// SSD1306

void logoshow(void)
{
    display.clearDisplay();

    display.setTextSize(1);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner
    display.println(F("MaUWB DW3000"));

    display.setCursor(0, 20); // Start at top-left corner
    // display.println(F("with STM32 AT Command"));

    display.setTextSize(2);

    String temp = "";

    temp = temp + "T" + UWB_INDEX;

    temp = temp + "   850k";

    display.println(temp);

    display.setCursor(0, 40);

    temp = "Total: ";
    temp = temp + UWB_TAG_COUNT;
    display.println(temp);

    display.display();

    delay(2000);
}

String sendData(String command, const int timeout, boolean debug)
{
    String response = "";
    // command = command + "\r\n";

    SERIAL_LOG.println(command);
    SERIAL_AT.println(command); // send the read character to the SERIAL_LOG

    long int time = millis();

    while ((time + timeout) > millis())
    {
        while (SERIAL_AT.available())
        {

            // The esp has data so display its output to the serial window
            char c = SERIAL_AT.read(); // read the next character.
            response += c;
        }
    }

    if (debug)
    {
        SERIAL_LOG.println(response);
    }

    return response;
}

String config_cmd()
{
    String temp = "AT+SETCFG=";

    // Set device id
    temp = temp + UWB_INDEX;

    // Set device role

    temp = temp + ",0";

    // Set frequence 850k or 6.8M

    temp = temp + ",0";

    // Set range filter
    temp = temp + ",1";

    return temp;
}

String cap_cmd()
{
    String temp = "AT+SETCAP=";

    // Set Tag capacity
    temp = temp + UWB_TAG_COUNT;

    //  Time of a single time slot

    temp = temp + ",15";

    return temp;
}

void calibrateSensor() {
  const int numSamples = 1000;
  float gyroZSum = 0;

  Serial.println("Calibrating...");

  for (int i = 0; i < numSamples; i++) {
    mySensor.gyroUpdate();

    // Accumulate readings
    gyroZSum += mySensor.gyroZ();

    delay(10); // Small delay between readings
  }

  // Calculate averages (offsets)
  gZOffset = gyroZSum / numSamples;

  Serial.println("Calibration complete!");
  Serial.println("Gyro Z Offset: " + String(gZOffset));
}

void getIMUData(){
  // Update sensor data
  mySensor.gyroUpdate();

  // Get current time
  currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // Convert to seconds

  // Apply gyroscope offset
  gZ = mySensor.gyroZ() - gZOffset;

  // Calculate yaw from gyroscope data (integrate angular velocity)
  yawGyro += gZ * dt;

  // Normalize yaw to range 0-360 degrees
  yaw = fmod(yawGyro, 360.0);
  if (yaw < 0) yaw += 360;

  // Output yaw
  // Serial.print("Yaw: ");
  // Serial.println(yaw);

  lastTime = currentTime;
}
