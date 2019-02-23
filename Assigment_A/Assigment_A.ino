// Author Yurdaer Dalkic & Hadi Deknache
#include <analogWrite.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include "DHT.h"


#define DHTPIN 22       // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  

// Digital and analag pins connected to the LED's and Sensors
const int red_pin = 12;
const int green_pin = 13;
const int yellow_pin = 25;
const int ldr_pin = 32;
const int thermistor_pin = 27;
const int mic_pin = 33;

// Values and variables for thermistor
float R1 = 10000;
float logRt, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

const int room_temp_limit = 25; // room tempratue constant

int ldr_values         = 0; //variable to store the value coming from the LDR sensor
int noise_values       = 0; //variable to store the value coming from the microphone sensor
double temp_value_C    = 0; //variable to store the value coming from the temperature sensor Celcius
double temp_value_F    = 0; //variable to store the value coming from the temperature sensor Fahrenheit
double humid_value     = 0; //variable to store the value coming from the temperature sensor
float thermistor_value = 0; //variable to store the value coming from the temperature sensor (NTC)

/*Boolean for enabling/disabling sensor reading*/
bool LDR   = true;
bool NOISE = true;
bool TEMP  = true;
bool HUMID = true;

/*Configurations of WiFi and MQTT service*/
const char* password      = "12345678";
const char* ssid          = "Yurdaer";
const char* mqtt_server   = "m23.cloudmqtt.com";
const int mqtt_port       = 10941;
const char* mqtt_user     = "dqaqegod";
const char* mqtt_password = "JVPMD0qw7ij4";
char* inTopic             = "SensorIn";
char* outTopic            = "SensorOut";
char* helloMsg            = "Sensor is online";

char msg[100];
String sMsg  = "";
int freq     = 30; // Data transmition rate
int loopTime = 0;

WiFiClient espClient;
PubSubClient client(espClient);


DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor


void setup() {
  Serial.begin(9600); //sets serial port for communication
  setup_wifi();

  analogReadResolution(10);
  analogWriteResolution(10);
  pinMode(mic_pin, INPUT);
  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(yellow_pin, OUTPUT);
  pinMode(ldr_pin, INPUT);
  dht.begin();
  client.setServer(mqtt_server, mqtt_port);
  client.connect("ESP32Client", mqtt_user, mqtt_password);
  client.setCallback(callback);
  client.publish(outTopic, "Ready");
  client.subscribe(inTopic);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  sMsg = "";

  if (LDR) {
    ldr_values = ReadLDR();
    if (ldr_values < 800) {
      digitalWrite(green_pin, HIGH );
    }
    else {
      digitalWrite(green_pin, LOW );
    }

    sMsg = "LDR Value:" + String(ldr_values) ;
  }

  if (NOISE) {
    noise_values = ReadMIC();
    if (noise_values < 500) {
      digitalWrite(red_pin, LOW);
    }
    else {
      digitalWrite(red_pin, HIGH);
    }
    sMsg += " , MIC Value:" + String(noise_values);
  }

  if (TEMP || HUMID) {

    if (ReadTempHum()) {
      if ( temp_value_C > room_temp_limit) {
        digitalWrite(yellow_pin, HIGH);
      }
      else {
        digitalWrite(yellow_pin, LOW);
      }
    }
    if (TEMP) {

      sMsg += " , Temperature Value:" + String(temp_value_C) ;
    }
    if (HUMID) {
      sMsg += " , Humidity :" + String(humid_value);

    }
  }
  sMsg.toCharArray(msg, 100);

  thermistor_value = CalculateThermistor(analogRead(thermistor_pin));
  Serial.println(thermistor_value);
  if ( thermistor_value > room_temp_limit) {
    digitalWrite(yellow_pin, HIGH);
  }
  else {
    digitalWrite(yellow_pin, LOW);
  }



  delay(100);
  loopTime++;
  if (loopTime == 100) {
    client.publish(outTopic, msg);
    Serial.print("LDR values :");
    Serial.println(ldr_values);
    Serial.print("NOISE values :");
    Serial.println(noise_values);
    Serial.print("Temprature value Celcius :");
    Serial.println(temp_value_C);
    Serial.print("Temprature value Fahrenheit :");
    Serial.println(temp_value_F);
    Serial.print("Humidity value  :");
    Serial.println(humid_value);
    Serial.println(msg);
    loopTime = 0;
  }
}

/**
   This method calclates the temprature value based on Stein-hart equation.

*/
float CalculateThermistor(int AnalogValue) {
  logRt = log(10000.0 * ((1024.0 / AnalogValue - 1))); //Vout= (Vin * Rt) / (R + Rt) ==> Rt = R (Vin/Vout) – 1 ==> T=1/(c1+c2*logRt+c3*logRt*logRt*logRt)
  T = (1.0 / (c1 + c2 * logRt + c3 * logRt * logRt * logRt)); // We get the temperature value in Kelvin from this Stein-Hart equation
  Tc = T - 273.15;                     // Convert Kelvin to Celsius
  Tf = (Tc * 1.8) + 32.0;              // Convert Kelvin to Fahrenheit
  return T;
}

/**
   This method reads value comming rom LDR sensor
*/
int ReadLDR() {
  return analogRead(ldr_pin);
}

/**
   This method reads value coming from Sound/audio sensor
*/
int ReadMIC() {
  return analogRead(mic_pin);
}

/**
   This method reads the temprature and humidity values coming from DHT22 sensor
*/
bool ReadTempHum() {
  // Read humidity
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);
  // Check if any reads failed.
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return false;
  }
  else {
    temp_value_C = t;
    temp_value_F = f;
    humid_value = h;
    return true;
  }

}

/**
   This method does all configuration for WiFi and establish a connection to the WiFi network.
*/
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  /*
     Loop until we are connected to the network
  */
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/**
   This method checks to MQTT connection and try to re-connect to the MQTT server if the connection is down
*/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(outTopic, helloMsg);
      // ... and resubscribe
      client.subscribe(inTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/**
   MQTT callback handles messages that are received
*/
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String inMessage;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    inMessage += (char)message[i];
  }
  Serial.println();

  if (inMessage == "LDR=ON") {
    Serial.println(inMessage);
    LDR = true;
  }
  else if (inMessage.equals("LDR=OFF")) {
    LDR = false;
    digitalWrite(green_pin, LOW );
  }
  else if (inMessage == "NOISE=ON") {
    NOISE = true;
  }
  else if (inMessage == "NOISE=OFF") {
    Serial.println(inMessage);
    NOISE = false;
    digitalWrite(red_pin, LOW);
  }
  else if (inMessage == "TEMP=ON") {
    TEMP = true;
  }
  else if (inMessage == "TEMP=OFF") {
    Serial.println(inMessage);
    TEMP = false;
    digitalWrite(yellow_pin, LOW);
  }
  else if (inMessage == "HUMID=ON") {
    HUMID = true;
  }
  else if (inMessage == "HUMID=OFF") {
    HUMID = false;
  }
  else if (inMessage == "ALL=OFF") {
    LDR = false;
    NOISE = false;
    TEMP = false;
    HUMID = false;
    digitalWrite(yellow_pin, LOW);
    digitalWrite(red_pin, LOW);
    digitalWrite(green_pin, LOW );
  }
  else if (inMessage == "ALL=ON") {
    LDR = true;
    NOISE = true;
    TEMP = true;
    HUMID = true;
  }


}
