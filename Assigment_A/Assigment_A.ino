// Author Yurdaer Dalkic & Hadi Deknache
#include <analogWrite.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include "DHT.h"


#define DHTPIN 22     // Digital pin connected to the DHT sensor

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)


const int red_pin = 12;
const int green_pin = 13;
const int yellow_pin = 25;
const int ldr_pin = 14; //select the input pin for LDR
const int thermistor_pin = 27;
const int mic_pin = 33;

float R1 = 10000;
float logRt, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
const int room_temp_limit = 25;;
int ldr_values = 0; //variable to store the value coming from the LDR sensor
int noise_values = 0; //variable to store the value coming from the sound sensor
double temp_value_C = 0; //variable to store the value coming from the temprature sensor Celcius
double temp_value_F = 0; //variable to store the value coming from the temprature sensor Fahrenheit
double humid_value = 0; //variable to store the value coming from the temprature sensor
float thermistor_value = 0;

bool LDR = false;
bool NOISE = false;
bool TEMP = false;
bool HUMID = false;

const char* password = "206264D2480";
const char* ssid = "Tele2Internet-9EB85";
const char* mqtt_server = "m23.cloudmqtt.com";
const int mqtt_port =   10941;
const char* mqtt_user = "Weareble";
const char* mqtt_password = "Weareble";
char* inTopic = "Project/InteractionIn";
char* outTopic = "Project/InteractionOut";
char* helloMsg = "Sensor is online";
char msg[50];
String sMsg = "";

WiFiClient espClient;
PubSubClient client(espClient);


DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor


void setup() {
  Serial.begin(9600); //sets serial port for communication
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  analogReadResolution(10);
  analogWriteResolution(10);
  pinMode(mic_pin, INPUT);
  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(yellow_pin, OUTPUT);
  pinMode(ldr_pin, INPUT);
  dht.begin();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  sMsg = "";

  if (LDR) {
    ldr_values = ReadLDR();
    int light_value = (1023 - (ldr_values - 700) * 3.5);
    if (light_value < 200) {
      light_value = 0;
    }
    else if (light_value > 900) {
      light_value = 1023;
    }
    analogWrite(green_pin, light_value );
    Serial.print("LDR values :");
    Serial.println(ldr_values);
    sMsg += " , LDR Value:" + ldr_values;

  }

  else if (NOISE) {
    noise_values = ReadMIC();
    if (noise_values < 500) {
      digitalWrite(red_pin, LOW);
    }
    else {
      digitalWrite(red_pin, HIGH);
    }

    Serial.print("NOISE values :");
    Serial.println(noise_values);
    sMsg += " , MIC Value:" + noise_values;
  }

  else if (TEMP || HUMID) {

    if (ReadTempHum()) {
      if ( temp_value_C > room_temp_limit) {
        digitalWrite(yellow_pin, HIGH);
      }
      else {
        digitalWrite(yellow_pin, LOW);
      }
    }
    if (TEMP) {
      Serial.print("Temprature value Celcius :");
      Serial.println(temp_value_C);
      Serial.print("Temprature value Fahrenheit :");
      Serial.println(temp_value_F);
      sMsg += " , Temperature Value:" + String(temp_value_C) ;
    }
    else if (HUMID) {
      sMsg += " , Humidity :" + String(humid_value);
      Serial.print("Humidity value  :");
      Serial.println(humid_value);
    }
  }
  sMsg.toCharArray(msg, 50);
  client.publish(outTopic, msg);
  /*
    thermistor_value = CalculateThermistor(analogRead(thermistor_pin));
    Serial.println(thermistor_value);
    if ( thermistor_value > room_temp_limit) {
      digitalWrite(yellow_pin, HIGH);
    }
    else {
      digitalWrite(thermistor_value, LOW);
    }
  */
  delay(500);

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
int ReadLDR() {
  return analogRead(ldr_pin);
}
int ReadMIC() {
  return analogRead(mic_pin);
}
bool ReadTempHum() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
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

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
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
  if (String(topic) == "Project/InteractionIn") {
    if (inMessage == "LDR=ON") {
      Serial.println(inMessage);
      LDR = true;
    }
    else if (inMessage == "LDR=OFF") {
      Serial.println(inMessage);
      LDR = false;
      analogWrite(green_pin, 0 );
    }
    else if (inMessage == "NOISE=ON") {
      Serial.println(inMessage);
      NOISE = true;
    }
    else if (inMessage == "NOISE=OFF") {
      Serial.println(inMessage);
      NOISE = false;
      digitalWrite(red_pin, LOW);
    }
    else if (inMessage == "TEMP=ON") {
      Serial.println(inMessage);
      TEMP = true;
    }
    else if (inMessage == "TEMP=OFF") {
      Serial.println(inMessage);
      TEMP = false;
      digitalWrite(yellow_pin, LOW);
    }
    else if (inMessage == "HUMID=ON") {
      Serial.println(inMessage);
      HUMID = true;
    }
    else if (inMessage == "HUMID=OFF") {
      Serial.println(inMessage);
      HUMID = false;
    }
    else if (inMessage == "ALL=OFF") {
      Serial.println(inMessage);
      bool LDR = false;
      bool NOISE = false;
      bool TEMP = false;
      bool HUMID = false;
      digitalWrite(yellow_pin, LOW);
      digitalWrite(red_pin, LOW);
      analogWrite(green_pin, 0 );
    }
    else if (inMessage == "ALL=ON") {
      Serial.println(inMessage);
      bool LDR = true;
      bool NOISE = true;
      bool TEMP = true;
      bool HUMID = true;
    }
  }
}
