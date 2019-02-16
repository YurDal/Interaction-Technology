// Author Yurdaer Dalkic & Hadi Deknache

#include <math.h>
#include "DHT.h"


#define DHTPIN 22     // Digital pin connected to the DHT sensor

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)


const int red_pin = 13;
const int green_pin = 13;
const int yellow_pin = 25;
const int ldr_pin = 14; //select the input pin for LDR
const int thermistor_pin = A1;

float R1 = 10000;
float logRt, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

int room_temp;
int ldr_values = 0; //variable to store the value coming from the LDR sensor
int noise_values = 0; //variable to store the value coming from the sound sensor
float temp_value_C = 0; //variable to store the value coming from the temprature sensor Celcius
float temp_value_F = 0; //variable to store the value coming from the temprature sensor Fahrenheit
float humid_value = 0; //variable to store the value coming from the temprature sensor
float thermistor_value = 0;



DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor


/**
   This method calclates the temprature value based on Stein-hart equation.

*/
float CalculateThermistor(int AnalogValue) {
  logRt = log(100000.0 * ((1024.0 / AnalogValue - 1))); //Vout= (Vin * Rt) / (R + Rt) ==> Rt = R (Vin/Vout) – 1 ==> T=1/(c1+c2*logRt+c3*logRt*logRt*logRt)
  T = (1.0 / (c1 + c2 * logRt + c3 * logRt * logRt * logRt)); // We get the temperature value in Kelvin from this Stein-Hart equation
  Tc = T - 273.15;                     // Convert Kelvin to Celsius
  Tf = (Tc * 1.8) + 32.0;              // Convert Kelvin to Fahrenheit
  return T;
}
int ReadLDR() {
  return analogRead(A0);
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

void setup() {
  Serial.begin(9600); //sets serial port for communication
  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(red_pin, OUTPUT);
  pinMode(ldr_pin, INPUT);
  dht.begin();
}

void loop() {
  ldr_values = ReadLDR();
  analogWrite(green_pin, (ldr_values / 4));
  Serial.print("LDR values :");
  Serial.println(ldr_values);
  if (ReadTempHum()) {
    if ( temp_value_C > 20) {
      digitalWrite(yellow_pin, HIGH);
    }
    else {
      digitalWrite(yellow_pin, LOW);
    }
    Serial.print("Temprature value Celcius :");
    Serial.println(temp_value_C);
    Serial.print("Temprature value Fahrenheit :");
    Serial.println(temp_value_F);
  }
  /**

    thermistor_value = CalculateThermistor(analogRead(thermistor_pin));
      if ( thermistor_value > 20) {
      digitalWrite(yellow_pin, HIGH);
    }
    else {
      digitalWrite(thermistor_value, LOW);
    }
  */


}
