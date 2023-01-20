#include <LiquidCrystal_I2C.h>
#include <Stepper.h>
#include "DHT.h"

DHT dht = DHT(A9,11);

const uint8_t Water_Sensor = 22U;
const uint8_t SWITCH = 4U;
const uint8_t RGBR = 7U;
const uint8_t LED_PIN1 = 51;
const uint8_t LED_PIN2 = 52U;
const uint8_t LED_PIN3 = 53U;
int step_per_revolution = 2048;
class Stepper stepping(step_per_revolution,13,11,12,10);
class LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200UL);
  dht.begin();
  pinMode(SWITCH,INPUT);
  pinMode(Water_Sensor,INPUT);
  pinMode(RGBR,OUTPUT);
  pinMode(LED_PIN1,OUTPUT);
  pinMode(LED_PIN2,OUTPUT);
  pinMode(LED_PIN3,OUTPUT);

  stepping.setSpeed(14L); //60 RPM
  lcd.backlight();
  lcd.init();
  lcd.print("LCD init");
  delay(1000UL);
  lcd.clear();
}

void loop() {
  // put your main code here, to run repeatedly:
  const uint8_t value = digitalRead(SWITCH);
  if(value == 0){
    if(dht.read()){
      float temperature {dht.readTemperature()};
      Serial.println(temperature);
      if(temperature > 31 )
      {
       lcd.setCursor(0,0);
       lcd.print("WARNING");
       lcd.setCursor(0,1);
       lcd.print("WARNING");
       digitalWrite(RGBR, HIGH);
       digitalWrite(LED_PIN1,HIGH);
       digitalWrite(LED_PIN2,HIGH);
       digitalWrite(LED_PIN3,HIGH);

        delay(500UL);
        digitalWrite(RGBR, LOW);
        digitalWrite(LED_PIN1,LOW);
        digitalWrite(LED_PIN2,LOW);
        digitalWrite(LED_PIN3,LOW);
        delay(500UL);
      }   
      else if(temperature <= 31){
        if(digitalRead(Water_Sensor) == true){
          lcd.print("Today is");
          lcd.setCursor(0,1);
          lcd.println("raining_Day      ");
          Serial.println(digitalRead(Water_Sensor));
          stepping.step(-step_per_revolution);
          delay(1000U);
          stepping.step(step_per_revolution);
          delay(1000U);
        }
        else
        {
          lcd.setCursor(0,0);
          lcd.print("Today is");
          lcd.setCursor(0,1);
          lcd.println("fine day        ");
          stepping.step(0);
          delay(1000U);
        }
     }
    }
  }
  else
  {
    digitalWrite(RGBR, LOW);
    digitalWrite(LED_PIN1,LOW);
    digitalWrite(LED_PIN2,LOW);
    digitalWrite(LED_PIN3,LOW);
    stepping.step(0);
    lcd.println("Terminate        ");
  }  
}
