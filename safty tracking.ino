#define BLYNK_TEMPLATE_ID "TMPL3dxZ6zx77"
#define BLYNK_TEMPLATE_NAME "iot fall detection system"
#define BLYNK_AUTH_TOKEN "a19ptEVfogqz9uNIfDvp7E5yd3CSpEfV"

#define BLYNK_PRINT Serial
#define DHTTYPE DHT11 

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <DHT.h>
       
  


const int DHTPIN = 4; 
const int pulsePin = 32; 
int pulseValue; 
int bpm; 

Adafruit_MPU6050 mpu;
DHT dht(DHTPIN, DHTTYPE);

char auth[] = "a19ptEVfogqz9uNIfDvp7E5yd3CSpEfV";

char ssid[] = "9400";
char pass[] = "88888888";

BlynkTimer timer;

void sendSensor()
{pinMode(pulsePin, INPUT);
  if(mpu.getMotionInterruptStatus()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

 
  float accelerationX = a.acceleration.x;
  float accelerationY = a.acceleration.y;
  float accelerationZ = a.acceleration.z;
  float totalAcceleration = sqrt(pow(accelerationX, 2) + pow(accelerationY, 2) + pow(accelerationZ, 2));
  Serial.println(totalAcceleration);
  
    
    
    Blynk.virtualWrite(V2, a.acceleration.z);
    Blynk.virtualWrite(V3,g.gyro.x);
    Blynk.virtualWrite(V4,g.gyro.y);
    Blynk.virtualWrite(V5,totalAcceleration);
   
   delay(200);
  }
}
void setup()
{   
  
   Serial.begin(115200);
    while (!Serial)
    delay(10); 

  Serial.println("Adafruit MPU6050 test!");

  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); 
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
 
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(100L, sendSensor);

  dht.begin();
 
  }

void loop()
{
  pulseValue = analogRead(pulsePin);

  if (pulseValue > 600) {
    
    bpm = 60000 / pulseValue +30; 
    Serial.print("Heart rate: ");
    Serial.print(bpm);
    Serial.println(" BPM");

    // Send the heart rate to Blynk
    Blynk.virtualWrite(V6, bpm);}                          //bpm ends

 

  float temperature = dht.readTemperature();       
  float humidity = dht.readHumidity();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C\t");
  Blynk.virtualWrite(V0,temperature);

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");  
  Blynk.virtualWrite(V1, humidity);                        //dth ends

   
   
   delay(1000);
  Blynk.run();
  timer.run();
}
