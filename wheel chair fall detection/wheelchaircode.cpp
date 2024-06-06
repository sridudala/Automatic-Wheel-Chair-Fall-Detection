#include <Wire.h>
#include <MPU6050.h>

  #include <SoftwareSerial.h>

#define rxPin 7
#define txPin 8
SoftwareSerial sim800L(rxPin,txPin); 

String buff;

MPU6050 mpu;

const int buzzerPin = 10; // Connect the buzzer to digital pin 8
const int threshold = 14000; // Set the threshold for acceleration

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize the MPU6050
  mpu.initialize();

  // Set the MPU6050 to full scale range of accelerometer (+/- 2g)
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // Set the MPU6050 to full scale range of gyroscope (+/- 250 degrees/sec)
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  pinMode(buzzerPin, OUTPUT);
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  
  //Begin serial communication with Arduino and SIM800L
  sim800L.begin(9600);

  Serial.println("Initializing...");
  
  sim800L.println("AT");
  waitForResponse();

  sim800L.println("ATE1");
  waitForResponse();

  sim800L.println("AT+CMGF=1");
  waitForResponse();

  sim800L.println("AT+CNMI=1,2,0,0,0");
  waitForResponse();
}

void loop() {
  // Read accelerometer data
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Print individual x, y, z acceleration values to the serial monitor
  Serial.print("Acceleration - X: ");
  Serial.print(ax);
  Serial.print("  Y: ");
  Serial.print(ay);
  Serial.print("  Z: ");
  Serial.println(az);
  delay(2000);
   

  // Check if any of the acceleration values are above the threshold
  if (abs(ax) > threshold || abs(ay) > threshold || abs(az) > threshold) {
    // Sound the buzzer
    digitalWrite(buzzerPin, HIGH);
    delay(100);  // Adjust the delay according to your preference
    digitalWrite(buzzerPin, LOW);

    while(sim800L.available()){
    buff = sim800L.readString();
    Serial.println(buff);
  }
  while(Serial.available())  {
    buff = Serial.readString();
    buff.trim();
    if(buff == "s")
      send_sms();
    else if(buff== "c")
      make_call();
    else
      sim800L.println(buff);
  }
  }

  delay(100); // Adjust delay as needed
}
void send_sms(){
  sim800L.print("AT+CMGS=\"+917995930488\"\r");
  waitForResponse();
  
  sim800L.print("Hello from SIM800L");
  sim800L.write(0x1A);
  waitForResponse();
}


void make_call(){
  sim800L.println("ATD+919010992426;");
  waitForResponse();
}

void waitForResponse(){
  delay(1000);
  while(sim800L.available()){
    Serial.println(sim800L.readString());
  }
  sim800L.read();
}
