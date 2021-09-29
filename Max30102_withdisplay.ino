#include <Wire.h>
#include "MAX30105.h"  //sparkfun MAX3010X library
#include "heartRate.h" // Library to Calculate Heart Rate
#include "PubSubClient.h"
#include "WiFi.h"
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
#include <Adafruit_MLX90614.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>

//display config
SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_64, I2C_ONE, 100000);

//DS18B20 Config
//#define DS18B20 4
//OneWire oneWire(DS18B20);
//DallasTemperature thermometer(&oneWire);
double temperatureC = 0;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();


MAX30105 particleSensor; //MAX30105 class object

//Configuration
#define USEFIFO
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 80 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 1 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 50000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0

double avered = 0; 
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;//calculate SpO2 by this sampling interval
double ESpO2 = 95.0; //initial value of estimated SpO2
double FSpO2 = 0.7;  //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component



 


//HR Config
const byte RATE_SIZE = 8; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg = 0;

//publisher
int iterations = 0;
int counter = 0;
String spo2_;
String bpm_;
String bodytemp;

// WIFI 

const char* ssid = "IOT Lab";                 // Your personal network SSID
const char* wifi_password = "iotlab_123"; // Your personal network password

// MQTT
const char* mqtt_server = "10.3.25.107";  // IP of the MQTT broker
const char* spO2_topic = "vitals/spo2/";
const char* bpm_topic = "vitals/HR/";
const char* temperature_topic = "vitals/Temperature/";
const char* mqtt_username = "fyp"; // MQTT username
const char* mqtt_password = "iomd"; // MQTT password
const char* clientID = "IOMD_Watch"; // MQTT client ID

//Condition
bool condition = false;

WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient); 


void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);
  
  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }


  // Debugging - Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}




void setup()
{
  Serial.begin(115200);
  mlx.begin();
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
     Serial.println("MAX30102 was not found. Please check wiring.");
    while (1);
  }

  //Setup to sense a nice looking saw tooth on the plotter for SpO2
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384


  //thermometer.begin();
  connect_MQTT();
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor 
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
}

void loop()
{ 

  iterations++;
  if(iterations % 1000 == 0){
    iterations = 0;
    condition = true;
  }

  
  spo2_= String(ESpO2);
  bpm_= String(beatAvg);
  bodytemp = String(temperatureC);

  uint32_t ir, red , green;
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered
  particleSensor.check();
  while (particleSensor.available()) {
    
    ir = particleSensor.getFIFORed(); 
    red = particleSensor.getFIFOIR();
    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate);  //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered);
    sumirrms += (fir - aveir) * (fir - aveir);
    
    if ((i % SAMPLING) == 0) {
      if ( millis() > TIMETOBOOT) {

        if (checkForBeat(red) == true){
          long delta = millis() - lastBeat;
          lastBeat = millis();
          beatsPerMinute = 60 / (delta / 1000.0);
      
        if (beatsPerMinute < 255 && beatsPerMinute > 20){
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable
          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++){beatAvg += rates[x];}
          beatAvg /= RATE_SIZE;
      }
      }
        
        float ir_forGraph = (2.0 * fir - aveir)/aveir * SCALE ;  
        float red_forGraph = (2.0 * fred - avered)/avered * SCALE ; 
        
        if (red < FINGER_ON){ 
          ESpO2 = MINIMUM_SPO2;
          beatAvg = 0; 
          } //indicator for finger detached
        Serial.print(" IR = ");Serial.print(ir_forGraph); // to display pulse wave at the same time with SpO2 data
        Serial.print(" | Red: "); Serial.print(red_forGraph); // to display pulse wave at the same time with SpO2 data
        Serial.print(" | SpO2(%): "); Serial.print(ESpO2); //low pass filtered SpO2
        Serial.print(" | Average HR (bpm): "); Serial.print(beatAvg);
        Serial.print(" | Body Temp (C): "); Serial.println(bodytemp);
        
        if(beatAvg!=0){
          display_oled();
          publisher();
          
          if(beatAvg>40 && int(ESpO2)>90){
          temperature_sensor();}
          }  

           
        }  
    }

    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      SpO2 = -23.3 * (R - 0.4) + 100; //https://ww1.microchip.com/downloads/en/AppNotes/00001525B.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;

    }
    particleSensor.nextSample();
   }
}


void publisher(){
  
 
  if(((int)ESpO2 != 80) && (beatAvg > 40) && (temperatureC != 0)){
    counter = counter + 1;

    if(counter % 50 == 0){
      counter = 0; 
      if (client.publish(spO2_topic, String(spo2_).c_str())) {
        Serial.println("SpO2 sent!");}
      else {
          Serial.println("SpO2 failed to send. Reconnecting to MQTT Broker and trying again");
          client.connect(clientID, mqtt_username, mqtt_password);
          delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
          client.publish(spO2_topic, String(spo2_).c_str());
          }
          
      if (client.publish(bpm_topic, String(bpm_).c_str())) {
          Serial.println("Heart Rate sent!");}
      else {
          Serial.println("Temperature failed to send. Reconnecting to MQTT Broker and trying again");
          client.connect(clientID, mqtt_username, mqtt_password);
          delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
          client.publish(bpm_topic, String(bpm_).c_str());
          }
      if(temperatureC != 0){
        if (client.publish(temperature_topic, bodytemp.c_str())) {
          Serial.println("Temperature sent!");
          }
        else {
          Serial.println("Temperature failed to send. Reconnecting to MQTT Broker and trying again");
          client.connect(clientID, mqtt_username, mqtt_password);
          delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
          client.publish(temperature_topic, bodytemp.c_str());
        }
      }
    }   
   }
  }
  
  
void temperature_sensor(){
    if(condition == true){
      condition = false; 
      temperatureC = mlx.readObjectTempC();
      delay(10);
    }
  }
  
  
void display_oled(){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(9, 0,  "HR   = " +bpm_ +" bpm");
  display.drawString(0, 19, "SpO2 = " +spo2_ +" %" );
  if(temperatureC != 0){display.drawString(0, 40, "Temp = " +bodytemp +" C");}
  // write the buffer to the display
  display.display();
  delay(10); 
  }
  
  
 
  
