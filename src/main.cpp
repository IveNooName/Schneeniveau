/******************************************************************************************************
 *  Dieses Programm ist für die UEberwachung der Heizung. Angeschlossen sind ein Ultraschall Distanz-
 *  sensor um den Füllstand des Oeltanks zu messen und 4 DS1820 Temperatursensoren. Diese messen
 *  die Temperatur im Speichertank sowie Vor- und Rücklauftemperatur.
 *  Die Messwerte werden via MQTT versendet.
 * 
 * 
 * 
********************************************************************************************************/

#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HardwareSerial.h>
#include <TimeLib.h>
#include <PubSubClient.h>
#include "config.h"
#include "time.h"


// Dallas 
// Data wire is plugged into port 22 on the ESP32
#define ONE_WIRE_BUS 22
#define TEMPERATURE_PRECISION 12
#define max_DS1820 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Add 4 prepared sensors to the bus
// use the UserDataWriteBatch demo to prepare 4 different labeled sensors
struct {
  int id;
  DeviceAddress addr;
} T[max_DS1820];

float temperatureC[max_DS1820];
int DS1820_count=0;


// Ultrasonic Sensor A01NYUB / DFRobot SEN0313
#define timeout 5  //timeout in seconds when reading distance or so
#define A01NYUB_VCC_Pin 17  // Pin sof SEN Sensor
int64_t jetzt;
int distance; 

//SoftwareSerial
#define RX 12   //  RX
#define TX 13   // TX
#define BAUD_RATE 9600

HardwareSerial mySerial(1);

int temperatureTX_interval = 30;   //transmit interval in seconds for temperature
int distanceTX_interval = 10;      //transmit interval in seconds for Fuel level

int64_t last_distanceTX_timestamp = 10 - distanceTX_interval;  // We want the first measurement to be transimitted 10 seconds after boot time
int64_t last_temperatureTX_timestamp = 10 - temperatureTX_interval ;

unsigned char data[4]={};    //we read always 4 byte from the serial port 

//configure OLED display
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);


//ESP MQTT
//boolean connect (clientID, [username, password], [willTopic, willQoS, willRetain, willMessage], [cleanSession])
WiFiClient espClient;
PubSubClient mqttclient;
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
char topic[MSG_BUFFER_SIZE];
int value = 0;

//Function definition
void printAddress(DeviceAddress deviceAddress);
char stringAddress(DeviceAddress deviceAddress);
void printTemperature(DeviceAddress deviceAddress);
int Read_A01NYUB() ;
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void  printLocalTime();

// Time
struct tm timeinfo;


// ============================================================================
// Setup function
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  u8g2.begin();
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_t0_17b_mf); // choose a suitable font
  u8g2.drawStr(10,12,"Tankmonitor");  // write something to the internal memory
  u8g2.drawHLine(20, 16, 100);
  u8g2.sendBuffer();          // transfer internal memory to the display

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, WiFiPassword );
  // Wi-Fi
  Serial.println("Connecting to WiFi..");

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Connected to the WiFi network");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  u8g2.setFont(u8g2_font_t0_11_mf); // choose a suitable font
  u8g2.setCursor(0, 62);
  //u8g2.setCursor(128-u8g2.getUTF8Width(WiFi.localIP().toString()), 62);
  u8g2.print(WiFi.localIP());
  u8g2.sendBuffer();          // transfer internal memory to the display

  //Initialise DS1820
  sensors.begin();

  // locate devices on the bus
  DS1820_count = sensors.getDeviceCount();
  Serial.println("Found " + String(DS1820_count) + " devices.");
  
  if ((DS1820_count > 0)) {
    
    // Read ID's per sensor and put them in T array
    for(byte index=0 ;index < DS1820_count; index++) {
      sensors.getAddress(T[index].addr, index);
      Serial.print("\t\tIndex: ");
      Serial.print(index);
      Serial.print("\t\tID: ");
      T[index].id = sensors.getUserData(T[index].addr);
      Serial.print(T[index].id);
      Serial.print("\t\tAddr: ");
      printAddress(T[index].addr);
      Serial.println();
      sensors.setResolution(T[index].addr, TEMPERATURE_PRECISION);
    }
  }
 
   //mySerial.begin(BAUD_RATE, SWSERIAL_8N1, D5, D6, false, 95, 11);
   mySerial.begin(9600, SERIAL_8N1, 12, 13);

  //VCC Pin for Sensor, sensor consumes 15mA, port is rated for 20ma
  pinMode(A01NYUB_VCC_Pin, OUTPUT);
  digitalWrite(A01NYUB_VCC_Pin, 0);

//  mqttclient.setClient(espClient);
//  mqttclient.setServer(mqtt_server, 1883);
//  mqttclient.setCallback(callback);

  //init Time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

}


// ============================================================================
// Main Program
// ============================================================================
void loop() {
  // printLocalTime();
  // put your main code here, to run repeatedly:
//  if (!mqttclient.connected()) {
//    reconnect();
//  }
//  mqttclient.loop();


  sleep(3);
  Serial.println ("Foo0");
  if ((last_distanceTX_timestamp+distanceTX_interval) < now() ) {

  Serial.println ("Foo1");
  distance = Read_A01NYUB();
  Serial.println ("Foo2");
  u8g2.setFont(u8g2_font_t0_17b_mf); // choose a suitable font
  u8g2.setCursor(0, 34);
  u8g2.print("Fuel: ");
  u8g2.setCursor(60, 34);
  
  if (distance) {
    Serial.println ("Distance measured: " + String(distance));
    u8g2.print(distance);
    u8g2.print("mm");
    last_distanceTX_timestamp = now();
    
//    snprintf (msg, MSG_BUFFER_SIZE, "%ld", distance);
// mqttclient.publish("heizung/tank/level", msg);


  } else {
    Serial.println ("ERROR measuring distance");
    u8g2.print("N/A");
  }
  
  u8g2.sendBuffer();          // transfer internal memory to the display
  }
  if ((last_temperatureTX_timestamp+temperatureTX_interval) < now() ) {
    Serial.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    Serial.println("DONE");
    delay(100);
  

    Serial.println ("Foo3");
    if ((DS1820_count > 0)) {
    
      // Read ID's per sensor and put them in T array
      for(byte index=0 ;index < DS1820_count; index++) {
  
        temperatureC[index] = sensors.getTempC(T[index].addr);
        Serial.print(" Temp: ");
        Serial.print(temperatureC[index]);
        Serial.print(" °C");
        Serial.print("\t\tIndex: ");
        Serial.print(index);
        Serial.print("\t\tID: ");
        Serial.print(T[index].id);
        Serial.print("\t\tAddr: ");
        printAddress(T[index].addr);
        Serial.println("");

        // snprintf (msg, MSG_BUFFER_SIZE, "%.1f", temperatureC[index]);
        // snprintf (topic, MSG_BUFFER_SIZE, "heizung/DS1820/%d", T[index].id);
        // //topic = "heizung/DS1820/" + stringAddress(T[index].addr);
        // mqttclient.publish(topic, msg);

      }
    }
    Serial.println ("Foo4");
    last_temperatureTX_timestamp = now();

    Serial.println ("Foo5"); 
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    } else {
      u8g2.setFont(u8g2_font_t0_11_mf); // choose a suitable font
      u8g2.setCursor(0, 52);
      u8g2.print(&timeinfo, "%d %b %Y %H:%M");

      u8g2.sendBuffer(); 
    }
    Serial.println ("Foo6");
  }
}

// ---------------------------------------------------------------------
// function to read the distance value ffrom A01NYUB
int Read_A01NYUB() {
  Serial.print("Requesting distance:  ");
  jetzt = esp_timer_get_time() ;
  digitalWrite(A01NYUB_VCC_Pin, 1);
  // wait until we get valid data from the sensor. Value found by experimenting
  sleep (2);
  // clear serial input buffer as it may contain garbage or old data
   while(mySerial.available() > 0) {
    char t = mySerial.read();
   }
  // Read data from serial port. Sensor transmits 0xFF as padding, so read until something else or a timeout occurs
  // See https://wiki.dfrobot.com/A01NYUB%20Waterproof%20Ultrasonic%20Sensor%20SKU:%20SEN0313
  do{
    data[1]=mySerial.read();
    if (esp_timer_get_time()-jetzt > timeout*1000000 ) {
      Serial.println("Sensor Timeout!");
      digitalWrite(A01NYUB_VCC_Pin, 0);
      return(0);
    }
  } while(data[1]==0xff);

  /* Deguing serial port
  while (1) {
     if (mySerial.available() > 0) {
       data[1]=mySerial.read();
       if (data[1]!=0xff) {
        Serial.print (String(data[1]) + " ");
      } else {
        Serial.print(".");
      }
    }
  }
*/

  //data[0] is 0xff in the formula below, read the next two bytes
  data[0]=0xff;
  while(mySerial.available() == 0) {
    usleep(10);
  }
   data[2]=mySerial.read();
   
   while(mySerial.available() == 0) {
  usleep(10);
   }
   data[3]=mySerial.read();
   
  int sum;
  Serial.print ("Header= " + String(data[0]) + "; High= " + String(data[1]) + "; Low = "+ String(data[2]) + "; Checksum = "+ String(data[3]) + " ");
  sum=(data[0]+data[1]+data[2])&0x00FF;
  if(sum==data[3])
  {
    distance=((data[1]<<8)+data[2]);
      Serial.print("Distance= " + String(distance)+"mm  ");
    if(distance>260) {
        digitalWrite(A01NYUB_VCC_Pin, 0);
        return distance;
    } else {
      Serial.println("Below the lower limit");        
    }
  } else Serial.println("Checksum Error");
  digitalWrite(A01NYUB_VCC_Pin, 0);
  return 0;
}

// ---------------------------------------------------------------------
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
char stringAddress(DeviceAddress deviceAddress)
{
  char text[15] = "";
  for (uint8_t i = 0; i < 8; i++)
  {
 //   if (deviceAddress[i] < 16) text[2*i]=("0");
   // text += String(deviceAddress[i], HEX);
  }
  return *text;
}
// ---------------------------------------------------------------------
// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}
  
// ---------------------------------------------------------------------
// function to handle the message received
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    //digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    //digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}
// ---------------------------------------------------------------------
// function to 
void reconnect() {
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttclient.connect(clientId.c_str(),mqtt_user,mqtt_pass)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttclient.publish("outTopic", "hello world");
      // ... and resubscribe
      mqttclient.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}