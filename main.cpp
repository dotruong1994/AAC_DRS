/*
 *   $Id: local_config.h,v 1.5 2019/04/04 23:06:53 gaijin Exp $
 *
 * Change settings below to customize for -YOUR- local network.
 * 
 */


/*
 * W5500 "hardware" MAC address.
 */
			// Number of seconds to sleep between queries to the time
						// server. Please don't set this any lower than 10 unless
						// timeServer[] is a local NTP server on -your- network.


#include <WiFi.h>
#include <SPI.h>
#include <Ethernet.h>
#include "ModbusClientTCP.h"
#include <AsyncTCP.h>
EthernetClient theClient;
ModbusClientTCP MB(theClient);
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include "time.h"
#define WIFI_TIMEOUT_MS 10000 // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 10000 // Wait 30 seconds after a failed connection attempt
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
const char* ntpServer = "172.21.129.8";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 0;
char timeHour[11];
char timeHour2[9];
const char* ssid = "Factory-K1-2F-3"; 
//const char* ssid = "AE_CAM";
//const char* ssid = "factory-A6-1F-2";
// const char* ssid = "factory-A17-1F-2"; 
const char* password = "qwerasdf";
//const char* password = "Aa123456";

const char* mqtt_server = "172.21.149.109";
uint8_t eth_MAC[] = { 0x08, 0x00, 0x27, 0x36, 0x5A, 0x7B };


/*
 * Define the static network settings for this gateway's ETHERNET connection
 * on your LAN.  These values must match YOUR SPECIFIC LAN.  The "eth_IP"
 * is the IP address for this gateway's ETHERNET port.
 */
IPAddress eth_IP(192, 168, 1, 116);		// *** CHANGE THIS to something relevant for YOUR LAN. ***
IPAddress eth_MASK(255, 255, 255, 0);		// Subnet mask.
IPAddress eth_DNS(0,0,0,0);		// *** CHANGE THIS to match YOUR DNS server.           ***
IPAddress eth_GW(192, 168, 1, 1);		// *** CHANGE THIS to match YOUR Gateway (router).     ***


#define RESET_P	26				// Tie the Wiz820io/W5500 reset pin to ESP32 GPIO26 pin.

//const uint16_t localPort = 55432;		// Local port for UDP packets.

/*
 * Choose the NTP server pool for your geographical region for best
 * performance (fewer lost packets).
 *
 * *** Uncomment only one of the following "timeServer[]" defines. ***
 */
//const char timeServer[] = "time.pouchen.com";		// Default NTP server pool.
const char timeServer[] = "africa.pool.ntp.org";		// Regional server pool.
// const char timeServer[] = "asia.pool.ntp.org";		// Regional server pool.
// const char timeServer[] = "europe.pool.ntp.org";		// Regional server pool.
// const char timeServer[] = "north-america.pool.ntp.org";	// Regional server pool.
// const char timeServer[] = "oceania.pool.ntp.org";		// Regional server pool.
// const char timeServer[] = "south-america.pool.ntp.org";	// Regional server pool.
// const char timeServer[] = "time.nist.gov";			// Original example target server (least preferred).

const uint8_t SLEEP_SECS = 15;
#define MQTT_PORT 1883
#define MQTT_SUB_Output "AAC"
const int reset_w5500 = 32;
int bien_tam = 0;
unsigned long reset_delay = 0;

int dem=0;
int ledState = LOW;

//const uint8_t testdata;

String timeHour1;
String timeHour3;

const uint8_t SLOTS(120);
uint16_t myArray[SLOTS]; 

byte quantity_1;
byte quantity_2;

byte STD_CT_1;
byte STD_CT_2;
bool kt =0;
byte QTYofTODAY_1;
byte QTYofTODAY_2;

byte Machine_STATUS_1;
byte Machine_STATUS_2;

byte PowerOnTD_1;
byte PowerOnTD_2;

byte PowerOffTD_1;
byte PowerOffTD_2;

byte OPtimetoday_1;
byte OPtimetoday_2;

byte IdleTimetoday_1;
byte IdleTimetoday_2;



String sendContent;
// String test[]="";

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 2000;        // Interval at which to publish sensor readings

long lastMsg = 0;
uint16_t addr, words;
/*
   Wiz W5500 reset function.  Change this for the specific reset
   sequence required for your particular board or module.
*/

void WizReset() {
  Serial.print("Resetting Wiz W5500 Ethernet Board...  ");
  pinMode(RESET_P, OUTPUT);
  digitalWrite(RESET_P, HIGH);
  delay(250);
  digitalWrite(RESET_P, LOW);
  delay(50);
  digitalWrite(RESET_P, HIGH);
  delay(350);
  Serial.println("Done.");
}


/*
   This is a crock. It's here in an effort
   to help people debug hardware problems with
   their W5100 ~ W5500 board setups.  It's
   a copy of the Ethernet library enums and
   should, at the very least, be regenerated
   from Ethernet.h automatically before the
   compile starts (that's a TODO item).

*/
/*
   Print the result of the hardware status enum
   as a string.
   Ethernet.h currently contains these values:-

    enum EthernetHardwareStatus {
   	EthernetNoHardware,
   	EthernetW5100,
   	EthernetW5200,
   	EthernetW5500
    };

*/
void prt_hwval(uint8_t refval) {
  switch (refval) {
    case 0:
      Serial.println("No hardware detected.");
      break;
    case 1:
      Serial.println("WizNet W5100 detected.");
      break;
    case 2:
      Serial.println("WizNet W5200 detected.");
      break;
    case 3:
      Serial.println("WizNet W5500 detected.");
      break;
    default:
      Serial.println
      ("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
  }
}


/*
   Print the result of the ethernet connection
   status enum as a string.
   Ethernet.h currently contains these values:-

    enum EthernetLinkStatus {
       Unknown,
       LinkON,
       LinkOFF
    };

*/
void prt_ethval(uint8_t refval) {
  switch (refval) {
    case 0:
      Serial.println("Uknown status.");
      break;
    case 1:
      Serial.println("Link flagged as UP.");
      break;
    case 2:
      Serial.println("Link flagged as DOWN. Check cable connection.");
      break;
    default:
      Serial.println
      ("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
  }
}
// MB_addREQUEST
void get_Value_42977(){
  Error err = MB.addRequest((uint32_t)millis(), 1, READ_HOLD_REGISTER, 42997, 100);
  if (err != SUCCESS) {
    ModbusError e(err);
    Serial.printf("Error creating request: %02X - %s\n", (int)e, (const char *)e);
  }
}

void get_Value_43098(){
  Error err = MB.addRequest((uint32_t)millis(), 1, READ_HOLD_REGISTER, 43098, 100);
  if (err != SUCCESS) {
    ModbusError e(err);
    Serial.printf("Error creating request: %02X - %s\n", (int)e, (const char *)e);
  }
}
  // MB_addREQUEST

  // MQTT SEND
  void sendSample() {
  dem++;
  StaticJsonDocument<128> doc;
  doc.clear();
  doc["Machine name"] = "AAC_01";
  doc["Product name"] = "ABCXYZ";
  doc["Output"] = sendContent;
  doc["Date"] = timeHour1;
  doc["Time"] = timeHour3;
  char buffersend[128];

  //  serializeJson(doc, Serial);
  serializeJson(doc, buffersend);


  uint16_t packetIdPub1 = mqttClient.publish("Nothing", 1, true, buffersend);
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", "Nothing", packetIdPub1);
  //  Serial.printf("Message: %.2f \n", content);
}

//void sendNew() {
//  StaticJsonDocument<400> doc;
//
//  doc["Machine_name"] = "UCC_01";
//  doc["Product_name"] = "1351824120";
//  doc["Date"] = timeHour1;
//  doc["Time"] = timeHour3;
//
//  JsonObject To_Mqtt = doc.createNestedObject("To_Mqtt");
//
//  JsonArray To_Mqtt_Power_on = To_Mqtt.createNestedArray("Power_on");
//  To_Mqtt_Power_on.add(PowerOnTD_1);
//  To_Mqtt_Power_on.add(PowerOnTD_2);
//
//  JsonArray To_Mqtt_Power_off = To_Mqtt.createNestedArray("Power_off");
//  To_Mqtt_Power_off.add(PowerOffTD_1);
//  To_Mqtt_Power_off.add(PowerOffTD_2);
//
//  JsonArray To_Mqtt_Operation_time = To_Mqtt.createNestedArray("Operation_time");
//  To_Mqtt_Operation_time.add(OPtimetoday_1);
//  To_Mqtt_Operation_time.add(OPtimetoday_2);
//
//  JsonArray To_Mqtt_Idle_time = To_Mqtt.createNestedArray("Idle_time");
//  To_Mqtt_Idle_time.add(IdleTimetoday_1);
//  To_Mqtt_Idle_time.add(IdleTimetoday_2);
//
//  JsonArray To_Mqtt_Cycle_time = To_Mqtt.createNestedArray("Cycle_time");
//  To_Mqtt_Cycle_time.add(STD_CT_1);
//  To_Mqtt_Cycle_time.add(STD_CT_2);
//
//  JsonArray To_Mqtt_Output = To_Mqtt.createNestedArray("Output");
//  To_Mqtt_Output.add(response[3]);
//  To_Mqtt_Output.add(quantity_2);
//
//  char buffersendnew[400];
//  serializeJson(doc, buffersendnew);
//
//  //  serializeJson(doc, Serial);
//
//
//  uint16_t packetIdPub1 = mqttClient.publish("UCC-01", 1, true, buffersendnew);
//  //  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", "Nothing", packetIdPub1);
//}

void sendArray() {
  StaticJsonDocument<2048> doc;

  doc["Machine_name"] = "AAC_05";

  JsonArray To_Mqtt_Modbus = doc["To_Mqtt"].createNestedArray("Modbus1");
  for (int i = 0; i < 100; i++) {
    To_Mqtt_Modbus.add(myArray[i]);
  }
  char buffersendArray[2048];
  serializeJson(doc, buffersendArray);
  uint16_t packetIdPub1 = mqttClient.publish("AAC_01", 1, true, buffersendArray);
}


void sendArray2() {
  StaticJsonDocument<2048> doc;

  doc["Machine_name"] = "AAC_06";

  JsonArray To_Mqtt_Modbus = doc["To_Mqtt"].createNestedArray("Modbus2");
  for (int i = 0; i <120; i++) {
    To_Mqtt_Modbus.add(myArray[i]);
  }
  char buffersendArray[2048];
  serializeJson(doc, buffersendArray);
  uint16_t packetIdPub1 = mqttClient.publish("AAC_02", 1, true, buffersendArray);
}
  // MQTT SEND

  // WIFI FUNCTION
  void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  uint16_t packetIdSub = mqttClient.subscribe(MQTT_SUB_Output, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}
void onMqttPublish(uint16_t packetId) {
  Serial.println("");
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("\n Publish received.");
  Serial.print("topic: ");
  Serial.println(topic);
  String messageTemp;
  for (int i = 0; i < len; i++) {
    messageTemp += (char)payload[i];
  }


  if (messageTemp == "esp_reset") {
bien_tam = 1;
}}


void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  strftime(timeHour,11, "%F", &timeinfo);
  strftime(timeHour2,9, "%T", &timeinfo);
  timeHour1 = String(timeHour);
  timeHour3 = String(timeHour2);
  String timeresetShift1 = "07:00:00"; //06:00:00
//  String timeresetShift2 = "14:00:00"; //14:00:00
//  String timeresetShift3 = "22:00:00";
  if (timeHour3 == timeresetShift1){Serial.println("ESP reset chứ còn cái nịt gì nữa");ESP.restart();}
//  if (timeHour3 == timeresetShift2){Serial.println("ESP reset chứ còn cái nịt gì nữa");ESP.restart();}
//  if (timeHour3 == timeresetShift3){Serial.println("ESP reset chứ còn cái nịt gì nữa");ESP.restart();}
  Serial.println(timeHour3);
}
  // WIFI FUNCTION


  // MODBUS Function
  void handleData(ModbusMessage response, uint32_t token) 
{
  Serial.printf("Response: serverID=%d, FC=%d, Token=%08X, length=%d:\n", response.getServerID(), response.getFunctionCode(), token, response.size());
  for (auto& byte : response) {
    Serial.printf("%02X ", byte);
  }
//    byte tam[response.size()] = response;
  uint16_t position = 3;  // first data right after the length byte 
  for (uint8_t arrayIndex = 0; arrayIndex < SLOTS; arrayIndex++) { 
    position = response.get(position, myArray[arrayIndex]); // get next value, advancing the byte position
  }

  
 
}

// Define an onError handler function to receive error responses
// Arguments are the error code returned and a user-supplied token to identify the causing request
void handleError(Error error, uint32_t token) 
{
  // ModbusError wraps the error code and provides a readable error message for it
  ModbusError me(error);
  Serial.printf("Error response: %02X - %s\n", (int)me, (const char *)me);
}

  // MODNUS Function

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(reset_w5500,OUTPUT);
  digitalWrite(reset_w5500, LOW);
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(mqtt_server, MQTT_PORT);
  connectToWifi();

  // Use Ethernet.init(pin) to configure the CS pin.
  Ethernet.init(5);           // GPIO5 on the ESP32.
  WizReset();
  
  /*
     Network configuration - all except the MAC are optional.

     IMPORTANT NOTE - The mass-produced W5500 boards do -not-
                      have a built-in MAC address (depite
                      comments to the contrary elsewhere). You
                      -must- supply a MAC address here.
  */
  Serial.println("Starting ETHERNET connection...");
  Ethernet.begin(eth_MAC, eth_IP, eth_DNS, eth_GW, eth_MASK);

  delay(200);

  Serial.print("Ethernet IP is: ");
  Serial.println(Ethernet.localIP());

  /*
     Sanity checks for W5500 and cable connection.
  */
  Serial.print("Checking connection.");
  bool rdy_flag = false;
  for (uint8_t i = 0; i <= 20; i++) {
    if ((Ethernet.hardwareStatus() == EthernetNoHardware)
        || (Ethernet.linkStatus() == LinkOFF)) {
      Serial.print(".");
      rdy_flag = false;
      delay(80);
    } else {
      rdy_flag = true;
      break;
    }
  }
  if (rdy_flag == false) {
    Serial.println
    ("\n\r\tHardware fault, or cable problem... cannot continue.");
    Serial.print("Hardware Status: ");
    prt_hwval(Ethernet.hardwareStatus());
    Serial.print("   Cable Status: ");
    prt_ethval(Ethernet.linkStatus());
    while (true) {
      delay(10);          // Halt.
      ESP.restart();
    }
  } else {
    Serial.println(Ethernet.localIP());
    Serial.println(" OK");
  }

  // Set up ModbusTCP client.
  // - provide onData handler function
  MB.onDataHandler(&handleData);
  // - provide onError handler function
  MB.onErrorHandler(&handleError);
  // Set message timeout to 2000ms and interval between requests to the same host to 200ms
  MB.setTimeout(2000, 200);
  // Start ModbusTCP background task
  MB.begin();

  // Issue a request
  // Set Modbus TCP server address and port number
  // (Fill in your data here!)
  MB.setTarget(IPAddress(192, 168, 1, 6), 502); //192.168.40.10

  // Create request for
  // (Fill in your data here!)
  // - token to match the response with the request. We take the current millis() value for it.
  // - server ID = 4
  // - function code = 0x03 (read holding register)
  // - start address to read = word 2
  // - number of words to read = 6
  //
  // If something is missing or wrong with the call parameters, we will immediately get an error code
  // and the request will not be issued

  //run 1st time
  /*  
    Error err = MB.addRequest((uint32_t)millis(), 1, READ_INPUT_REGISTER, 1056, 1);
    if (err != SUCCESS) {
      ModbusError e(err);
      Serial.printf("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }
  */   
}

void loop() {
if(bien_tam == 1){
    digitalWrite(reset_w5500,HIGH);
    reset_delay = millis();
    bien_tam = 2;
    }
if(bien_tam == 2){
    if(millis() -reset_delay>1000)
      {
        ESP.restart();  
      }
    }

long now = millis();
  if (now - lastMsg > 3000) 
  {
    if(kt ==0 )
    {
    lastMsg = now;
 //   printLocalTime();  
    Error err = MB.addRequest((uint32_t)millis(), 1, READ_HOLD_REGISTER, 42997, 100);
    if (err != SUCCESS) {
      ModbusError e(err);
      Serial.printf("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }
    sendArray();
    kt =1;
    }
    delay(1000);
    if ( kt ==1){
      lastMsg = now;
   //   printLocalTime();  
      Error err = MB.addRequest((uint32_t)millis(), 1, READ_HOLD_REGISTER, 43098, 100);
      if (err != SUCCESS) {
        ModbusError e(err);
        Serial.printf("Error creating request: %02X - %s\n", (int)e, (const char *)e);
      }
      sendArray2();
      kt =0;
      }
  }
}

