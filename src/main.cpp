/*************************************************************************
   PROJECT: Bharat Pi 4G Board Code for data push to HiveMQ Cloud
   AUTHOR: Bharat Pi
 
   FUNC: 4G testing with MQTT call to HiveMQ cloud server.
   
   SIMCARD: 4G sim cards from Airtel/Vodaphone/Jio/BSNL can be used. 
   
   IMPORTANT: Configure the APN accordingly as per your Sim provider
   
   GPS: If you have bought Bharat Pi wtih GPS board then you need to connect 
        the GPS antenna to the UFL connector and antenna should have clear sky visibility
        preferrably on the terrace or open field.

   TODO: (Before you upload the code to Bharat Pi board) 
   1) Change APN as per your Sim provider
   2) Change the Thingspeak API key as per your account/setup
   3) Use a power adapter 9V 2amps as the 4G module requires enough power to operate
   
   COPYRIGHT: BharatPi @MIT license for usage on Bharat Pi boards
 *************************************************************************/

#define TINY_GSM_MODEM_SIM7600 //TINY_GSM_MODEM compatible for 7672 as well
#define TINY_GSM_RX_BUFFER 1024


#define SerialAT Serial1
#define SerialMon Serial

#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      17
#define PIN_RX      16
#define PWR_PIN     32

#define LED_PIN 2

#include <TinyGsmClient.h> //Lib version - 0.12.0
#include "SSLClient.h" //Lib version - 2.1.7
#include <PubSubClient.h> //Lib version - 2.8

//CA Certificate for HiveMQ Private Cluster
const char root_ca[] PROGMEM =
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----";


// Pin definitions
const int OVERHEAD_TANK_PIN = 19;  
const int UNDERGROUND_TANK_PIN = 18;  
const int Pump = 26;

// ISR variables 

volatile bool buttonPressed_OHT = false;
volatile unsigned long lastInterruptTime_OHT = 0;

volatile bool buttonPressed_UGT = false;
volatile unsigned long lastInterruptTime_UGT = 0;

const unsigned long debounceTime = 200; // 200 ms debounce time

volatile bool OHT_State = false;
volatile bool UGT_State = false;

/*********************************************
  SECTION: Set APN based on your sim card
    AIRTEL: "airtelgprs.com" 
    BSNL: "bsnlnet / bsnllive" 
    VI: "portalnmms"
    JIO: "jionet"
*********************************************/
const char apn[]  = "bsnlnet"; //Change this as per your Sim card operator

/*********************************************
  SECTION: MQTT Broker Credentials
*********************************************/
const char* mqtt_server = "8e3ddd6ba80a4e3e99739281bebb36d8.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;  
const char* mqtt_username = "Sandhep";
const char* mqtt_password = "Sandhep13";
const char* clientID = "ESP32";
const char* lwt_topic = "Pumphouse/Status";
const char* lwt_message = "Offline";
const int qos = 1;
const bool lwt_retain = true;

TinyGsm modem(SerialAT);

TinyGsmClient gsm(modem);
SSLClient secure_client(&gsm);
PubSubClient  client(secure_client);


/*********************************************
  SECTION: Modem operation functions
*********************************************/
void modemPowerOn(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff(){
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500);
  digitalWrite(PWR_PIN, HIGH);
}

void modemRestart(){
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Define your phone numbers array
String phoneNumbers[] = {"9384584369","9944220260","9789751068","9791265763"};

void sendSMS(String message) {

  // Send SMS to each number in the array

  for (int i = 0; i < sizeof(phoneNumbers) / sizeof(phoneNumbers[0]); i++) {
    Serial.print("Sending SMS to ");
    Serial.println(phoneNumbers[i]);

    // Send SMS
    modem.sendSMS(phoneNumbers[i], message);

    delay(500); // Delay between SMS sends
  }
  Serial.println();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message = "";
  for (int i = 0; i < length; i++) {
     message += (char)payload[i];
  }
  Serial.println(message);

  // Control PUMP based on MQTT message
  if (strcmp(topic, "Pumphouse/Switch/Pump_State") == 0) {
    if (message == "ON") {
      digitalWrite(Pump,LOW);
      sendSMS("Pump is turned ON");
    } else {
      digitalWrite(Pump,HIGH);
      sendSMS("Pump is turned OFF");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientID, mqtt_username, mqtt_password, lwt_topic, qos, lwt_retain, lwt_message)) {
      Serial.println("connected");
      client.publish("Pumphouse/Status", "Online");
      client.subscribe("Pumphouse/Switch/Pump_State");

      OHT_State = digitalRead(OVERHEAD_TANK_PIN);
      client.publish("Pumphouse/Sensor/OHT_Float", OHT_State ? "OFF":"ON");
      if(OHT_State){
             sendSMS("Overhead Tank Float is OFF");
      }else{
             sendSMS("Overhead Tank Float is ON");
      }

      UGT_State = digitalRead(UNDERGROUND_TANK_PIN);
      client.publish("Pumphouse/Sensor/UGT_Float", UGT_State ? "OFF":"ON");
      if(UGT_State){
             sendSMS("Underground Tank Float is OFF");
      }else{
             sendSMS("Underground Tank Float is ON");
      } 
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// ISR for Float Switches

void IRAM_ATTR OHT_ISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime_OHT > debounceTime) {
    buttonPressed_OHT = true;
    lastInterruptTime_OHT = currentTime;
  }
}


void IRAM_ATTR UGT_ISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime_UGT > debounceTime) {
    buttonPressed_UGT = true;
    lastInterruptTime_UGT = currentTime;
  }
}

/*********************************************
  SECTION: Main setup
*********************************************/
void setup(){
  // Set console baud rate
  SerialMon.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(OVERHEAD_TANK_PIN, INPUT_PULLUP);  // Enable pull-up for overhead tank
  pinMode(UNDERGROUND_TANK_PIN, INPUT_PULLUP); // Enable pull-up for underground tank
  attachInterrupt(digitalPinToInterrupt(OVERHEAD_TANK_PIN), OHT_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(UNDERGROUND_TANK_PIN), UGT_ISR, CHANGE);

  pinMode(Pump,OUTPUT);
  digitalWrite(Pump,HIGH);

  delay(100);

  modemPowerOn();
 
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  //secure_layer.setCACert(root_ca);
  secure_client.setCACert(root_ca);

  Serial.clearWriteError();
  Serial.println();
  Serial.println();
  Serial.println("/*******************************************************************************/");
  Serial.println("  Bharat Pi HiveMQ Cloud Sync using MQTT over 4G/LTE ");
  Serial.println("");
  Serial.println("  IMPORTANT: To initialize/latch the 4G/LTE network, please make sure the antenna has been");
  Serial.println("  connected, SIM is inserted in the SIM slot (back side of the board) and 9V 2A power adapter is connected.");
  Serial.println("/******************************************************************************/\n\n");

  delay(2000);

  String res;
  Serial.println("Initializing Modem...");

  if (!modem.init()) {
    digitalWrite(LED_PIN, HIGH);
    modemRestart();
    delay(2000);
    Serial.println("Failed to restart modem, continue without restarting");
    digitalWrite(LED_PIN, LOW);
    return;
  }

  //Blue LED on the board use as an indicator
  //If blinking: Modem not able to boot
  //If turned ON: connected to network
  //If turned OFF: Modem booted successfully but not connected to network, check your SIM, network coverage etc.

  digitalWrite(LED_PIN, LOW); 

  Serial.println("Running SIMCOMATI command...");
  modem.sendAT("+SIMCOMATI"); //Get the module information
  modem.waitResponse(1000L, res);
  //res.replace(GSM_NL "OK" GSM_NL, "");
  Serial.println(res);
  res = "";
  Serial.println();

  Serial.println("Preferred mode selection (GSM/LTE)...");
  modem.sendAT("+CNMP?");
  if (modem.waitResponse(1000L, res) == 1) {
    //res.replace(GSM_NL "OK" GSM_NL, "");
    Serial.println(res);
  }
  res = "";
  Serial.println();

  //Get module manufacturer details
  String modemName = modem.getModemModel();
  Serial.println("Modem Name : " + modemName);
  delay(1000);

  String modemInfo = modem.getModemInfo();
  Serial.println("Modem Info : " + modemInfo);
  delay(1000);

  /*********************************************
    SECTION: Connect to Sim network 
  *********************************************/
  Serial.println("Network mode connectivity testing (GSM, LTE or GSM/LTE)...");

    uint8_t network= 38;
    modem.setNetworkMode(network);
    delay(3000);
    bool isConnected = false;
    int tryCount = 60;
    while (tryCount--) {
      String netoworkOerator = modem.getOperator();
      Serial.print("Operator: ");
      Serial.println(netoworkOerator);
      int16_t signal =  modem.getSignalQuality();
      Serial.print("Signal: ");
      Serial.println(signal);
      Serial.print("isNetworkConnected: ");
      isConnected = modem.isNetworkConnected();
      Serial.println( isConnected ? "CONNECTED" : "NOT CONNECTED YET");
      if (isConnected) {
        break;
      }
      delay(1000);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

  digitalWrite(LED_PIN, HIGH); //Modem connected to network

  Serial.println();
  Serial.println("Yehhh....Device is connected to Sim network.");
  Serial.println();

  delay(1000);
  Serial.println("Checking UE (User Equipment) system information...");
  Serial.println();
  modem.sendAT("+CPSI?");
  if (modem.waitResponse(1000L, res) == 1) {
    res.replace(AT_NL "OK" AT_NL, "");
    Serial.println(res);
  }

  delay(1000);  
  Serial.println("");
  Serial.println("");

  if(modem.isNetworkConnected()){
    Serial.println("Mobile Network is connected.......");
  }  
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("DATA Connection: Connecting to APN: "));
  SerialMon.print(apn);
  if(!modem.gprsConnect(apn, "", "")) {
    Serial.println(" APN connect failed");
    delay(10000);
    return;
  }
  Serial.println(" APN connect success");

  if (modem.isGprsConnected()) { 
    Serial.println("");
    Serial.println("GPRS network is connected");
  }
  Serial.println("");

  /******************************************************************
  SECTION: Set variables and values for pushing data to cloud via MQTT.
           If you want to dynamically capture data from 
           sensors or GPIOs then move this section to Loop 
           and set variables accordingly to push data to HiveMQ Cloud.
  *******************************************************************/
  delay(5000);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
}

/******************************************************************
SECTION: Loop call for capturing dynamic data from sensors/GPIOs
*******************************************************************/  

void loop(){
 if (!client.connected()) {
    reconnect();
  }
  client.loop();

   if (buttonPressed_OHT) {
    Serial.println("OHT Float Position Changed!");
    delay(500);
    OHT_State = digitalRead(OVERHEAD_TANK_PIN);
    client.publish("Pumphouse/Sensor/OHT_Float", OHT_State ? "OFF":"ON");
    if(OHT_State){
       sendSMS("Overhead Tank Float is OFF");
    }else{
       sendSMS("Overhead Tank Float is ON");
    }
    buttonPressed_OHT = false;
  }

  if (buttonPressed_UGT) {
    Serial.println("UGT Float Position Changed!");
    delay(500);
    UGT_State = digitalRead(UNDERGROUND_TANK_PIN);
    client.publish("Pumphouse/Sensor/UGT_Float", UGT_State ? "OFF":"ON");
    if(UGT_State){
      sendSMS("Underground Tank Float is OFF");
    }else{
      sendSMS("Underground Tank Float is ON");
    }
    buttonPressed_UGT = false;
  }

}