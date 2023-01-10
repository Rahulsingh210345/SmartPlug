#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <DNSServer.h>
#include "HLW8012.h"
#include "string.h"

#define SERIAL_BAUDRATE                 115200

#define BLUE_LED                       13

char aBuf[100]          = {0};
uint32_t Voltage        = 0;
double Current          = 0.0;
uint32_t Active_Power   = 0;
uint32_t Energy         = 0;

// GPIOs
#define RELAY_PIN                       14
#define SEL_PIN                         12
#define CF1_PIN                         5
#define CF_PIN                          4

// MQTT Broker
char *mqtt_broker = "broker.mqttdashboard.com";
char mqtt_username[25] = "1";
char mqtt_password[25] = "1";
char PublishTopic[25] = "DEV000001/Data";
int Port = 1883;
char PubPayload[100] = {0};
bool RelayFlag = false;
bool PreviousRelayFlag = false;

// Check values every 10 seconds
#define UPDATE_TIME                     5000

// Set SEL_PIN to HIGH to sample current
// This is the case for Itead's Sonoff POW, where a
// the SEL_PIN drives a transistor that pulls down
// the SEL pin in the HLW8012 when closed
#define CURRENT_MODE                    LOW //HIGH

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 2 * 1000000 ) // Real: 2Mohm
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

HLW8012           hlw8012;

// connecting to a WiFi network
WiFiManager wifiManager;

// When using interrupts we have to call the library entry point
// whenever an interrupt is triggered
void ICACHE_RAM_ATTR hlw8012_cf1_interrupt() {
    hlw8012.cf1_interrupt();
}
void ICACHE_RAM_ATTR hlw8012_cf_interrupt() {
    hlw8012.cf_interrupt();
}

WiFiServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length);
void MQTT_connect();

// Library expects an interrupt on both edges
void setInterrupts() {
    attachInterrupt(digitalPinToInterrupt(CF1_PIN), hlw8012_cf1_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CF_PIN), hlw8012_cf_interrupt, CHANGE);
}

void calibrate() {

    // Let some time to register values
    unsigned long timeout = millis();
    while ((millis() - timeout) < 5000) {
        delay(1);
    }

    // Calibrate using a 60W bulb (pure resistive) on a 230V line
    hlw8012.expectedActivePower(60.0);
    hlw8012.expectedVoltage(230.0);
    hlw8012.expectedCurrent(60.0 / 230.0);

    // Show corrected factors
    Serial.print("[HLW] New current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
    Serial.print("[HLW] New voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
    Serial.print("[HLW] New power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());

}

void setup() {

    // Init serial port and clean garbage
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println();
    Serial.println();

    // Close the relay to switch on the load
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    for(uint8_t i=0; i<5; i++) {
      digitalWrite(BLUE_LED, HIGH);
      delay(500);
      digitalWrite(BLUE_LED, LOW);
      delay(500);
    }
    
   digitalWrite(BLUE_LED, HIGH);
   
  // wifiManager.resetSettings();
   wifiManager.autoConnect("DEV000001");
   server.begin();
   Serial.print("Connecting to ");
   delay(1000);
   
   while (WiFi.status() != WL_CONNECTED) {
       delay(500);
       Serial.println("Connecting to WiFi..");
   }
   Serial.println("WiFi connected");
   
    // Initialize HLW8012
    // void begin(unsigned char cf_pin, unsigned char cf1_pin, unsigned char sel_pin, unsigned char currentWhen = HIGH, bool use_interrupts = false, unsigned long pulse_timeout = PULSE_TIMEOUT);
    // * cf_pin, cf1_pin and sel_pin are GPIOs to the HLW8012 IC
    // * currentWhen is the value in sel_pin to select current sampling
    // * set use_interrupts to true to use interrupts to monitor pulse widths
    // * leave pulse_timeout to the default value, recommended when using interrupts
    hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, true);

    // These values are used to calculate current, voltage and power factors as per datasheet formula
    // These are the nominal values for the Sonoff POW resistors:
    // * The CURRENT_RESISTOR is the 1milliOhm copper-manganese resistor in series with the main line
    // * The VOLTAGE_RESISTOR_UPSTREAM are the 5 470kOhm resistors in the voltage divider that feeds the V2P pin in the HLW8012
    // * The VOLTAGE_RESISTOR_DOWNSTREAM is the 1kOhm resistor in the voltage divider that feeds the V2P pin in the HLW8012
    hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);

    // Show default (as per datasheet) multipliers
    Serial.print("[HLW] Default current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
    Serial.print("[HLW] Default voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
    Serial.print("[HLW] Default power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
    Serial.println();

    setInterrupts();
    hlw8012.setVoltageMultiplier((double)36519172.51);

    calibrate();

    client.setServer(mqtt_broker, Port);
    client.setCallback(callback);

    digitalWrite(BLUE_LED, LOW);
}

char buffer[50];

void loop() {

    static unsigned long last = millis();

    // This UPDATE_TIME should be at least twice the interrupt timeout (2 second by default)
    if ((millis() - last) > UPDATE_TIME) {

        last = millis();
        Active_Power = hlw8012.getActivePower();
        Voltage  = hlw8012.getVoltage();
        Current  = hlw8012.getCurrent();

        sprintf(aBuf, "Volt : %u \t Curr : %.3lf \t Power : %u \n", Voltage, Current, Active_Power);
        Serial.print(aBuf);
        sprintf(PubPayload, "{\"V\":\"%u\",\"I\":\"%.3lf\",\"P\":\"%u\",\"RLY\":\"%d\"}", Voltage, Current, Active_Power, RelayFlag);
        client.publish(PublishTopic,PubPayload);
        PreviousRelayFlag = RelayFlag;
    }
    MQTT_connect();
    client.loop();

}

/*****************************************************************/
void MQTT_connect() {
  while (!client.connected()) {
    String client_id = "HomeAutomation";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("MQTT Connected................!");
      client.subscribe("DEV000001/Relay");
      
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

/****************************************************************/
void callback(char *topic, byte *payload, unsigned int length) {
  if(strstr((char *)payload,"RLYON")) {
    digitalWrite(RELAY_PIN, HIGH);
    RelayFlag = true;
      
  }else if(strstr((char *)payload,"RLYOFF")) {
    digitalWrite(RELAY_PIN, LOW);
    RelayFlag = false;
      
  }else{
    for (int i = 0; i < length; i++) {
     Serial.print((char) payload[i]);
   }  
   Serial.println();
  }
}
