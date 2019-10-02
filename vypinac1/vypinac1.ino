/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 *
 * Script for double SSR relay board by Aproxx
 * https://www.openhardware.io/view/77/AC-DC-double-solid-state-relay-module
 * https://forum.mysensors.org/topic/3671/ac-dc-double-solid-state-relay-module
 * Control 2 circuits either from controller or from physical buttons connected on pins 4 & 7
 * Optional DS18b20 is connected on pin 8
 * 
 *  HISTORY :
 * xx/xx/2016 original version by Aproxx
 * 08/02/2016 upgraded to MySensors 2.0 by mr_const
 * 08/30/2016 changes by Nca78 :
 *        - fixed initialization of physical buttons/debouncer status
 *        - centralized pin status change for relays in setRelayState method
 *        - centralized debug information for state changes in one method + added debug info when changed by physical switches
 *        - added #ifdef MY_DEBUG before each Serial.print (saves prog memory when not in debug mode) and F() macros for debug strings (saves RAM when in debug mode)
 *        - added #define USE_TEMP_SENSOR to make temperature sensor optional (not used if line is commented)
 *        - put back #define for repeater feature
 *        - add #define TEMPERATURE_ROUNDING for custom temperature rounding
**/

// MySensor Debug
//#define MY_DEBUG

// Enables repeater functionality (relays messages from other nodes)
//#define MY_REPEATER_FEATURE

// Comment line below if you don't want to use the temperature sensor
#define USE_TEMP_SENSOR
#define MY_TRANSPORT_WAIT_READY_MS 3000     //set how long to wait for transport ready.
#define MY_RADIO_RF24
#define MY_NODE_ID 11
#include <MySensors.h>
#include <SPI.h>
#include <Bounce2.h>

#define RELAY_PIN    3  // Arduino Digital I/O pin number for relay 
#define RELAY_PIN_2  5
#define BUTTON_PIN   4  // Arduino Digital I/O pin number for button 
#define BUTTON_PIN_2 7


#define CHILD_ID 8 // Id of the sensor child for 1st relay
#define CHILD_ID_2 9 // Id of the sensor child for 2nd relay

// Relay status
#define RELAY_ON 1
#define RELAY_OFF 0

// Source of state change (used when printing debug information)
#define CHANGE_STATE_SOURCE_RADIO 0
#define CHANGE_STATE_SOURCE_SWITCH 1


// Temperature sensor definitions
#ifdef USE_TEMP_SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 8
#define CHILD_DSB_ID 13 // Id of the sensor child for temperature sensor
#define TEMPERATURE_ROUNDING 2.f   // Change value to change rounding of temperature value: 10.f for 0.1°C change, 5.f for 0.2°C change, 2.f for 0.5°C change
#endif



Bounce debouncer = Bounce();
int oldValue;
bool state;
Bounce debouncer2 = Bounce();
int oldValue2;
bool state2;

MyMessage msg(CHILD_ID, V_LIGHT);
MyMessage msg2(CHILD_ID_2, V_LIGHT);

#ifdef USE_TEMP_SENSOR
MyMessage msgTemp(CHILD_DSB_ID, V_TEMP);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.
#endif

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Double Relay & Button", "0.2");
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID, S_LIGHT);
  present(CHILD_ID_2, S_LIGHT);
#ifdef USE_TEMP_SENSOR
  present(CHILD_DSB_ID, S_TEMP);
#endif
}

void setup()
{
#ifdef USE_TEMP_SENSOR
  sensors.begin();
  sensors.setWaitForConversion(false);
#endif

  // Setup the button
  pinMode(BUTTON_PIN, INPUT);
  // Activate internal pull-up
  digitalWrite(BUTTON_PIN, HIGH);

  // Setup the button
  pinMode(BUTTON_PIN_2, INPUT);
  // Activate internal pull-up
  digitalWrite(BUTTON_PIN_2, HIGH);

  // After setting up the button, setup debouncer
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(5);

  debouncer2.attach(BUTTON_PIN_2);
  debouncer2.interval(5);

  // Set the initial values of oldValue/oldValue2 variables from status of physical switches
  //  if this is not done the loop() will detect status change and switch the relays on or off
  debouncer.update();
  debouncer2.update();
  oldValue = debouncer.read();
  oldValue2 = debouncer2.read();


  // Make sure relays are off when starting up
  setRelayState(RELAY_PIN, RELAY_OFF);
  // Then set relay pins in output mode
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN_2, RELAY_OFF);
  // Then set relay pins in output mode
  pinMode(RELAY_PIN_2, OUTPUT);

  // Set relay to last known state (using eeprom storage)
  state = loadState(CHILD_ID);
  setRelayState(RELAY_PIN, state);

  state2 = loadState(CHILD_ID_2);
  setRelayState(RELAY_PIN_2, state2);
}


/*
   Example on how to asynchronously check for new messages from gw
*/
void loop()
{
#ifdef USE_TEMP_SENSOR
  static float prevTemp = 0;
#endif

  debouncer.update();
  debouncer2.update();
  // Get the update value
  int value = debouncer.read();
  int value2 = debouncer2.read();

  if (value != oldValue) {
    state =  !state;                                                  // Toggle the state
    send(msg.set(state), false);                          // send new state to controller, no ack requested
    setRelayState(RELAY_PIN, state);                // switch the relay to the new state
    saveState(CHILD_ID, state);        // Store state in eeprom
    // Write some debug info
    printStateChangedDebug(CHANGE_STATE_SOURCE_SWITCH, CHILD_ID, value);
    oldValue = value;
  }
  

  if (value2 != oldValue2) {
    state2 =  !state2;                                         // Toggle the state
    send(msg2.set(state2), false);                 // send new state to controller, no ack requested
    setRelayState(RELAY_PIN_2, state2);     // switch the relay to the new state
    saveState(CHILD_ID_2, state2);     // Store state in eeprom
    // Write some debug info
    printStateChangedDebug(CHANGE_STATE_SOURCE_SWITCH, CHILD_ID_2, value2);
    oldValue2 = value2;
  }
  

  // Fetch temperatures from Dallas sensors
#ifdef USE_TEMP_SENSOR
  sensors.requestTemperatures();
  // Fetch and round temperature to one decimal
  float temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(0) * TEMPERATURE_ROUNDING)) / TEMPERATURE_ROUNDING;

  if (temperature != -127.00f && temperature != 85.00f && prevTemp != temperature) {
    // Send in the new temperature
    send(msgTemp.set(temperature, 1));
#ifdef MY_DEBUG
    Serial.print("Sent temperature: ");
    Serial.println(temperature);
#endif
    prevTemp = temperature;
  }
#endif
}

void receive(const MyMessage &message) { 
if (message.type == V_STATUS) {
 switch (message.sensor) {
  case CHILD_ID:    
    state = message.getBool();          // Change relay state
    setRelayState(RELAY_PIN, state);    
    saveState(CHILD_ID, state);        // Store state in eeprom
    // Write some debug info
    printStateChangedDebug(CHANGE_STATE_SOURCE_RADIO, CHILD_ID, state);
    break; 
  case CHILD_ID_2:
    state2 = message.getBool();
    setRelayState(RELAY_PIN_2, state2);
    saveState(CHILD_ID_2, state2);     // Store state in eeprom
    // Write some debug info
    printStateChangedDebug(CHANGE_STATE_SOURCE_RADIO, CHILD_ID_2, state2);
    break;
  }
 }
}



// Set status of a relay pin
void setRelayState(byte relayPin, bool value) {
  digitalWrite(relayPin, value ? RELAY_ON : RELAY_OFF);
}

// Print debug info, centralized in one place to minimize memory usage and have only one #ifdef MY_DEBUG for all state change messages
void printStateChangedDebug(int source, int sensorID, bool value) {
#ifdef MY_DEBUG
  Serial.print(F("Sensor value changed, source="));
  Serial.print(source == CHANGE_STATE_SOURCE_RADIO ? F("Radio") : F("Physical switch"));
  Serial.print(F(", Sensor="));
  Serial.print(sensorID);
  Serial.print(F(", New status: "));
  Serial.println(value);
#endif
}
