
// ##include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <time.h>

#include "defines.h"
#include "utilities.h"
#include "cntrl2.h"

#include <AsyncMqttClient_Generic.hpp>

#define ESP8266_DRD_USE_RTC true
#define ESP_DRD_USE_LITTLEFS false
#define ESP_DRD_USE_SPIFFS false
#define ESP_DRD_USE_EEPROM false
#define DOUBLERESETDETECTOR_DEBUG true
#include <ESP_DoubleResetDetector.h>

//***********************
// Template functions
//***********************
bool onMqttMessageAppExt(char *, char *, const AsyncMqttClientMessageProperties &, const size_t &, const size_t &, const size_t &);
// bool onMqttMessageCntrlExt(char *, char *, const AsyncMqttClientMessageProperties &, const size_t &, const size_t &, const size_t &);
void appMQTTTopicSubscribe();
void telnet_extension_1(char);
void telnet_extension_2(char);
void telnet_extensionHelp(char);
void startTimesReceivedChecker();
void processCntrlTOD_Ext();
// void app_WD_on(void *);
// void app_WE_off(cntrlState &);
// void app_WD_on(cntrlState &);
// void app_WE_off(cntrlState &);
// void app_WD_auto(cntrlState &);
// void app_WE_auto(cntrlState &);

//*************************************
// defined in asyncConnect.cpp
//*************************************
extern void mqttTopicsubscribe(const char *topic, int qos);
extern void platform_setup(bool);
extern void handleTelnet();
extern void printTelnet(String);
extern AsyncMqttClient mqttClient;
extern void wifiSetupConfig(bool);
extern templateServices coreServices;
extern char ntptod[MAX_CFGSTR_LENGTH];

// application specific
void readBoilerTemprature();
void checkValveStatus();
void setBoilerDemand();

//*************************************
// defined in cntrl.cpp
//*************************************
cntrlState USCntrlState; // Upstairs
cntrlState DSCntrlState; // Downstairs
cntrlState HWCntrlState; // Hotwater

#define CmdStateUpstairsWD "/house/cntrl/heating/upstairs/wd-command"			// UI Button press, ON, OFF, NEXT, SET
#define CmdStateUpstairsWE "/house/cntrl/heating/upstairs/we-command"			// UI Button press
#define CommandUpstairsWDTimes "/house/cntrl/heating/upstairs/wd-control-times" // Times message from either UI or Python app
#define CommandUpstairsWETimes "/house/cntrl/heating/upstairs/we-control-times" // Times message from either UI or MySQL via Python app

#define CmdStateDownstairsWD "/house/cntrl/heating/downstairs/wd-command"			// UI Button press
#define CmdStateDownstairsWE "/house/cntrl/heating/downstairs/we-command"			// UI Button press
#define CommandDownstairsWDTimes "/house/cntrl/heating/downstairs/wd-control-times" // Times message from either UI or Python app
#define CommandDownstairsWETimes "/house/cntrl/heating/downstairs/we-control-times" // Times message from either UI or MySQL via Python app

#define CmdStateHotwaterWD "/house/cntrl/heating/hotwater/wd-command"			// UI Button press
#define CmdStateHotwaterWE "/house/cntrl/heating/hotwater/we-command"			// UI Button press
#define CommandHotwaterWDTimes "/house/cntrl/heating/hotwater/wd-control-times" // Times message from either UI or Python app
#define CommandHotwaterWETimes "/house/cntrl/heating/hotwater/we-control-times" // Times message from either UI or MySQL via Python app

#define StateUpstairsRuntime "/house/cntrl/heating/upstairs/runtime-state"	   // e.g. ON, OFF and WAIT
#define StateDownstairsRuntime "/house/cntrl/heating/downstairs/runtime-state" // e.g. ON, OFF and WAIT
#define StateHotwaterRuntime "/house/cntrl/heating/hotwater/runtime-state"	   // e.g. ON, OFF and WAIT

#define StateUpstairsValveState "/house/cntrl/heating/upstairs/valve-state"		// OK, FAULT
#define StateDownstairsValveState "/house/cntrl/heating/downstairs/valve-state" // OK, FAULT
#define StateHotwaterValveState "/house/cntrl/heating/hotwater/valve-state"		// OK, FAULT

#define StateBoilerTemprature "/house/cntrl/heating/boiler/temprature"	 // Temprature sensor
#define StateCylinderDemand "/house/cntrl/heating/water-cylinder/demand" // Ok, KO - Cylinder demand

#define RefreshID "HEATING"

// Double Reset Detection configuration
#define DRD_TIMEOUT 3
#define DRD_ADDRESS 0

DoubleResetDetector *drd;

// defined in telnet.cpp
extern int reporting;
extern bool telnetReporting;

//
// Application specific
//

String deviceName = "heating";
String deviceType = "CNTRL";
String app_id = "HTG"; // configure

/**********************
 *  Define GPIO pins
 **********************/
#define HTG_UPSTAIRS_DEMAND 0
#define HTG_DOWNSTAIRS_DEMAND 16
#define HW_DEMAND 5
#define MAX_TEMP 2

#define HTG_UPSTAIRS_STATUS 14
#define HTG_DOWNSTAIRS_STATUS 13
#define HW_STATUS 12
#define HW_TEMP_STATUS 4

float oldTemprature = 0;

bool bManMode = false; // true = Manual, false = automatic
bool bCheckUS = true;
bool bCheckDS = true;
bool bCheckHW = true;

	int upHeatDemand = 0;
	int downHeatDemand = 0;
	int waterHeatDemand = 0;
	int demand = 0;

/************************************
 * Define relay states to demand heat
 ************************************/
int ON = 0;	 // pin LOW
int OFF = 1; // pin HIGH

//************************
// Application specific
//************************
bool processCntrlMessageApp_Ext(char *, const char *, const char *, const char *);
void processAppTOD_Ext();

devConfig espDevice;

Ticker checkValveAndDemandStatus;
Ticker configurationTimesReceived;
Ticker checkBoileroutputTimer;

bool timesReceived;

void setup()
{
	//***************************************************
	// Set-up Platform - hopefully dont change this
	//***************************************************
	bool configWiFi = false;
	Serial.begin(115200);
	while (!Serial)
		delay(300);

	espDevice.setup(deviceName, deviceType);
	Serial.println("\nStarting Outside lights Controller on ");
	Serial.println(ARDUINO_BOARD);

	drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
	if (drd->detectDoubleReset())
	{
		configWiFi = true;
	}

	// this app is a contoller
	// configure the MQTT topics for the Controller
	USCntrlState.setCntrlObjRef(USCntrlState);
	DSCntrlState.setCntrlObjRef(DSCntrlState);
	HWCntrlState.setCntrlObjRef(HWCntrlState);

	USCntrlState.setCntrlName((String)app_id + "US");
	USCntrlState.setRefreshID(RefreshID);

	DSCntrlState.setCntrlName((String)app_id + "DS");
	DSCntrlState.setRefreshID(RefreshID);

	HWCntrlState.setCntrlName((String)app_id + "HW");
	HWCntrlState.setRefreshID(RefreshID);

	// startCntrl();

	// Platform setup: Set up and manage: WiFi, MQTT and Telnet
	platform_setup(configWiFi);

	//***********************
	// Application setup
	//***********************
	// Define the output pins
	pinMode(HTG_UPSTAIRS_DEMAND, OUTPUT);
	pinMode(HTG_DOWNSTAIRS_DEMAND, OUTPUT);
	pinMode(HW_DEMAND, OUTPUT);
	pinMode(MAX_TEMP, OUTPUT);
	// Define the input pins
	pinMode(HTG_UPSTAIRS_STATUS, INPUT);
	pinMode(HTG_DOWNSTAIRS_STATUS, INPUT);
	pinMode(HW_STATUS, INPUT);
	pinMode(HW_TEMP_STATUS, INPUT);
	// Boiler Temp
	pinMode(A0, INPUT);
	// Intialise start up status
	// HIGH == Demand OFF
	digitalWrite(HTG_UPSTAIRS_DEMAND, HIGH);
	digitalWrite(HTG_DOWNSTAIRS_DEMAND, HIGH);
	digitalWrite(HW_DEMAND, HIGH);
	digitalWrite(MAX_TEMP, HIGH);

	upHeatDemand = 0;
	downHeatDemand = 0;
	waterHeatDemand = 0;

	configurationTimesReceived.attach(30, startTimesReceivedChecker);
	checkBoileroutputTimer.attach(60, readBoilerTemprature);
	checkValveAndDemandStatus.attach(10, checkValveStatus);
}

void loop()
{
	// char logString[MAX_LOGSTRING_LENGTH];
	drd->loop();

	// Go look for OTA request
	ArduinoOTA.handle();

	handleTelnet();
}

//****************************************************************
// Process any application specific inbound MQTT messages
// Return False if none
// Return true if an MQTT message was handled here
//****************************************************************
bool onMqttMessageAppExt(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties, const size_t &len, const size_t &index, const size_t &total)
{
	(void)payload;

	char mqtt_payload[len + 1];
	mqtt_payload[len] = '\0';
	strncpy(mqtt_payload, payload, len);

	if (reporting == REPORT_DEBUG)
	{
		mqttLog(mqtt_payload, true, true);
	}

	// Prceess the messages for each controller created
	USCntrlState.onMqttMessageCntrlExt(topic, payload, properties, len, index, total);
	DSCntrlState.onMqttMessageCntrlExt(topic, payload, properties, len, index, total);
	HWCntrlState.onMqttMessageCntrlExt(topic, payload, properties, len, index, total);

	return false; // FIXTHIS - I thiink I need to return true is a message is processed
}

void processAppTOD_Ext()
{
	mqttLog("HEATING Application Processing TOD", true, true);
}

bool processCntrlMessageApp_Ext(char *mqttMessage, const char *onMessage, const char *offMessage, const char *commandTopic)
{
	if (strcmp(mqttMessage, "SET") == 0)
	{
		// mqttClient.publish(StateUpstairsRuntime,1, true, "AUTO");			// This just sets the UI to show that MAN start is OFF
		return true;
	}
	return false;
}

/***********************************************************************************************************************************************
 *  Read Temperature sensor TMP36.
 *        Offset: 0.5v (500mV)
 *        Ratio:  10mV/Degree Centigrade
 *        ADC:    reads mv (i.e 0.5V is read as 500.
 *        Hard ware implementation: To avoid exceeding 1V on ADC pin I used a 1K:2K2 Resistor ratio.
 *                                  This means 100 Degree Centigrade = 1V ( I assume my boiler wont get to 100degC )
 *                                  1:2.2 drops voltage by 2K2/3K2 volts i.e. 0.6875.
 *                                  So the TMP36 would output 1.5v at 100 degC but with resistors (0.6875 * 1.5 = 1V)
 *                                  The ADC pin reads 0.6875 less than expected.  Multiplying by the reciprocal of 0.6875 corrects the reading.
 ***********************************************************************************************************************************************/
void readBoilerTemprature()
{
	char buff[20];
	int reading = analogRead(0);
	// reading = reading + 10; // reading is out 10mV from Digital volt meter, approx 1 degree
	reading = reading * 1.07;
	// FIX THIS - does the above need to be a multiplier ??
	float newTemprature = ((reading * 1.45455) - 500) / 10;
	if (oldTemprature != newTemprature)
	{
		oldTemprature = newTemprature;
		dtostrf(newTemprature, 3, 0, buff); // 3 is minimum width, 1 is precision - not sure about rounding !!
		mqttClient.publish(StateBoilerTemprature, 0, true, buff);

		/* Serial.print(reading);
		Serial.print(" Temperature: ");
		Serial.print(buff);
		Serial.println(" degrees centigrade");
		 */
	}
}

// Subscribe to application specific topics
void appMQTTTopicSubscribe()
{

	DSCntrlState.setWDUIcommandStateTopic(StateDownstairsRuntime);
	DSCntrlState.setWDCntrlTimesTopic(CommandDownstairsWDTimes);
	DSCntrlState.setWDUIcommandStateTopic(CmdStateDownstairsWD);
	DSCntrlState.setWECntrlRunTimesStateTopic(StateDownstairsRuntime);
	DSCntrlState.setWECntrlTimesTopic(CommandDownstairsWETimes);
	DSCntrlState.setWEUIcommandStateTopic(CmdStateDownstairsWE);

	USCntrlState.setWDUIcommandStateTopic(StateUpstairsRuntime);
	USCntrlState.setWDCntrlTimesTopic(CommandUpstairsWDTimes);
	USCntrlState.setWDUIcommandStateTopic(CmdStateUpstairsWD);
	USCntrlState.setWECntrlRunTimesStateTopic(StateUpstairsRuntime);
	USCntrlState.setWECntrlTimesTopic(CommandUpstairsWETimes);
	USCntrlState.setWEUIcommandStateTopic(CmdStateUpstairsWE);

	HWCntrlState.setWDUIcommandStateTopic(StateHotwaterRuntime);
	HWCntrlState.setWDCntrlTimesTopic(CommandHotwaterWDTimes);
	HWCntrlState.setWDUIcommandStateTopic(CmdStateHotwaterWD);
	HWCntrlState.setWECntrlRunTimesStateTopic(StateHotwaterRuntime);
	HWCntrlState.setWECntrlTimesTopic(CommandHotwaterWETimes);
	HWCntrlState.setWEUIcommandStateTopic(CmdStateHotwaterWE);
}
//
// status: 0=closed, 1=open
// demand: 0=no demand, 1=demand
//
// if different then waiting
// if the same then Heat or no Heat
//

void checkValveStatus()
{
     setBoilerDemand();

	//########################################
	// test UPSTAIRS  Demand and Supply
	//########################################
	if (digitalRead(HTG_UPSTAIRS_STATUS) == 0 && upHeatDemand == 0)
	{
		printTelnet("Upstairs: Status : OFF , DEMAND : OFF");
		mqttClient.publish(StateUpstairsRuntime, 0, true, "OFF");
	}
	else if (digitalRead(HTG_UPSTAIRS_STATUS) == 1 && upHeatDemand == 1)
	{
		printTelnet("Upstairs: Status : ON , DEMAND : ON");
		mqttClient.publish(StateUpstairsRuntime, 0, true, "ON");
		demand = 1;
	}
	else
	{
		printTelnet("upstairs: Status : Valve opening or closing");
		mqttClient.publish(StateUpstairsRuntime, 0, true, "WAIT"); // send WAIT if changing valve state. i.e. wait for motorised valve to open or close
	}
	//#########################################
	// test DOWNSTAIRS  Demand and Supply
	//#########################################
	if (digitalRead(HTG_DOWNSTAIRS_STATUS) == 0 && downHeatDemand == 0)
	{
			printTelnet("Downstairs: Status : OFF , DEMAND : OFF");
			mqttClient.publish(StateDownstairsRuntime, 0, true, "OFF");
	}
	else if (digitalRead(HTG_DOWNSTAIRS_STATUS) == 1 && downHeatDemand == 1)
	{
			printTelnet("Downstairs: Status : ON , DEMAND : ON");
			mqttClient.publish(StateDownstairsRuntime, 0, true, "ON");
			demand = 1;
	}
	else
	{
		printTelnet("Downstairs: Status : Valve opening or closing");
		mqttClient.publish(StateDownstairsRuntime, 0, true, "WAIT"); // send WAIT if changing valve state. i.e. wait for motorised valve to open or close
	}
	//####################################
	// test HOTWATER  Demand and Supply
	//###################################
	if (digitalRead(HW_STATUS) == 0 && waterHeatDemand == 0)
	{
		printTelnet("Hotwater: Status : OFF , DEMAND : OFF");
		mqttClient.publish(StateHotwaterRuntime, 0, true, "OFF");
	}
	else if (digitalRead(HW_STATUS) == 1 && waterHeatDemand == 1)
	{
		printTelnet("Hotwater: Status : ON , DEMAND : ON");
		mqttClient.publish(StateHotwaterRuntime, 0, true, "ON");
		demand = 1;
	}
	else
	{
		printTelnet("Hotwater: Status : Valve opening or closing");
		mqttClient.publish(StateHotwaterRuntime, 0, true, "WAIT"); // send WAIT if changing valve state. i.e. wait for motorised valve to open or close
	}
}

void setBoilerDemand()
{
	if ( waterHeatDemand == 0 && downHeatDemand == 0 && upHeatDemand == 0)
	{
		printTelnet("Switch boiler off");
		demand = 0;
	}
	if (demand == 1)
	{
		printTelnet("Switch boiler on");
	}
}




void app_WD_on(void *cid)
{
	cntrlState *obj = (cntrlState *)cid;
	String msg = obj->getCntrlName() + " WD ON";
	mqttLog(msg.c_str(), true, true);

	// FIXTHIS : Remove hard codeing of controller name

	if (obj->getCntrlName() == "HTGUS")
	{
			printTelnet("Upstairs switching on.");
			digitalWrite(HTG_UPSTAIRS_DEMAND, ON); 					    // Open valve
			upHeatDemand = 1;											// Record value as digitalRead of OUT pin is not working (FIXTHIS)	
	}
	if (obj->getCntrlName() == "HTGDS")
	{
			printTelnet("Downstairs switching on.");
			digitalWrite(HTG_DOWNSTAIRS_DEMAND, ON);					// open valve
			downHeatDemand = 1;											// Record value as digitalRead of OUT pin is not working (FIXTHIS)
	}
	if (obj->getCntrlName() == "HTGHW")
	{
			printTelnet("Hotwater switching on.");
			digitalWrite(HW_DEMAND, ON); 								// open valve
			waterHeatDemand = 1;										// Record value as digitalRead of OUT pin is not working (FIXTHIS)
	}
}

void app_WD_off(void *cid)
{
	cntrlState *obj = (cntrlState *)cid;
	String msg = obj->getCntrlName() + " WD OFF";
	mqttLog(msg.c_str(), true, true);

	if (obj->getCntrlName() == "HTGUS")
	{
		upHeatDemand = 0;
		printTelnet("Upstairs switching off.");
		digitalWrite(HTG_UPSTAIRS_DEMAND,OFF); // Valve is open so close valve
	}
	if (obj->getCntrlName() == "HTGDS")
	{
		downHeatDemand = 0;
		printTelnet("Downstairs switching off.");
		digitalWrite(HTG_DOWNSTAIRS_DEMAND, OFF); // Valve is open so close valve
	}
	if (obj->getCntrlName() == "HTGHW")
	{
		waterHeatDemand = 0;
		printTelnet("Hotwater switching off.");
		digitalWrite(HW_DEMAND, OFF); // Valve is open so close valve
	}
}

void app_WE_on(void *cid)
{
	cntrlState *obj = (cntrlState *)cid;
	String msg = obj->getCntrlName() + " WE ON";
	mqttLog(msg.c_str(), true, true);
}

void app_WE_off(void *cid)
{
	cntrlState *obj = (cntrlState *)cid;
	String msg = obj->getCntrlName() + " WE OFF";
	mqttLog(msg.c_str(), true, true);
}
void app_WD_auto(void *cid)
{
	cntrlState *obj = (cntrlState *)cid;
	String msg = obj->getCntrlName() + " WD AUTO";
	mqttLog(msg.c_str(), true, true);
}

void app_WE_auto(void *cid)
{
	cntrlState *obj = (cntrlState *)cid;
	String msg = obj->getCntrlName() + " WE AUTO";
	mqttLog(msg.c_str(), true, true);
}

void startTimesReceivedChecker()
{
	USCntrlState.runTimeReceivedCheck();
	DSCntrlState.runTimeReceivedCheck();
	HWCntrlState.runTimeReceivedCheck();
}

void processCntrlTOD_Ext()
{
	USCntrlState.processCntrlTOD_Ext();
	DSCntrlState.processCntrlTOD_Ext();
	HWCntrlState.processCntrlTOD_Ext();
}

void telnet_extension_1(char c)
{
	USCntrlState.telnet_extension_1(c);
	DSCntrlState.telnet_extension_1(c);
	HWCntrlState.telnet_extension_1(c);
}
// Process any application specific telnet commannds
void telnet_extension_2(char c)
{
	char logString[501];
	memset(logString, 0, sizeof logString);
	sprintf(logString,
				"\r%s%i\n\r%s%i\n\r%s%i\n\r%s%i\n\r%s%i\n\r%s%i\n",
				"Upstairs Heating\t", digitalRead(HTG_UPSTAIRS_STATUS), 
				"Upstairs Demand\t\t", upHeatDemand,
				"Downstairs Heating\t", digitalRead(HTG_DOWNSTAIRS_STATUS),
				"Downstairs Demand\t", downHeatDemand,
				"Hot Water\t\t", digitalRead(HW_STATUS),
				"Hot Water Demand\t", waterHeatDemand);

				printTelnet((String)logString);
}

// Process any application specific telnet commannds
void telnet_extensionHelp(char c)
{
	printTelnet((String) "x\t\tSome description");
}

// bool onMqttMessageCntrlExt(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties, const size_t &len, const size_t &index, const size_t &total)
//{
//	return USCntrlState.onMqttMessageCntrlExt(topic, payload, properties, len, index, total); //FIXTHIS - totally broken
// }