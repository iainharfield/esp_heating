
// ##include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <AsyncMqttClient_Generic.hpp>
#include <time.h>

#include "hh_defines.h"
#include "hh_utilities.h"
#include "hh_cntrl.h"

// Folling line added to stop compilation error suddenly occuring in 2024???
#include "ESPAsyncDNSServer.h"

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
bool onMqttMessageAppCntrlExt(char *, char *, const AsyncMqttClientMessageProperties &, const size_t &, const size_t &, const size_t &);
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
void DSvalveFault();
void setHeating(void *, int, String, int, int, int );

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

	int usDemand = 0;
	int dsDemand = 0;
	int hwDemand = 0;

    int usValveRetry = 0;
    int dsValveRetry = 0;
	int hwValveRetry = 0;

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
	digitalWrite(HTG_UPSTAIRS_DEMAND, HIGH);			// OFF
	digitalWrite(HTG_DOWNSTAIRS_DEMAND, HIGH);			// OFF
	digitalWrite(HW_DEMAND, HIGH);						// OFF
	digitalWrite(MAX_TEMP, HIGH);						// NO Demand

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

	mqttLog(mqtt_payload, REPORT_DEBUG, true, true);

	// Prceess the messages for each controller created
	USCntrlState.onMqttMessageCntrlExt(topic, payload, properties, len, index, total);
	DSCntrlState.onMqttMessageCntrlExt(topic, payload, properties, len, index, total);
	HWCntrlState.onMqttMessageCntrlExt(topic, payload, properties, len, index, total);

	return false; // FIXTHIS - I thiink I need to return true is a message is processed
}

void processAppTOD_Ext()
{
	mqttLog("HEATING Application Processing TOD", REPORT_INFO, true, true);
}

bool processCntrlMessageApp_Ext(char *mqttMessage, const char *onMessage, const char *offMessage, const char *commandTopic)
{
	//String msg = "Application Specific message  handling: " + (String)mqttMessage ;
	//mqttLog(msg.c_str(), true, true);
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


//###############################################
// status: 0=closed, 1=open
// demand: 0=no demand, 1=demand
//
// if different then waiting
// if the same then Heat or no Heat
//###############################################
void checkValveStatus()
{
     setBoilerDemand();

	//########################################
	// test UPSTAIRS  Demand and Supply
	//########################################
	if (digitalRead(HTG_UPSTAIRS_STATUS) == 0 && upHeatDemand == 0)
	{
		usValveRetry = 0;

		mqttLog("Upstairs: Status : OFF , DEMAND : OFF", REPORT_DEBUG, true, true);
		
		mqttClient.publish(StateUpstairsRuntime, 1, true, "OFF");
		mqttClient.publish(StateUpstairsValveState,  1, true, "OK");				// Updates UI
	}
	else if (digitalRead(HTG_UPSTAIRS_STATUS) == 1 && upHeatDemand == 1)
	{
		usValveRetry = 0;
		usDemand = 1;

		mqttLog("Upstairs: Status : ON , DEMAND : ON", REPORT_DEBUG, true, true);

		mqttClient.publish(StateUpstairsRuntime, 1, true, "ON");
		mqttClient.publish(StateUpstairsValveState,  0, true, "OK");
	}
	else
	{
		usValveRetry++;
		mqttLog("Upstairs: Status : Valve opening or closing", REPORT_INFO, true, true);
		mqttClient.publish(StateUpstairsRuntime, 1, true, "WAIT"); // send WAIT if changing valve state. i.e. wait for motorised valve to open or close
	}
	//#########################################
	// test DOWNSTAIRS  Demand and Supply
	//#########################################
	if (digitalRead(HTG_DOWNSTAIRS_STATUS) == 0 && downHeatDemand == 0)
	{
		dsValveRetry = 0;

		mqttLog("Downstairs: Status : OFF , DEMAND : OFF", REPORT_DEBUG, true, true);

		mqttClient.publish(StateDownstairsRuntime, 1, true, "OFF");					// Updates UI
		mqttClient.publish(StateDownstairsValveState,  1, true, "OK");				// Updates UI
	}
	else if (digitalRead(HTG_DOWNSTAIRS_STATUS) == 1 && downHeatDemand == 1)
	{
		dsValveRetry = 0;
		dsDemand = 1;

		mqttLog("Downstairs: Status : ON , DEMAND : ON", REPORT_DEBUG, true, true);
	
		mqttClient.publish(StateDownstairsRuntime, 1, true, "ON");
		mqttClient.publish(StateDownstairsValveState,  0, true, "OK");
	}
	else
	{
		dsValveRetry++;
		mqttLog("Downstairs: Status : Valve opening or closing", REPORT_INFO, true, true);
		mqttClient.publish(StateDownstairsRuntime, 1, true, "WAIT"); // send WAIT if changing valve state. i.e. wait for motorised valve to open or close
	}

	//#########################################
	// test HOTWATER  Demand and Supply
	//#########################################
	if (digitalRead(HW_STATUS) == 0 && waterHeatDemand == 0)
	{
		hwValveRetry = 0;

		mqttLog("Hotwater: Status : OFF , DEMAND : OFF", REPORT_DEBUG, true, true);
		
		mqttClient.publish(StateHotwaterRuntime, 1, true, "OFF");					// Updates UI
		mqttClient.publish(StateHotwaterValveState,  1, true, "OK");				// Updates UI		
	}
	else if (digitalRead(HW_STATUS) == 1 && waterHeatDemand == 1)
	{
		hwValveRetry = 0;
		hwDemand = 1;

		mqttLog("Hotwater: Status : ON , DEMAND : ON", REPORT_DEBUG, true, true);
		
		mqttClient.publish(StateHotwaterRuntime, 1, true, "ON");					// Updates UI
		mqttClient.publish(StateHotwaterValveState,  0, true, "OK");				// Updates UI
	}
	else
	{
		hwValveRetry++;
		mqttLog("Hotwater: Status : Valve opening or closing",REPORT_INFO, true, true);
		mqttClient.publish(StateHotwaterRuntime, 1, true, "WAIT"); 					// send WAIT if changing valve state. i.e. wait for motorised valve to open or close	
	}
	

	// Update UI on valve state
	if (usValveRetry >2) 
		mqttClient.publish(StateUpstairsValveState,  0, true, "FAULT");			// Updates UI
	if (dsValveRetry >2)
		mqttClient.publish(StateDownstairsValveState,  0, true, "FAULT");				// Updates U
	if (hwValveRetry >2)
		mqttClient.publish(StateHotwaterValveState,  0, true, "FAULT");				// Updates U


	setBoilerDemand();
}

void setBoilerDemand()
{
	if ( waterHeatDemand == 0 && downHeatDemand == 0 && upHeatDemand == 0)
	{

		mqttLog("Switch boiler OFF" ,REPORT_DEBUG, true, true);
		
		usDemand = 0;
		dsDemand = 0;
		hwDemand = 0;
		digitalWrite(MAX_TEMP, OFF);					// Swich boiler OFF
	}
	if (usDemand == 1 || dsDemand == 1 || hwDemand == 1)
	{
		
		mqttLog("Switch boiler ON", REPORT_DEBUG, true, true);
		
		digitalWrite(MAX_TEMP, ON);						// Swich boiler ON
	}
}



void app_WD_on(void *cid)
{
	if (coreServices.getWeekDayState() == 1)			// 1 means weekday
	{
		setHeating(cid, ON, "WD ON", HTG_UPSTAIRS_DEMAND, HTG_DOWNSTAIRS_DEMAND, HW_DEMAND);
	}	
}
void app_WD_off(void *cid)
{
	if (coreServices.getWeekDayState() == 1)			// 1 means weekday
	{
		setHeating(cid, OFF, "WD OFF", HTG_UPSTAIRS_DEMAND, HTG_DOWNSTAIRS_DEMAND, HW_DEMAND);
	}	
}
void app_WE_on(void *cid)
{
	if (coreServices.getWeekDayState() == 0)			// 0 means weekend
	{
		setHeating(cid, ON, "WE ON", HTG_UPSTAIRS_DEMAND, HTG_DOWNSTAIRS_DEMAND, HW_DEMAND);
	}	
}
void app_WE_off(void *cid)
{
	if (coreServices.getWeekDayState() == 0)			// 0 means weekend
	{
		setHeating(cid, OFF, "WE OFF", HTG_UPSTAIRS_DEMAND, HTG_DOWNSTAIRS_DEMAND, HW_DEMAND);
	}	
}
void app_WD_auto(void *cid)
{
	if (coreServices.getWeekDayState() == 1)			// 1 means weekday
	{
		cntrlState *obj = (cntrlState *)cid;
		String msg = obj->getCntrlName() + " WD AUTO";
		mqttLog(msg.c_str(), REPORT_INFO, true, true);
							
		//mqttClient.publish(getWDCntrlRunTimesStateTopic().c_str(), 0, true, "AUTO");
		mqttClient.publish(obj->getWDUIcommandStateTopic().c_str(), 1, true, "SET"); //
	}	
}
void app_WE_auto(void *cid)
{
	if (coreServices.getWeekDayState() == 0)			// 0 means weekend
	{
		cntrlState *obj = (cntrlState *)cid;
		String msg = obj->getCntrlName() + " WE AUTO";
		mqttLog(msg.c_str(), REPORT_INFO, true, true);
		//mqttClient.publish(getWECntrlRunTimesStateTopic().c_str(), 0, true, "AUTO");
		mqttClient.publish(obj->getWEUIcommandStateTopic().c_str(), 1, true, "SET");
	}
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


void setHeating(void *cid, int state, String stateMsg, int usDemand, int dsDemand, int hwDemand )
{
	cntrlState *obj = (cntrlState *)cid;
	String msg = obj->getCntrlName() + "," +  stateMsg;
	mqttLog(msg.c_str(), REPORT_INFO, true, true);

	// FIXTHIS : Remove hard codeing of controller name

	if (obj->getCntrlName() == "HTGUS")
	{
		digitalWrite(usDemand, state); 					// Open valve
		if (state == ON)
		{
			upHeatDemand = 1;							// Record value as digitalRead of OUT pin is not working (FIXTHIS)
			USCntrlState.setOutputState(1);				// Feedback the Controller tho output of the requested action
		}
		else
		{
			upHeatDemand = 0;
			USCntrlState.setOutputState(0);				// Feedback the Controller tho output of the requested action
		}										
	}
	if (obj->getCntrlName() == "HTGDS")
	{
		digitalWrite(dsDemand, state);					// open valve
		if (state == ON)
		{
			downHeatDemand = 1;							// Record value as digitalRead of OUT pin is not working (FIXTHIS)
			DSCntrlState.setOutputState(1);				// Feedback the Controller tho output of the requested action
		}
		else
		{
			downHeatDemand = 0;	
			DSCntrlState.setOutputState(0);				// Feedback the Controller tho output of the requested action
		}
	}
	if (obj->getCntrlName() == "HTGHW")
	{
		digitalWrite(hwDemand, state); 					// open valve
		if (state == ON)
		{
			waterHeatDemand = 1;						// Record value as digitalRead of OUT pin is not working (FIXTHIS)
			HWCntrlState.setOutputState(1);				// Feedback the Controller tho output of the requested action
		}
		else
		{
			waterHeatDemand = 0;
			HWCntrlState.setOutputState(0);				// Feedback the Controller tho output of the requested action
		}	
	}
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

bool onMqttMessageAppCntrlExt(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties, const size_t &len, const size_t &index, const size_t &total)
{
	return false;
}