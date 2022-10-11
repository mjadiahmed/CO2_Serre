#ifndef GLOBALS_H
#define GLOBALS_H


#include "config.h"
#include <stdio.h>
#include <string.h>
#define TINY_GSM_MODEM_SIM808
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


// classes
#include "controller.h"
#include "stringSplitter.h"
//#include "packetBuilder.h"


void DBGG(String);
void DBGG(unsigned char, int);
void DBGL(String);
void DBGL(unsigned char, int);

namespace global
{
    // Routine functions
    /**
     * Function:            handles everything related to sim card and network registration.
     * */
    bool checkSim();


    /**
     * Function:            handles Broker status.
     * */
    bool checkBroker();


    /**
     * Function:            enable/disables Debug via USB.
     * 
     * @param status        debug status
     * */
    void enableDebug(bool status);

    /**
     * Function:            returns topic string
     * 
     * @param topic         raw topic
     * */
    String resolveTopic(String topic);

    /**
     * Function:            handle status LEDs and Buzzer
     * */
    void status();
}

namespace time
{
    /**
     * Function:            requests time from the ESP or network.
     * */
    String getTime();

    /**
     * Function:            syncs the modem to the provided ntp server and time zone.
     * 
     * @param server        ntp server
     * @param timezone      time zone
     * */
    bool setNtp(String server, int timezone);

    /**
     * Function:            converts DateTime to timestamps.
     * */
    String convertDateTime();

    /**
     * Function:            converts date to number of days.
     * 
     * @param y             year
     * @param m             month
     * @param d             day      
     * */
    uint16_t date2days(uint16_t y, uint8_t m, uint8_t d);

    /**
     * Function:            converts time to number seconds.
     * 
     * @param days          days
     * @param h             hours
     * @param m             minutes
     * @param s             seconds
     * */
    uint32_t time2ulong(uint16_t days, uint8_t h, uint8_t m, uint8_t s);
}

// SENSOR functions
namespace sensor
{
    /**
     * Function:            gets temperature current temperature.
     * */
    bool getTemperature();

    /**
     * Function:            resets the sensor
     * */
    void reset();
}

// SIM functions
namespace sim
{
    /**
     * Function:            returns modem status ON/OFF.
     * */
    bool isSimOn();
    /**
     * Function:            powers ON/OFF the SIMCOM module.
     * */
    bool powerSim(bool status);
    /**
     * Function:            returns restarts the SIMCOM module.
     * */
    bool restartSim();
    /**
     * Function:            returns whether or not the sim card is locked.
     * */
    bool isSimLocked();
    /**
     * Function:            attempts to unlock the sim card using the provided pin code.
     * */
    bool unlockSim(char* pin);
    /**
     * Function:            returns whether or not the modem is registered to the network.
     * */
    bool isSimRegistered();

    /**
     * Function:            picks which APN to use.
     * */
    bool resolveApn();

    /**
     * Function:            sets the given apn params and connects to gprs.
     * 
     * @param apn           apn url
     * @param user          apn username
     * @param password      apn password
     * */
    bool setApn(const char* apn, const char* usr, const char* pwd);

    /**
     * Function:            returns the IP address if exists otherwise it returns (.).
     * */
    String getIp();

    /**
     * Function:            returns the ISP code.
     * */
    String getIsp();

    /**
     * Function:            returns the IMEI.
     * */
    String getImei();

    /**
     * Function:            returns the CCID.
     * */
    String getCcid();

    /**
     * Function:            returns the signal quality.
     * */
    String getCsq();

    /**
     * Function:            returns whether or not the module is registered to the network.
     * */
    bool isSimConnected();

    /**
     * Function:            send SMS
     * 
     * @param number        phone number
     * @param message       message
     * */
    bool sendSms(String number, String message);
}


// MQTT functions
namespace mqtt
{
    /**
     * Function             connects to broker.
     * */
    bool mqttConnect();

    /**
     * Function:            sends packets to provided broker topic.
     * 
     * @param topic         mqtt topic
     * @param packet        json like string packet
     * */
    bool sendPacket(String topic, String packet);

    /**
     * Function:            subscribs to provided topic.
     * 
     * @param topic         mqtt topic
     * */
    bool mqttSubscribe(String topic);

    /**
     * Function:            unsubscribe to provided topic.
     * 
     * @param topic         mqtt topic
     * */
    bool mqttUnsubscribe(String topic);

    /**
     * Function:            callback function that will be called when a packet is received.
     * 
     * @param topic
     * @param payload
     * @param lenght
     * */
    void mqttCallback(char* topic, byte* payload, unsigned int length);

    /**
     * Function:            returns whether or not the modem is connected to broker.
     * */
    bool isBrokerConnected();

    /**
     * Function:            returns the connection status
     * */
    void brokerConnectionStatus();
}

// INFO functions
namespace info
{
    /**
     * Function:                assembles info data.
     * */
    bool assemblePacket(String lci = "0");

    /**
     * Function:                sends info packet or pong in response to ping.
     * 
     * @param type              payload type
     * */
    bool sendInfo(String type);
}



#endif
