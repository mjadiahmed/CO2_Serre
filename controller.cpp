#include "controller.h"

void controller::debugCommands(String raw_input)
{
    String command = "";
    String value = "";

    if(stringSplitter::split(raw_input, command, value))
    {
        // DBGG("[i] Executing command: " + command + "\n [i] value: " + value);

        // DEBUG COMMANDS:
        if(command == "POWSIM")
        {
            if(value == "1") sim::powerSim(1);
            else if(value == "0") sim::powerSim(0);
        }

       

        if(command == "RESTART")
        {
            sim::restartSim();
        }

        if(command == "GETTIME")
        {
            time::getTime();
        }

        if(command == "MQTT")
        {
            mqtt::mqttConnect();
        }

        if(command == "UNLOCK")
        {
            DBGG("[d] pin: " + value);
            sim::unlockSim(const_cast<char*>(value.c_str()));
        }

        if(command == "ISLOCKED")
        {
            sim::isSimLocked();
        }

        if(command == "ISREG")
        {
            sim::isSimRegistered();
        }

        if(command == "GETIP")
        {
            sim::getIp();
        }

        if(command == "GETOP")
        {
            DBGG(sim::getIsp());
        }

        if(command == "GETCCID")
        {
            DBGG(sim::getCcid());
        }

        if(command == "SETAPN")
        {
            DBGG("[d] APN: " + value);
            sim::setApn(const_cast<char*>(value.c_str()), NULL, NULL);
        }

    


        if(command == "MQTT")
        {
            if(value == "CONNECT") mqtt::mqttConnect();
            if(value == "SEND") mqtt::sendPacket("", "");
        }

        if(command == "GETIMEI")
        {
            DBGG("[d] Imei: " + sim::getImei());
        }

        if(command == "GETCSQ")
        {
            DBGG("[d] Csq: " + sim::getCsq());
        }

        if(command == "DEBUG")
        {
            if(value == "1")
            {
                global::enableDebug(1);
                DBGG("[d] Debugger enabled.");
            }
            else
            {
                global::enableDebug(0);
                DBGG("[d] Debugger disabled.");
            }
        }
    }
    else DBGG("[d] Invalid command!");
}
