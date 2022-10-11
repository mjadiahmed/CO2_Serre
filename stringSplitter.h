#ifndef STRING_SPLITTER_H
#define STRING_SPLITTER_H

#include "globals.h"

namespace stringSplitter
{
    /**
     * Function:                    splits the input string into command and value.
     * 
     * @param raw                   input string
     * @param cmd                   command variable that will be changed
     * @param val                   value variable that will be changed
     * */
    bool split(String raw, String &cmd, String &val);

    /**
     * Function:                    splits the raw packet into type and payload.
     * 
     * @param raw                   raw json like string
     * @param type                  type of command (passed by ref)
     * */
    bool getType(String raw, String& type);
    
    /**
     * Function:                    splits the payload into variables.
     * 
     * @param payload               raw json payload
     * @param is_gps_on             gps flag
     * @param is_can0_on            can0 flag
     * @param is_can1_on            can1 flag
     * @param push_event_interval   events timer
     * @param event_topic_name      events topic
     * @param device_type           device type
     * @param rpc_topic_name        config topic
     * @param res_rpc_topic_name    response topic
     * @param last_config_id        last config ID
     * */
    bool parsePayload(String payload, bool& is_gps_on, bool& is_can0_on, bool& is_can1_on, uint32_t& push_event_interval, String& event_topic_name, String& rpc_topic_name, String& res_rpc_topic_name, String& last_config_id);

    /**
     * Function:                    splits the payload into variables
     * 
     * @param payload               raw json payload
     * @param number                phone number
     * @param message               sms message
     * */
    bool parsePayload(String payload, String& number, String& message);

    /**
     * Function:                    splits the payload into variables
     * 
     * @param pid                   pid
     * */
    bool parsePayload(String payload, unsigned char& pid);

    /**
     * Function:                    checks topics for errors
     * 
     * @param topic                 topic string
     * */
    bool checkTopic(String topic);
}

#endif
