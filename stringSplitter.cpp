#include "stringSplitter.h"

bool stringSplitter::split(String raw, String &cmd, String &val)
{
    String raw_cmd;
    char c_compare;
    String output[2];

    // filter text
    for(unsigned int char_index = 0; char_index < raw.length(); char_index++)
    {
        c_compare = raw[char_index];
        //DBGG("[s] " + String(char_index) + ": " + c_compare);
        if(c_compare != ' '
            && c_compare != '\a'
            && c_compare != '\b'
            && c_compare != '\t'
            && c_compare != '\n'
            && c_compare != '\v'
            && c_compare != '\f'
            && c_compare != '\r')
        {
            raw_cmd += c_compare;
        }
    }

    // parse data
    if(raw_cmd.indexOf('#') >= 0 && raw_cmd.indexOf('!') > 0 && raw_cmd.indexOf('*') > 0)
    {
        output[0] = raw_cmd.substring(raw_cmd.indexOf('#')+1, raw_cmd.indexOf('!'));
        output[1] = raw_cmd.substring(raw_cmd.indexOf('!')+1, raw_cmd.indexOf('*'));
    }
    else
    {
        return 0;
    }
    

    // output
    cmd = output[0];
    val = output[1];
    return 1;
}

bool stringSplitter::getType(String raw, String& type)
{
    // error handler
    if(raw.indexOf("type") == -1)
    {
        return 0;
    }

    DynamicJsonDocument json_packet(1024);
    deserializeJson(json_packet, raw);

    String _type = json_packet["type"];
    DBGG("[d] Type: " + _type);
    type = _type;
    return 1;
}

bool stringSplitter::parsePayload(String payload, bool& is_gps_on, bool& is_can0_on, bool& is_can1_on, uint32_t& push_event_interval, String& event_topic_name, String& rpc_topic_name, String& res_rpc_topic_name, String& last_config_id)
{
    // error handler
    if(payload.indexOf("is_gps_on") == -1
    || payload.indexOf("is_can0_on") == -1
    || payload.indexOf("is_can1_on") == -1
    || payload.indexOf("push_event_interval") == -1
    || payload.indexOf("event_topic_name") == -1
    || payload.indexOf("rpc_topic_name") == -1
    || payload.indexOf("res_rpc_topic_name") == -1
    || payload.indexOf("config_id") == -1)
    {
        return 0;
    }
    DBGG("[t] String packet ok");

    DynamicJsonDocument json_payload(1024);
    deserializeJson(json_payload, payload);
    if(!json_payload.containsKey("payload")
    || !json_payload["payload"].containsKey("is_gps_on")
    || !json_payload["payload"].containsKey("is_can0_on")
    || !json_payload["payload"].containsKey("is_can1_on")
    || !json_payload["payload"].containsKey("push_event_interval")
    || !json_payload["payload"].containsKey("event_topic_name")
    || !json_payload["payload"].containsKey("rpc_topic_name")
    || !json_payload["payload"].containsKey("res_rpc_topic_name")
    || !json_payload["payload"].containsKey("config_id"))
    {
        return 0;
    }
    DBGG("[t] json keys ok");
    
    if(!json_payload["payload"]["is_gps_on"].is<bool>()
    || !json_payload["payload"]["is_can0_on"].is<bool>()
    || !json_payload["payload"]["is_can1_on"].is<bool>()
    || !json_payload["payload"]["is_delta_on"].is<bool>()
    || !json_payload["payload"]["push_event_interval"].is<unsigned int>()
    || !json_payload["payload"]["push_warning_interval"].is<unsigned int>()
    || !json_payload["payload"]["device_type"].is<signed int>())
    {
        return 0;
    }
    DBGG("[t] json types ok");

    is_gps_on = json_payload["payload"]["is_gps_on"];
    is_can0_on = json_payload["payload"]["is_can0_on"];
    is_can1_on = json_payload["payload"]["is_can1_on"];
    push_event_interval = json_payload["payload"]["push_event_interval"];
    const char* event_topic_name_t = json_payload["payload"]["event_topic_name"];
    const char* rpc_topic_name_t = json_payload["payload"]["rpc_topic_name"];
    const char* res_rpc_topic_name_t = json_payload["payload"]["res_rpc_topic_name"];
    const char* last_config_id_t = json_payload["payload"]["config_id"];

    if(!stringSplitter::checkTopic(event_topic_name_t)
    || !stringSplitter::checkTopic(rpc_topic_name_t)
    || !stringSplitter::checkTopic(res_rpc_topic_name_t))
    {
        return 0;
    }
    DBGG("[t] json values ok");

    event_topic_name = event_topic_name_t;
    rpc_topic_name = rpc_topic_name_t;
    res_rpc_topic_name = res_rpc_topic_name_t;
    last_config_id = last_config_id_t;

    DBGG("[D] is_gps_on: " + String(is_gps_on));
    DBGG("[D] is_can0_on: " + String(is_can0_on));
    DBGG("[D] is_can1_on: " + String(is_can1_on));
    DBGG("[D] push_event_interval: " + String(push_event_interval));
    DBGG("[D] event_topic_name: " + event_topic_name);
    DBGG("[D] rpc_topic_name: " + rpc_topic_name);
    DBGG("[D] res_rpc_topic_name: " + res_rpc_topic_name);
    DBGG("[D] last_config_id: " + last_config_id);
    return 1;
}

bool stringSplitter::parsePayload(String payload, String& number, String& message)
{
    if(payload.indexOf("number") == -1
    || payload.indexOf("message") == -1)
    {
        return 0;
    }
    DBGG("[t] String packet ok");

    DynamicJsonDocument json_payload(1024);
    deserializeJson(json_payload, payload);
    if(!json_payload.containsKey("payload")
    || !json_payload["payload"].containsKey("number")
    || !json_payload["payload"].containsKey("message"))
    {
        return 0;
    }
    DBGG("[t] json pakcet ok");

    const char* number_t = json_payload["payload"]["number"];
    const char* message_t = json_payload["payload"]["message"];
    // TODO: validate phone number
    DBGG("[t] phone number ok");

    number = number_t;
    message = message_t;
    return 1;
}

bool stringSplitter::parsePayload(String payload, unsigned char& pid)
{
    if(payload.indexOf("pid") == -1)
    {
        return 0;
    }
    DBGG("[t] String packet ok");

    DynamicJsonDocument json_payload(1024);
    deserializeJson(json_payload, payload);
    if(!json_payload.containsKey("payload")
    || !json_payload["payload"].containsKey("pid"))
    {
        return 0;
    }
    DBGG("[t] json packet ok");

    // const char* pid_t = json_payload["payload"]["can"];
    int pid_t = json_payload["payload"]["pid"];

    pid = (unsigned char) pid_t;
    DBGG("[t] can ID ok");
    // DBGL("CAN PID: ");
    // DBGG(pid, HEX);
    
    return 1;
}

bool stringSplitter::checkTopic(String topic)
{
    if(topic.length() > 0 && topic.indexOf("/<serial>/") > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
