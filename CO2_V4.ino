/*Serre*/

#include "globals.h" 
#include "HardwareSerial.h"
#include "bsec.h"
#include "STM32LowPower.h"
#include "MapFloat.h"

#define iterations 3
int Counter =0;
uint32_t ts;
uint32_t Sleep_s = 240000 ;
String  TimeX;

//-- checks
unsigned long check_timer = 0;
bool sim_check = false;
bool mqt_check = false;

    
// ADC
#define ADC_BATT PA4
#define ADC_SOL PC2
bool Qualibrated=false;
int ADC_B = 0, ADC_S = 0;
float Bat_F[iterations], SOL_F[iterations];
// float ADC_Battery[4], ADC_S

// MQTT serre
String SerrePayload[4];
uint8_t PayloadCount = 0, Prev = 0;
// BME688
float temp[iterations], hum[iterations], pres[iterations], CO2[iterations], IAQ[iterations], IaqAcc[iterations], AccQ[iterations], VOCEE[iterations] , Acc = 0;



// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
// Create an object of the class Bsec
Bsec iaqSensor;
String output;
void checkIaqSensorStatus(void);
// BME688
//  Helper function definitions


// default topics
// String default_events_topic = "nextronic/devices/Serre/<serial>/events";
// String default_config_topic = "nextronic/devices/Serre/<serial>/config";
// String default_info_topic = "nextronic/devices/Serre/<serial>/info";

String default_events_topic = "devices/<serial>/events";
String default_config_topic = "devices/<serial>/config";
String default_info_topic = "devices/<serial>/info";

String jsonpayload[iterations];

// configuration struct
struct configuration_s
{
  bool valid;

  uint32_t publish_event_interval_t;
  char event_topic_name_t[30], rpc_topic_name_t[30], res_rpc_topic_name_t[30], last_config_t[30];
};
configuration_s mainConfig;

// Flash storage init

// default configuration

uint32_t _publish_event_interval = 1000 * 10 * 1; // 1min
String _event_topic_name = "";
String _rpc_topic_name = "";
String _res_rpc_topic_name = "";

// info
String _fw_version = "0.2.0";
String _local_time = "";
String _dbm = "";
String _lat = "";
String _lng = "";
String _bts = "";
String _bat = "";
String _last_config_id = "";

String _buzzer_status = "";
String _ccid = "";

// Time related data
const uint8_t daysInMonth[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30};
uint32_t x;
// SIM808 related variables
#define ISP_IAM "IAM"
#define ISP_INWI "60402"
#define ISP_ORANGE "MED TELECOM"
char *pin = const_cast<char *>("0000");
char *MA_APN[3] = {const_cast<char *>("www.iamgprs1.ma"), const_cast<char *>("www.inwi.ma"), const_cast<char *>("internet1.meditel.ma")};
String imei = "";

// TIME related variables
String dateTime;

// MQTT connection variables
// char* MQTT_SERVER = const_cast<char*>("mqtt2.nextrack.io");
char *MQTT_SERVER = const_cast<char *>("mqtt1.hardiot.com"); //mqtt1.hardiot.com     mqtt1.hardiot.com
int MQTT_PORT = 1883;
uint32_t MQTT_CONNECTION_KEEPALIVE = 60 * 2; // 2min

// Baudrates
#define Serial_BR 9600
#define SERIAL1_BR_0 9600

// Constructors definition
TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient tb(client);

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];
// can 1
long unsigned int rxId1;
unsigned char len1 = 0;
unsigned char rxBuf1[8];
char msgString1[128];

// Status flags
bool SIM_OK = false;
bool NET_OK = false;
bool BROKER_OK = false;
bool SERVER_PACKET_RDY = false;

// timer variables

uint32_t broker_connection_timeout = 0;
uint32_t packet_send_timeout = 0;
uint32_t packet_send_time = 0;

short broker_fix_timeout = 0;
short registration_timeout = 0;
short apn_timeout = 0;

// packets
String json_packet = "";
String json_info_packet = "";

// flags
bool DEBUG_FLAG = true;
bool INFO_FLAG = false;
HardwareSerial Serial1(PA10, PA9);

String payload_to_hex(const char *data, size_t len)
{
  static const char hex_table[] = "0123456789ABCDEF";
  String res = "";
  while (len--)
  {
    res += hex_table[*data >> 4];
    res += hex_table[*data & 0xF];
    data++;
  }
  return res;
}

String packetBuilderUniv(String csq, String ccid, String fv, String tmp, String hum, String co2, String prss, String batlvl, String solar_volt, String IAQQ, String ACCC, String VOCC , String Timestp, String RST)
{
  String Res = "";
  Res += csq + ",";
  Res += ccid + ",";
  Res += fv + ",";
  Res += tmp + ",";        //temperature
  Res += hum + ",";       //humidity
  Res += co2 + ",";        //co2
  Res += prss + ",";      //pressure
  Res += batlvl + ",";       //battery voltage
  Res += solar_volt + ","; //solar voltage
  Res += IAQQ + ",";        //static IAQ
  Res += ACCC + ",";        //accuracy
  Res +=  VOCC + ",";     //VOC equivalent
  Res += Timestp + ",";    //time stamp
  Res += RST;             //board reset event
  
  return payload_to_hex(Res.c_str(), Res.length());
}


void setup()
{
  // Setup UART
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("\t[i] Serre FIRMWARE version " + String(FW_Version) + "\n");

  // delay the program for debug

  // BME688
  Serial.println("\t[i] Serre TEST FIRMWARE\n");
  Wire.setSCL(I2C_SCL); // for  STM Nucleo L431RCT6
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);



//chequ sensors





  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);

  checkIaqSensorStatus();
  bsec_virtual_sensor_t sensorList[10] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_ULP);
  checkIaqSensorStatus();

   // Setup pins
  pinMode(POWER_KEY, OUTPUT);
  pinMode(SIM_STATUS, INPUT);
  pinMode(SIM_V, OUTPUT);
  digitalWrite(SIM_V, LOW); // turn off sim



  // change buffer size for mqtt packets
  tb.setBufferSize(1024);



  //=========================== Ckeck installation  ********************************************************************************************************************************************
  // SIM etage ON ---------------------------------------------------------------------------
  digitalWrite(SIM_V, HIGH); // turn off sim

  // check sim & mqtt ---------------------------------------------------------------------------
  check_timer = 0;
  sim_check = false;
  mqt_check = false;

  Serial.print(" [d] Broker:  ");
  Serial.println(String(MQTT_SERVER));

  
  Serial.println("[+] CHECKING SIM808");
  check_timer = millis();
  while ((millis() - check_timer) <= (1000 * 60 * 2)) // 60
  {
    sim_check = global::checkSim();
    if (sim_check || NET_OK)
      break;
    delay(1000);
  }
  Serial.println((sim_check || NET_OK) ? "\t[v] SIM808 CHECK\tOK" : "\t[v] SIM808 CHECK\tFAIL");

  // # check mqtt ---------------------------------------------------------------------------
  Serial.println("[+] CHECKING MQTT");
  check_timer = millis();
  while ((millis() - check_timer) <= (1000 * 60 * 2)) // 60
  {
    mqt_check = global::checkBroker();
    if (mqt_check || BROKER_OK)
      break;
    delay(1000);
  }
  Serial.println((mqt_check || BROKER_OK) ? "\t[v] MQTT CHECK\t\tOK" : "\t[v] MQTT CHECK\t\tFAIL");

  // Construct payload 255 ---------------------------------------------------------------------------
  String TestPayload = packetBuilderUniv(sim::getCsq(), sim::getCcid(), _fw_version, String(0xFF), String(0xFF), String(0xFF), String(0xFF), String(0xFF), String(0xFF), String(0xFF), String(0xFF), String(0xFF), String(ts), String(1));
  // publish payload ---------------------------------------------------------------------------
  if (tb.publish(_event_topic_name.c_str(), TestPayload.c_str()))
  {
    Serial.println("[V] Payload MQTT Published #TEST" );
  }
  else
  {
    Serial.println("[E] Payload Not Published #TEST" );
  }
  // turn SIM Off ---------------------------------------------------------------------------
  isSIM_OFF();
  // turn SIM etage Off ---------------------------------------------------------------------------
  digitalWrite(SIM_V, LOW); // turn off sim

  //=========================== END Ckeck installation ********************************************************************************************************************************************

}

void loop()
{

  // ------- init SIM Variable
    check_timer = 0;
    sim_check = false;
    mqt_check = false;

  //------------------------------------------  SIM LowPower
  isSIM_OFF();
  

  //------------------ END SIM TEST CURRENT 

  //============================= Sensors ==============================
  while (Acc == 0 && (Qualibrated == false))
  {
    if (iaqSensor.run())
    {
      Acc = iaqSensor.iaqAccuracy;
      output = String(iaqSensor.rawTemperature);
      output += " *C, " + String(iaqSensor.pressure);
      output += " Pa, " + String(iaqSensor.rawHumidity);
      output += " rH, " + String(iaqSensor.gasResistance);
      output += " GR, " + String(iaqSensor.iaq);
      output += " IAQ, [" + String(iaqSensor.iaqAccuracy);
      output += "], " + String(iaqSensor.temperature);
      output += " *C, " + String(iaqSensor.humidity);
      output += " rH, " + String(iaqSensor.staticIaq);
      output += " sIAQ, " + String(iaqSensor.co2Equivalent);
      output += " CO2, " + String(iaqSensor.breathVocEquivalent) + " VOCe";
      Serial.println(output);
    }
    else
    {
      checkIaqSensorStatus();
    }
  }

  if(Acc != 0){Qualibrated=true;}else{}

  if (Qualibrated) //if(Acc != 0)
  {
    if (iaqSensor.run())
    {
      Acc = iaqSensor.iaqAccuracy;
      output = String(iaqSensor.rawTemperature);
      output += " *C, " + String(iaqSensor.pressure);
      output += " Pa, " + String(iaqSensor.rawHumidity);
      output += " rH, " + String(iaqSensor.gasResistance);
      output += " GR, " + String(iaqSensor.iaq);
      output += " IAQ, [" + String(iaqSensor.iaqAccuracy);
      output += "], " + String(iaqSensor.temperature);
      output += " *C, " + String(iaqSensor.humidity);
      output += " rH, " + String(iaqSensor.staticIaq);
      output += " sIAQ, " + String(iaqSensor.co2Equivalent);
      output += " CO2, " + String(iaqSensor.breathVocEquivalent) + " VOCe";
      Serial.println(output);

     
      //
      // # ADC Values
      //------------------ADC---------------------------------------------------

if(PayloadCount%3 == 0){

 // # BME688 Values
      temp[Counter] = iaqSensor.temperature;
      hum[Counter] = iaqSensor.humidity;
      pres[Counter] = iaqSensor.pressure;
      CO2[Counter] = iaqSensor.co2Equivalent;
      IAQ[Counter] = iaqSensor.staticIaq;
      IaqAcc[Counter] = iaqSensor.staticIaq;
      AccQ[Counter] = iaqSensor.iaqAccuracy; 
      VOCEE[Counter] = iaqSensor.breathVocEquivalent  ;
      
      Serial.println("--- ADC ---");
      analogReadResolution(12);
      // Battery ADC
      ADC_B = 0;
              for (int i = 0; i < 20; i++)
              {
                ADC_B += analogRead(ADC_BATT);
              }
      ADC_B = ADC_B / 20;    

      Bat_F[Counter] = mapFloat(ADC_B, 2640, 3356, 4.31, 5.48); 

      
      Serial.println("Battery:  " + String(Bat_F[Counter]) + " V");

      // SOL Level
              for (int i = 0; i < 20; i++)
              {
                ADC_S += analogRead(ADC_SOL);
              }
      ADC_S = ADC_S / 20; // 3.74V=1945          4.5V=2320
      SOL_F[Counter] = mapFloat(ADC_S, 243, 825, 1.54, 5.23); // 3.74V=1945 --- 4.5V=2320
      Serial.println("Solar:  " + String(SOL_F[Counter]) + " V");
      Counter++;
}
      // SerrePayload[PayloadCount] = output;
      PayloadCount++;

      if (Prev != PayloadCount)
      {

        Prev = PayloadCount;
        Serial.println("going to sleep");
        delay(100);
        LowPower.deepSleep(Sleep_s);  //240000
        Serial.println("wake");
      }
    }
    else
    {
      checkIaqSensorStatus();
    }
  }
  //============================= END Sensors ==============================


  //----------------- MQTT Send--------------------------------------------------------

  if (Counter == iterations)
  {
    Counter = 0;


    //-------------------------------------------- check SIM & MQTT
//    unsigned long check_timer = 0;
//    bool sim_check = false;
//    bool mqt_check = false;

    // # check sim808
    digitalWrite(SIM_V, HIGH); // turn on sim

    Serial.println("[+] CHECKING SIM808");
    check_timer = millis();
    while ((millis() - check_timer) <= (1000 * 60 * 2)) // 60
    {
      sim_check = global::checkSim();
      if (sim_check || NET_OK)
        break;
      delay(1000);
    }
    Serial.println((sim_check || NET_OK) ? "\t[v] SIM808 CHECK\tOK" : "\t[v] SIM808 CHECK\tFAIL");

    // # check mqtt
    Serial.println("[+] CHECKING MQTT");
    check_timer = millis();
    while ((millis() - check_timer) <= (1000 * 60 * 2)) // 60
    {
      mqt_check = global::checkBroker();
      if (mqt_check || BROKER_OK)
        break;
      delay(1000);
    }
    Serial.println((mqt_check || BROKER_OK) ? "\t[v] MQTT CHECK\t\tOK" : "\t[v] MQTT CHECK\t\tFAIL");

    
    //--------------------------  Payload Construction

    for (int k = (iterations-1); k >= 0; k--)
    {
      jsonpayload[k] = packetBuilderUniv(sim::getCsq(), sim::getCcid(), _fw_version, String(temp[k]), String(hum[k]), String(CO2[k]), String(pres[k]), String(Bat_F[k]), String(SOL_F[k]),  String(IaqAcc[k])  ,   String(AccQ[k]) , String(VOCEE[k]),String((ts+((k-iterations)*(1620)))) , String(0) );
      Serial.println("[d] Payload [" + String(k) + "]: " + jsonpayload[k]);
    }

    

    // ---------- Send

    for (int i = 0; i < iterations; i++)
    {

      if (tb.publish(_event_topic_name.c_str(), jsonpayload[i].c_str()))
      {
        Serial.println("[V] Payload MQTT Published #" + String(i));
      }
      else
      {
        Serial.println("[E] Payload Not Published #" + String(i));
      }
      delay(2000);
    }

    Serial.println();
    PayloadCount = 0;
  }
}

// debug functions

void DBGG(String message)
{
  if (DEBUG_FLAG)
    Serial.println(message);
}

void DBGG(unsigned char character, int base = DEC)
{
  if (DEBUG_FLAG)
    Serial.println(character, base);
}

void DBGL(String message)
{
  if (DEBUG_FLAG)
    Serial.print(message);
}

void DBGL(unsigned char character, int base = DEC)
{
  if (DEBUG_FLAG)
    Serial.print(character, base);
}

void global::enableDebug(bool status)
{
  DEBUG_FLAG = status;
}

// status functions

bool global::checkSim()
{
  // disable watchdog
  // Watchdog.disable();

  DBGG("[d] -> CHECKING SIM");
  if (sim::isSimOn())
    {SIM_OK = true;}
  else
    {SIM_OK = false;}

  if (!SIM_OK)
  {
    // reset all flags
    NET_OK = false;

    BROKER_OK = false;
    // power on
    sim::powerSim(1);
    // init modem
    modem.init();
  }
  if (!NET_OK)
  {
    // check if sim is on
    if (!sim::isSimOn())
    {
      NET_OK = false;
      BROKER_OK = false;
      return 0;
    }
    // get imei
    if (imei == "")
    {
      imei = sim::getImei();
    }
    // check sim lock
    if (!sim::isSimLocked())
    {
      if (!sim::unlockSim(pin))
      {
        return 0;
      }
    }
    DBGG("[d] IMEI: " + imei);
    DBGG("[d] CSQ: " + sim::getCsq());
    // reconnect
    if (sim::isSimRegistered()) //--------------------------------------
    {
      if (sim::getIp().indexOf(".") == 0)
      {
        // invalid IP
        DBGG("[d] invalid ip!");
        NET_OK = false;
        BROKER_OK = false;
        // re-connect
        sim::resolveApn();
      }
      else
      {
        // valid IP
        DBGG("[d] valid ip.");
        NET_OK = true;
        BROKER_OK = false;
        // sync time
        DBGG("[d] Syncronizing Time.");
        time::setNtp("time.google.com", 4);
        TimeX= time::getTime();

        // resolve topics
        _event_topic_name = global::resolveTopic(default_events_topic);
        _rpc_topic_name = global::resolveTopic(default_config_topic);
        _res_rpc_topic_name = global::resolveTopic(default_info_topic);

        return 1;
      }
    }  
    else  //------- sim not registred
    {
      // wait
      if (registration_timeout >= 10)
      {
        DBGG("[d] REGISTRATION TIMED OUT!");
        SIM_OK = false;
        NET_OK = false;
        BROKER_OK = false;
        registration_timeout = 0;
      }
      registration_timeout++;
      // re-connect
      sim::resolveApn();
      return 0;
    }
  }//---------------------------------------------------------------
  else
  {
    if (broker_connection_timeout >= 5 || packet_send_timeout >= 5)
    {
      modem.gprsDisconnect();
      SIM_OK = false;
      NET_OK = false;
      BROKER_OK = false;
      broker_connection_timeout = 0;
      packet_send_timeout = 0;
    }
  }
  return 0;
}

bool global::checkBroker()
{
  if (!NET_OK)
  {
    return 0;
  }

  // DBGG("[d] -> CHECKING BROKER");
  // mqtt::brokerConnectionStatus();
  if (mqtt::isBrokerConnected())
  {
    // all good
    // DBGG("[d] Broker already connected!");
    SIM_OK = true;
    NET_OK = true;
    BROKER_OK = true;
    return true;
  }
  else
  {
    BROKER_OK = false;
    global::checkSim();
    DBGG("[d] Connecting to broker.");
    mqtt::mqttConnect();
    if (mqtt::isBrokerConnected())
    {
      DBGG("[d] Broker connected!");
      BROKER_OK = true;
      broker_connection_timeout = 0;
    }
    else
    {
      broker_connection_timeout++;
      DBGG("[d] Broker connection timeout: " + String(broker_connection_timeout));
    }

    // if connected to broker subscribe to config
    if (BROKER_OK)
    {
      mqtt::mqttSubscribe(_rpc_topic_name);
    }
    return BROKER_OK;
  }
}

String global::resolveTopic(String topic)
{
  String buffer[3] = {topic.substring(0, topic.indexOf("/<") + 1), imei, topic.substring(topic.indexOf(">/") + 1, topic.length())};
  DBGG("[d] Resolved topic: " + buffer[0] + buffer[1] + buffer[2]);
  return buffer[0] + buffer[1] + buffer[2];
}

// sim functions

bool sim::isSimConnected()
{
  switch (modem.getRegistrationStatus())
  {
  case RegStatus::REG_DENIED:
    return 0;
  case RegStatus::REG_NO_RESULT:
    return 0;
  case RegStatus::REG_OK_HOME:
    return 1;
  case RegStatus::REG_OK_ROAMING:
    return 0;
  case RegStatus::REG_SEARCHING:
    return 0;
  case RegStatus::REG_UNKNOWN:
    return 0;
  case RegStatus::REG_UNREGISTERED:
    return 0;

  default:
    DBGG("[d] No response!");
    return 0;
  }
}

bool sim::isSimOn()
{
  if (digitalRead(SIM_STATUS))
  {
    DBGG("[d] SIM is ON");
    SIM_OK = true;
    return 1;
  }
  else
  {
    DBGG("[d] SIM is OFF");
    SIM_OK = false;
    return 0;
  }
}

String sim::getIp()
{
  String ip = modem.getLocalIP();
  DBGG("[d] IP: " + ip);
  return ip.indexOf(".") <= 0 ? "." : ip;
}

bool sim::powerSim(bool status)
{
  if (status && isSimOn() == false)
  {
    // turn on
    DBGG("[d] Turning the module ON!");
    pinMode(POWER_KEY, OUTPUT);
    digitalWrite(POWER_KEY, HIGH);
    delay(2000);
    digitalWrite(POWER_KEY, LOW);
    delay(1000);
    return 1;
  }
  else if (!status && isSimOn())
  {
    // turn off
    DBGG("[d] Turning the module OFF!");
    pinMode(POWER_KEY, OUTPUT);
    digitalWrite(POWER_KEY, HIGH);
    delay(2000);
    digitalWrite(POWER_KEY, LOW);
    delay(1000);
    return 1;
  }
  else
    return 0;
}

bool sim::resolveApn()
{

//
//if(!  sim::setApn(MA_APN[0], (const char *)"", (const char *)"")){ 
//  return 0;
//  }
// else return 1;

  
    String ISP = sim::getIsp();
    DBGG("ISP is " + ISP + " | lenght: " + String(ISP.length()));
    if(ISP.indexOf(ISP_IAM) >= 0)
    {
      DBGG("[i] APN is " + String(ISP_IAM));
      apn_timeout = 0;
      if(!sim::setApn(MA_APN[0], (const char*)"", (const char*)"")) return 0;
      else return 1;
    }
    else if(ISP.indexOf(ISP_INWI) >= 0)
    {
      DBGG("[i] APN is " + String(ISP_INWI));
      apn_timeout = 0;
      if(!sim::setApn(MA_APN[1], (const char*)"", (const char*)"")) return 0;
      else return 1;
    }
    else if(ISP.indexOf(ISP_ORANGE) >= 0)
    {
      DBGG("[i] APN is " + String(ISP_ORANGE));
      apn_timeout = 0;
      if(!sim::setApn(MA_APN[2], (const char*)"", (const char*)"")) return 0;
      else return 1;
    }
    else
    {
      if(apn_timeout >= 2)
      {
        DBGG("[i] APN is unknown.");
        DBGG("[i] Trying default APN.");
        if(!sim::setApn(MA_APN[1], "", "")) return 0;
        else return 1;
      }
      apn_timeout++;
      return 0;
    }
}

bool sim::setApn(const char *_apn, const char *_usr, const char *_pwd)
{
  return modem.gprsConnect(_apn, _usr, _pwd);
}

bool sim::unlockSim(char *pin)
{
  DBGG("[d] Testing pin: " + String(pin));
  if (modem.simUnlock(pin))
  {
    DBGG("[d] SIM is unlocked!");
    return 1;
  }
  else
  {
    DBGG("[d] SIM is not unlocked!");
    return 0;
  }
}

bool sim::isSimLocked()
{
  switch (modem.getSimStatus())
  {
  case SimStatus::SIM_LOCKED:
    DBGG("[d] Sim is locked!");
    NET_OK = false;
    BROKER_OK = false;
    return 0;
  case SimStatus::SIM_READY:
    DBGG("[d] Sim is ready!");
    return 1;
  case SimStatus::SIM_ERROR:
    DBGG("[d] Sim card error!");
    SIM_OK = false;
    NET_OK = false;
    BROKER_OK = false;
    sim::restartSim();
    return 0;
  case SimStatus::SIM_ANTITHEFT_LOCKED:
    DBGG("[d] Sim is antitheft locked!");
    return 0;

  default:
    DBGG("[d] No response!");
    SIM_OK = false;

    NET_OK = false;

    BROKER_OK = false;
    return 0;
  }
}

bool sim::restartSim()
{
  DBGG("[d] Restarting SIM.");
  return modem.restart();
}

String sim::getImei()
{
  String buffer = modem.getIMEI();
  if (buffer == "" || buffer.length() != 15)
  {
    return "";
  }
  else
    return buffer;
}

String sim::getCcid()
{
  return modem.getSimCCID();
}

String sim::getIsp()
{
  return modem.getOperator();
}

String sim::getCsq()
{
  return String(modem.getSignalQuality());
}

bool sim::isSimRegistered()
{
  if (modem.isNetworkConnected())
  {
    DBGG("[d] Sim is registered.");
    return 1;
  }
  else
  {
    DBGG("[d] Sim is not registered!");

    return 0;
  }
}

bool sim::sendSms(String number, String message)
{
  if (modem.sendSMS(number, message))
  {
    DBGG("[s] SMS: [\"" + message + "\"], was sent to: " + number);
    return 1;
  }
  else
  {
    DBGG("[s] SMS was not sent.");
    return 0;
  }
}

String reverseStr(String str)
{
  String buffer = str;
  str = "";
  for (int i = 0; i < (int)buffer.length(); i++)
    str += buffer[buffer.length() - i - 1];
  return str;
}

String genPassword(String deviceId)
{
  char result[10];
  sprintf(result, "%s%s%.2X%s", (reverseStr(deviceId.substring(5, 8))).c_str(), (reverseStr(deviceId.substring(2, 5))).c_str(), (unsigned int)(deviceId.substring(0, 2).toInt()), (deviceId.substring(8, 10)).c_str());
  String res = result;
  return res;
}

String device_type = "005";
String client_id;
String username;
String password;

// mqtt functions

bool mqtt::mqttConnect()
{
  tb.setServer(MQTT_SERVER, MQTT_PORT);
  tb.setCallback(mqttCallback);
  Serial.println("connecting to : client_id : " + client_id + "   password: " + password);
  client_id = device_type + imei;
  username = imei;
  password = genPassword(imei);
  // char* cimei = const_cast<char*>(imei.c_str());
  // bool connection = tb.connect("yassine", "yassine", "yassine");

  bool connection = tb.connect(client_id.c_str(), username.c_str(), password.c_str());
 // bool connection = tb.connect(client_id.c_str(), "", "");

  tb.setKeepAlive(MQTT_CONNECTION_KEEPALIVE);
  return connection;
}

bool mqtt::mqttSubscribe(String topic)
{
  return tb.subscribe(const_cast<char *>(topic.c_str()));
}

bool mqtt::mqttUnsubscribe(String topic)
{
  return tb.unsubscribe(const_cast<char *>(topic.c_str()));
}

void mqtt::mqttCallback(char *topic, byte *payload, unsigned int length)
{
  char p[length + 1];
  memcpy(p, payload, length);
  String raw_payload = "";
  for (unsigned int i = 0; i < length; i++)
  {
    raw_payload += p[i];
  }
  DBGG("[s] Topic: " + String(topic));
  DBGG("[s] Payload: " + raw_payload);

  String type = "";

  if (stringSplitter::getType(raw_payload, type))
  {
    if (type == "restart")
    {
      // restart
      DBGG("[d] MQTT => RESTART.");
      SIM_OK = false;
      NET_OK = false;
      BROKER_OK = false;

      INFO_FLAG = false;
      modem.restart();
      return;
    }
    else if (type == "ping")
    {
      // pong
      DBGG("[t] MQTT => PONG.");
      //  if(info::assemblePacket())
      //  {
      //   info::sendInfo("pong");
      // }
    }
    else if (type == "config")
    {
      // temporary topic variables
      String _event_topic_name_t = "";
      String _warning_topic_name_t = "";
      String _rpc_topic_name_t = "";
      String _res_rpc_topic_name_t = "";

      // Config
      DBGG("[d] MQTT => CONFIG.");
    }
    else if (type == "sms")
    {
      String _number, _message;

      DBGG("[d] MQTT => SMS.");
    }
    else
    {
      DBGG("[d] MQTT => UNKNOWN TYPE.");
    }
  }
  else
  {
    DBGG("[d] incomplete packet!");
  }
}

bool mqtt::sendPacket(String topic, String packet)
{
  return tb.publish(const_cast<char *>(topic.c_str()), const_cast<char *>(packet.c_str()));
}

bool mqtt::isBrokerConnected()
{
  return tb.connected();
}

void mqtt::brokerConnectionStatus()
{
  short mqttState = tb.state();

  if (mqttState == MQTT_CONNECTED)
    DBGG("[d] MQTT STATUS: MQTT_CONNECTED");
  else if (mqttState == MQTT_CONNECTION_TIMEOUT)
    DBGG("[d] MQTT STATUS: MQTT_CONNECTION_TIMEOUT");
  else if (mqttState == MQTT_CONNECTION_LOST)
    DBGG("[d] MQTT STATUS: MQTT_CONNECTION_LOST");
  else if (mqttState == MQTT_CONNECT_FAILED)
    DBGG("[d] MQTT STATUS: MQTT_CONNECT_FAILED");
  else if (mqttState == MQTT_DISCONNECTED)
    DBGG("[d] MQTT STATUS: MQTT_DISCONNECTED");
  else if (mqttState == MQTT_CONNECT_BAD_PROTOCOL)
    DBGG("[d] MQTT STATUS: MQTT_CONNECT_BAD_PROTOCOL");
  else if (mqttState == MQTT_CONNECT_BAD_CLIENT_ID)
    DBGG("[d] MQTT STATUS: MQTT_CONNECT_BAD_CLIENT_ID");
  else if (mqttState == MQTT_CONNECT_UNAVAILABLE)
    DBGG("[d] MQTT STATUS: MQTT_CONNECT_UNAVAILABLE");
  else if (mqttState == MQTT_CONNECT_BAD_CREDENTIALS)
    DBGG("[d] MQTT STATUS: MQTT_CONNECT_BAD_CREDENTIALS");
  else if (mqttState == MQTT_CONNECT_UNAUTHORIZED)
    DBGG("[d] MQTT STATUS: MQTT_CONNECT_UNAUTHORIZED");
}

// time functions

String time::getTime()
{
  String _dateTime = "";

  DBGG("[d] requesting time.");

  time::setNtp("time.google.com", 4);

  // get time and convert to timestamps
  _dateTime = time::convertDateTime();
  DBGG("[d] NETWORK time: " + _dateTime);
  return _dateTime;
}

bool time::setNtp(String server, int timezone)
{
  int response = modem.NTPServerSync(server, timezone);

  switch (response)
  {
  case -1:
    DBGG("[n] Invalide response!");
    return 0;
  case 1:
    DBGG("[n] NTP sync successful.");
    return 1;
  case 61:
    DBGG("[n] Network error!");
    return 0;
  case 62:
    DBGG("[n] DNS resolution error!");
    return 0;
  case 63:
    DBGG("[n] Connection error!");
    return 0;
  case 64:
    DBGG("[n] Serivce response error!");
    return 0;
  case 65:
    DBGG("[n] Service response timeout!");
    return 0;

  default:
    return 0;
  }
}

String time::convertDateTime()
{
  // split date and time
  int YY = 0, MM = 0, DD = 0, hh = 0, mm = 0, ss = 0;
  float tz = 0;

  if (!modem.getNetworkTime(&YY, &MM, &DD, &hh, &mm, &ss, &tz))
  {
    return "";
  }

  uint16_t days = time::date2days(YY, MM, DD);
  ts = time::time2ulong(days, hh, mm, ss) + 946684800;
  TimeX = ts;

  return String(ts);
}

uint16_t time::date2days(uint16_t y, uint8_t m, uint8_t d)
{
  if (y >= 2000U)
    y -= 2000U;
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(daysInMonth + i - 1);
  if (m > 2 && y % 4 == 0)
    ++days;
  return days + 365 * y + (y + 3) / 4 - 1;
}

uint32_t time::time2ulong(uint16_t days, uint8_t h, uint8_t m, uint8_t s)
{
  return ((days * 24UL + h) * 60 + m) * 60 + s;
}

// info functions

bool info::sendInfo(String type)
{
  String json_info_packet_new = "{\"type\":\"" + type + "\", \"payload\":" + json_info_packet + "}";
  return mqtt::sendPacket(_res_rpc_topic_name, json_info_packet_new);
}

// can functions




void isSIM_OFF(void)
{
  if (digitalRead(SIM_STATUS))
  {
    DBGG("[d] Turning SIM Module OFF LowPower ");

    pinMode(POWER_KEY, OUTPUT);
    digitalWrite(POWER_KEY, HIGH);
    delay(2000);
    digitalWrite(POWER_KEY, LOW);
    delay(1000);
    
    // Reset flags
    SIM_OK = false;
    NET_OK = false;
    mqt_check = false;
    sim_check = false;
  }
}

void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK)
  {
    if (iaqSensor.status < BSEC_OK)
    {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
Serial.println("[E] :  ON ERR < BSEC_OK");
 //=========================== Ckeck ERR  ********************************************************************************************************************************************
  // SIM etage ON ---------------------------------------------------------------------------
  digitalWrite(SIM_V, HIGH); // turn off sim

  // check sim & mqtt ---------------------------------------------------------------------------
  check_timer = 0;
  sim_check = false;
  mqt_check = false;

  Serial.print(" [d] Broker:  ");
  Serial.println(String(MQTT_SERVER));

  
  Serial.println("[+] CHECKING SIM808");
  check_timer = millis();
  while ((millis() - check_timer) <= (1000 * 60 * 2)) // 60
  {
    sim_check = global::checkSim();
    if (sim_check || NET_OK)
      break;
    delay(1000);
  }
  Serial.println((sim_check || NET_OK) ? "\t[v] SIM808 CHECK\tOK" : "\t[v] SIM808 CHECK\tFAIL");

  // # check mqtt ---------------------------------------------------------------------------
  Serial.println("[+] CHECKING MQTT");
  check_timer = millis();
  while ((millis() - check_timer) <= (1000 * 60 * 2)) // 60
  {
    mqt_check = global::checkBroker();
    if (mqt_check || BROKER_OK)
      break;
    delay(1000);
  }
  Serial.println((mqt_check || BROKER_OK) ? "\t[v] MQTT CHECK\t\tOK" : "\t[v] MQTT CHECK\t\tFAIL");

  // Construct payload 255 ---------------------------------------------------------------------------
  String TestPayload = packetBuilderUniv(sim::getCsq(), sim::getCcid(), _fw_version, String(-2), String(-2), String(-2), String(-2), String(-2), String(-2), String(-2), String(-2), String(-2), String(ts), String(-2));
  // publish payload ---------------------------------------------------------------------------
  if (tb.publish(_event_topic_name.c_str(), TestPayload.c_str()))
  {
    Serial.println("[V] Payload MQTT Published #TEST" );
  }
  else
  {
    Serial.println("[E] Payload Not Published #TEST" );
  }
  // turn SIM Off ---------------------------------------------------------------------------
  isSIM_OFF();
  // turn SIM etage Off ---------------------------------------------------------------------------
  digitalWrite(SIM_V, LOW); // turn off sim
Serial.println("going to sleep");
        delay(100);
        LowPower.deepSleep(Sleep_s);  //240000
        Serial.println("wake");
  //=========================== END Ckeck ERR ********************************************************************************************************************************************
    











      
    }
    else
    {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);

    }








      
    }
  

  if (iaqSensor.bme680Status != BME680_OK)
  {
    if (iaqSensor.bme680Status < BME680_OK)
    {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
Serial.println("[E] :  ON ERR < BSEC_OK");
//=========================== Ckeck ERR  ********************************************************************************************************************************************
  // SIM etage ON ---------------------------------------------------------------------------
  digitalWrite(SIM_V, HIGH); // turn off sim

  // check sim & mqtt ---------------------------------------------------------------------------
  check_timer = 0;
  sim_check = false;
  mqt_check = false;

  Serial.print(" [d] Broker:  ");
  Serial.println(String(MQTT_SERVER));

  
  Serial.println("[+] CHECKING SIM808");
  check_timer = millis();
  while ((millis() - check_timer) <= (1000 * 60 * 2)) // 60
  {
    sim_check = global::checkSim();
    if (sim_check || NET_OK)
      break;
    delay(1000);
  }
  Serial.println((sim_check || NET_OK) ? "\t[v] SIM808 CHECK\tOK" : "\t[v] SIM808 CHECK\tFAIL");

  // # check mqtt ---------------------------------------------------------------------------
  Serial.println("[+] CHECKING MQTT");
  check_timer = millis();
  while ((millis() - check_timer) <= (1000 * 60 * 2)) // 60
  {
    mqt_check = global::checkBroker();
    if (mqt_check || BROKER_OK)
      break;
    delay(1000);
  }
  Serial.println((mqt_check || BROKER_OK) ? "\t[v] MQTT CHECK\t\tOK" : "\t[v] MQTT CHECK\t\tFAIL");

  // Construct payload 255 ---------------------------------------------------------------------------
  String TestPayload = packetBuilderUniv(sim::getCsq(), sim::getCcid(), _fw_version, String(-2), String(-2), String(-2), String(-2), String(-2), String(-2), String(-2), String(-2), String(-2), String(ts), String(-2));
  // publish payload ---------------------------------------------------------------------------
  if (tb.publish(_event_topic_name.c_str(), TestPayload.c_str()))
  {
    Serial.println("[V] Payload MQTT Published #TEST" );
  }
  else
  {
    Serial.println("[E] Payload Not Published #TEST" );
  }
  // turn SIM Off ---------------------------------------------------------------------------
  isSIM_OFF();
  // turn SIM etage Off ---------------------------------------------------------------------------
  digitalWrite(SIM_V, LOW); // turn off sim
Serial.println("going to sleep");
        delay(100);
        LowPower.deepSleep(Sleep_s);  //240000
        Serial.println("wake");
  //=========================== END Ckeck ERR ********************************************************************************************************************************************
    }





      
    }
    else
    {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);

    }





      
    }
  


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}
