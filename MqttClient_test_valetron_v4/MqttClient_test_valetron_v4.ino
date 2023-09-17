
#define TINY_GSM_MODEM_SIM7600


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

#define SerialAT Serial1


String response = " ";
String res = "";
// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon



// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]      = "www";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT details
const char* broker = "io.adafruit.com";

const char* topicGPS       = "Sachin_SMS/feeds/gpsloc/csv";


#include <TinyGsmClient.h>
#include <PubSubClient.h>


#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false



TinyGsm        modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

int ledStatus = LOW;
#define GPIO_IIC_DATA   5
#define GPIO_IIC_CLOCK  6
#define GPIO_PWRKEY     7
#define GPIO_GSM_ENABLE 10
#define GPIO_TPS_ENABLE 4
#define GPIO_INT1       3
#define GPIO_SOS        9
#define GPIO_CHG_IN     4
#define GPIO_LED_SIGNAL 8
uint32_t lastReconnectAttempt = 0;
#define BATTERY_LED  0
#define NETWORK_LED  1
#define LOCATION_LED 2

#define RED   0
#define GREEN 1
#define BLUE  2
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(3, GPIO_LED_SIGNAL, NEO_GRB + NEO_KHZ800);
#define BRIGHTNESS 64
/*void UpdateLED(int LED, int Color, int Brightness )
  {

  switch(Color)
  {
    case RED :
      pixels.setPixelColor(LED, pixels.Color(Brightness, 0, 0));
    break;
    case GREEN :
      pixels.setPixelColor(LED, pixels.Color(0, Brightness, 0));
    break;
    case BLUE :
      pixels.setPixelColor(LED, pixels.Color(0, 0, Brightness));
    break;
    default:
      pixels.setPixelColor(LED, pixels.Color(0, 0, 0));
    break;
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
  }*/
void EnableGSM(void)
{
  digitalWrite(GPIO_GSM_ENABLE, 1);// 10 GSM_ENABLE &LED_ENABLE
}
void DisableGSM(void)
{
  digitalWrite(GPIO_GSM_ENABLE, 0);// 10 GSM_DISABLE & LED_DISABLE
}
void InitGPIO(void)
{
  pinMode(GPIO_PWRKEY, OUTPUT);
  pinMode(GPIO_GSM_ENABLE, OUTPUT);
}
/*
  void InitLED(void)
  {
  // pixels.begin(); // This initializes the NeoPixel library.

  UpdateLED(BATTERY_LED,RED, BRIGHTNESS);
  UpdateLED(NETWORK_LED,GREEN, BRIGHTNESS);
  UpdateLED(LOCATION_LED,BLUE, BRIGHTNESS);
  }*/
void InitGSM(void)
{
  // !!!!!!!!!!!
  digitalWrite(GPIO_PWRKEY, 0);
  digitalWrite(GPIO_PWRKEY, 1);
  delay(1000);
  digitalWrite(GPIO_PWRKEY, 0);
  // Set your reset, enable, power pins here
  // !!!!!!!!!!!
}
void InitUART0(void)
{
  Serial.begin(115200);
}
void InitUART1(void)
{
  Serial1.begin(115200, SERIAL_8N1, 1, 0);
}


#define REG_CTRL_REG1  0x20
#define  REG_CTRL_REG2  0x21
#define  REG_CTRL_REG3  0x22
#define  REG_CTRL_REG4  0x23
#define  REG_CTRL_REG5  0x24
#define  REG_CTRL_REG6  0x25
#define  REG_INT1_CFG  0x30
#define  REG_INT1_SRC  0x31
#define  REG_INT1_THS  0x32
#define  REG_INT1_DURATION  0x33



// Function responsible for establishing connection to MQTT server
boolean mqttConnect()
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  //boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect("GsmClientName", "USERNAME", "AIOKEY");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  return mqtt.connected();
}


void setup() {


  delay(5000);
  InitGPIO();
  InitUART0();
  InitUART1();
  EnableGSM();
  //  InitLED();
  InitGSM();

  // !!!!!!!!!!!
  // Set your reset, enable, power pins here
  // !!!!!!!!!!!

  SerialMon.println("Wait...");

  // Set GSM module baud rate
  // TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  SerialAT.begin(115200);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

#if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }
#endif

#if TINY_GSM_USE_WIFI
  // Wifi connection parameters must be set before waiting for the network
  SerialMon.print(F("Setting SSID/password..."));
  if (!modem.networkConnect(wifiSSID, wifiPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
#endif

#if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
  // The XBee must run the gprsConnect function BEFORE waiting for network!
  modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }
#endif

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);

  Serial.println("GPS Location starting");
  modem.sendAT(GF("+IPR?"));
  modem.waitResponse(10000, "+IPR");

  modem.sendAT("+CPIN?");
  modem.waitResponse(10000, "+CPIN");

  modem.sendAT(" ");
  modem.waitResponse(10000, "+AT");
  delay(1000);


  modem.sendAT("+CGNSSPWR=1,1");
  modem.waitResponse(10000, "+CGNSSPWR");
}

void loop()
{
  // Make sure we're still registered on the network

  get_location();
  //Serial.printf("final_loc",final_loc);
  mqtt_send_data();
}

void get_location() {

  SerialAT.println("AT+CGPSINFO\r\n");
  wait_for_response();
  delay(1000);
}
void wait_for_response()
{
  while (!SerialAT.available());
  while (SerialAT.available())
  {
    char add = SerialAT.read();
    res = res + add;
    delay(1);
  }

  //res = res.substring(17, 38);
  response = &res[0];
  int point = response.indexOf(":");
  int point2 = response.indexOf("N");
  point = point + 1;

  String trim1 = response.substring(point);
  String long_raw = trim1.substring(14, 25);
  String long_dd = long_raw.substring(1, 3);
  String long_ss = long_raw.substring(3, 11);
  String long_a1 = long_ss.substring(0, 2);
  String long_a2 = long_ss.substring(3, 8);
  String long_ssf = long_a1 + long_a2;
  int float_long_ss = long_ssf.toInt();
  float_long_ss = float_long_ss / 60;

  String long_sufix;
  long_sufix = String(float_long_ss);

  String long_final = long_dd + "." + long_sufix;
  String lat_raw = trim1.substring(1, 11);
  String lat_dd = lat_raw.substring(0, 2);
  String lat_ss = lat_raw.substring(2, 10);
  String lat_a1 = lat_ss.substring(0, 2);
  String lat_a2 = lat_ss.substring(3, 8);
  String lat_ssf = lat_a1 + lat_a2;
  int float_lat_ss = lat_ssf.toInt();
  float_lat_ss = float_lat_ss / 60;
  String lat_sufix;
  lat_sufix = String(float_lat_ss);

  if (lat_sufix.substring(4) = ! "0" || "1" || "2" || "3" || "4" || "5" || "6" || "7" || "8" || "9") {
    lat_sufix = "0" + lat_sufix;
  }
  else {
    lat_sufix = lat_sufix;
  }
  Serial.println(lat_sufix.substring(4));
  String lat_final = lat_dd + "." + lat_sufix;
  Serial.print("Recevied Data - "); Serial.println(response); // printin the String in lower character form

  Serial.println(long_final); Serial.println("......"); Serial.println(lat_final);

  String location_cod = "0," + lat_final + "," + long_final +"," + "0"; // Speed, latitude, longitude, altitude 
 
  const char* temp = &location_cod[0];
  Serial.print("loc_cod - ");
  Serial.print(temp);
  mqtt.publish(topicGPS, temp); // Sending data to MQTT server
  Serial.println("Published");
  delay(10000);

  response = " ";
  res = "";
}
void mqtt_send_data()
{
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(18000L, true)) {
      SerialMon.println(" fail");
      delay(10000);
      ESP.restart();
      return;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network re-connected");
      get_location();
    }

#if TINY_GSM_USE_GPRS
    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) {
        SerialMon.println("GPRS reconnected");
      }
    }
#endif
  }

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  mqtt.loop();
}
