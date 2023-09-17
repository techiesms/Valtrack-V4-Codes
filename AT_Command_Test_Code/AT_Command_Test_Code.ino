

// #include "main.h"
// UART1 TX-----0
// UART1 RX-----1
// ANALOG IN----2
// INT1---------3
// TPS-ENABLE---4
// IIC-DATA-----5
// IIC-CLOCK----6
// PWRKEY-------7
// LED-SIGNAL---8
// SWITCH-SW2---9
// GSM-ENABLE---10
// USB DN-------18
// USB_DP-------19
// UART0 RX-----20
// UART0 TX-----21


#define TINY_GSM_MODEM_SIM7600

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1


// Increase RX buffer to capture the entire response
// Chips without internal buffering (A6/A7, ESP8266, M590)
// need enough space in the buffer for the entire response
// else data will be lost (and the http library will fail).
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 1024
#endif

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
// #define LOGGING  // <- Logging is for the HTTP library

// Range to attempt to autobaud
// NOTE:  DO NOT AUTOBAUD in production code.  Once you've established
// communication, set a fixed baud rate using modem.setBaud(#).
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet
// These defines are only for this example; they are not needed in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]      = "www";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
const char wifiSSID[] = "SSID";
const char wifiPass[] = "password";

// Server details
const char server[]   = "xyz.in";
const char resource[] = "";
const int  port       = 80;

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

TinyGsmClient client(modem);
HttpClient    http(client, server, port);



/////////////////////////////////////////////////////////////////////////////////////////////

// PWRKEY 7
// GSM ENABLE 10
// LED SIGNAL 8
// TPS ENABLE 4 //OR CHG IN
// INT1 3
// ANALOG IN 2
// IIC DATA 5
// IIC CLOCK 6
#define GPIO_IIC_DATA   5
#define GPIO_IIC_CLOCK  6
#define GPIO_PWRKEY     7
#define GPIO_GSM_ENABLE 10
#define GPIO_TPS_ENABLE 4
#define GPIO_INT1       3
#define GPIO_SOS        9
#define GPIO_CHG_IN     4
#define GPIO_LED_SIGNAL 8

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(3, GPIO_LED_SIGNAL, NEO_GRB + NEO_KHZ800);

#define BATTERY_LED  0
#define NETWORK_LED  1
#define LOCATION_LED 2

#define RED   0
#define GREEN 1
#define BLUE  2

#define BRIGHTNESS 64
void UpdateLED(int LED, int Color, int Brightness )
{

  switch (Color)
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
}
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
void InitLED(void)
{
  // pixels.begin(); // This initializes the NeoPixel library.

  UpdateLED(BATTERY_LED, RED, BRIGHTNESS);
  UpdateLED(NETWORK_LED, GREEN, BRIGHTNESS);
  UpdateLED(LOCATION_LED, BLUE, BRIGHTNESS);
}
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

#define ACCLEROMETER_I2C_ADDRESS 0x19  //LIS3D

uint8_t I2C_RdReg(uint8_t RegisterAddress)
{

  Wire.beginTransmission(ACCLEROMETER_I2C_ADDRESS);
  Wire.write(RegisterAddress);
  Wire.endTransmission();
  Wire.requestFrom(ACCLEROMETER_I2C_ADDRESS, 1);
  delay(2);

  return  (uint8_t)Wire.read();
}
void I2C_WrReg(uint8_t RegisterAddress, uint8_t Data)
{
  Wire.beginTransmission(ACCLEROMETER_I2C_ADDRESS);
  Wire.write(RegisterAddress);
  Wire.write(Data);
  Wire.endTransmission();
}
void InitAccelerometer(void)
{
  uint8_t VALREAD = 0;

  Wire.begin(GPIO_IIC_DATA, GPIO_IIC_CLOCK);
  VALREAD = I2C_RdReg(0x26);

  VALREAD = I2C_RdReg(0x0F);//VALREAD = I2C_RdReg(0x0D);
  Serial.print("Motion Sensor = ");
  if (VALREAD == 0x33)
  {
    Serial.println("LIS3DH Found");
  }
  else
  {
    Serial.println("LIS3DH Not Found");
  }

  I2C_WrReg(REG_CTRL_REG1, 0x57);
  I2C_WrReg(REG_CTRL_REG4, 0x08);

  delay(200);
  //  VALREAD = I2C_RdReg(REG_CTRL_REG1);
  I2C_WrReg(REG_CTRL_REG2, 0x05);
  I2C_WrReg(REG_CTRL_REG3, 0x40);//    I2C_WrReg(MMA8652_CTRL_REG3, 0x39);

  I2C_WrReg(REG_CTRL_REG5, 0x08);
  // VALREAD = I2C_RdReg(REG_CTRL_REG5);
  I2C_WrReg(REG_CTRL_REG6, 0x02);
  //I2C_WrReg(REG_CTRL_REG6, 0xFF);
  I2C_WrReg(REG_INT1_THS, 0x18);
  I2C_WrReg(REG_INT1_DURATION, 0x00);
  I2C_WrReg(REG_INT1_CFG, 0x2A);

  for (uint8_t i = 0x07; i <= 0x3F; i++)
  {
    VALREAD = I2C_RdReg(i);
  }

}
void setup() {

  delay(5000);
  InitGPIO();
  InitUART0();
  InitUART1();
  EnableGSM();
  InitLED();
  InitAccelerometer();

  InitGSM();



  Serial.println("Wait...");

  Serial.println("Wait...1");
  // Set GSM module baud rate
  //TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  //SerialAT.begin(115200,SERIAL_8N1,1,0);
  delay(6000);
  Serial.println("Hello\r\n");
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


  modemInfo = modem.getSignalQuality();
  SerialMon.print("Signal Quality: ");
  SerialMon.println(modemInfo);
  //xTaskCreate(ADCTask, "ADCTask", 2048, NULL, 10, NULL);
  // xTaskCreate(StartTimerTask, "StartTimerTask", 4096, NULL, 10, NULL);
  // xTaskCreate(StartMainTask, "StartMainTask", 8192, NULL, 10, NULL); //TIMER_TASK_STACK_SIZE

  //thisModem().sendAT(GF("+IPR="), baud);
  modem.sendAT(GF("+IPR?"));
  modem.waitResponse(10000, "+IPR");


}


void loop() {

  if (SerialAT.available()) {
    int inByte = SerialAT.read();
    SerialMon.write(inByte);
    }

    // read from port 0, send to port 1:
    if (SerialMon.available()) {
    int inByte = SerialMon.read();
    SerialAT.write(inByte);
    }

}
