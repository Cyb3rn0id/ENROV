// ENROV - ESP NOW Remote Operated Vehicle by @cyb3rn0id
// Variables and structures used on the remote command

// DEBUG
//#define DEBUG_ESPNOW_TX // for debugging ESP-Now transmission procedures only
//#define DEBUG_ESPNOW_RX // for debugging ESP-Now receiving procedures only
//#define DEBUG // debugs other than previous two

// PINS
#define STICK_X     A1  // analog joystick - horizontal trimmer
#define STICK_Y     A0  // analog joystick - vertical trimmer
#define BTN_C       4   // third button
#define BTN_B       5   // second button
#define BTN_A       6   // first button
#define BTN_STICK   7   // analog joystick - button (stick press)
#define DISP_RESET  8   // display reset (can be marked as RST or RES)
#define DISP_DC     9   // display data/command aka register select (can be marked as RS, CD, DC or A0)
#define DISP_CS     10  // display chip select
#define DISP_MOSI   11  // display MOSI (aka SDA or COPI)
#define DISP_MISO   12  // display MISO (aka SDO or CIPO)
#define DISP_CLOCK  13  // display clock

// GRAPHICS
#define FONT_H 18 // font height (pixels)
#define FONT_W 12 // font width (pixels)

// ANALOG STICKS customizing
#define STICK_LR_INVERT // decomment for inverting left/right
//#define STICK_UD_INVERT // decomment for inverting up/down
#define STICK_CENTER_VAL  512 // used 10bit resolution : trimmer at center
#define STICK_MAX_VAL    1023 // as above but max value

// My custom RGB565 colors
// find them on https://github.com/newdigate/rgb565_colors
#define ILI9341_ARDUINO   0x3D57
#define ILI9341_GREY      0x8410 // 0x808080 
#define ILI9341_HARLEQUIN 0x47E0 // 0x3FFF00 

// Car MAC address
// you must know this before using another program to retrieve it
uint8_t carAddress[] = {0x34, 0x85, 0x18, 0x7B, 0xE7, 0x30};

#define CHANNEL 11 // ESP-NOW (WiFi) channel
#define BATTERYMAX100 // uncomment if you want to show percentage higher than 100

// DATA RECEIVED FROM THE CAR - global vars for multicore purposes
uint8_t   alarms=0;
uint16_t  currentL=0;
uint16_t  currentR=0;
uint16_t  maxMotorCurrent=1000;
float     batteryVoltage=0;
uint8_t   batteryPercent=0;
float     lastAlarmValue=0;
uint16_t  frontalObstacleDistance=0;

// DATA SET BY THE REMOTE - global vars for multicore/interrupt purposes
uint16_t  xVal=0;
uint16_t  yVal=0;
uint8_t   flags=0;
bool      payloadTransmitted=true;

uint32_t lastPacketReceived=0; // last time (millis) we received a packet
#define TIMEOUT   2000  // stop motors if passed more than those ms from last packet received

// flags used for updating graphical state of buttons statuses
volatile bool resetEngaged=false;
volatile bool lightsEngaged=false;
volatile bool soundEngaged=false;
volatile bool stickbuttonEngaged=false;
volatile bool soundSignal=false;

// flags used for such accessories requiring a toggle
volatile bool lightsOn=false;
volatile bool disableMotors=false;

// MULTICORE STUFF
// references: https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/system/freertos.html
TaskHandle_t refreshMainScreen; // Task Handle function assigned to Core 0 will be referred to
SemaphoreHandle_t Semaphore; // Semaphore for accessing variables through tasks without causing guru meditations

// Structure used for sending data (data sent from here to vehicle)
struct structTX
  {
    uint8_t  flags; // accessories, see below
    uint16_t stickY; // analog stick horizontal position
    uint16_t stickX; // analog stick vertical position
  };
// bits of flags property
#define FLAGBIT_MOTORS        0 // motors enabled (1) / disabled (0)
#define FLAGBIT_LIGHTS        1 // lights on (1) / off (0)
#define FLAGBIT_SOUND         2 // sound on (1) / off (0)
#define FLAGBIT_3             3 // not used
#define FLAGBIT_4             4 // not used
#define FLAGBIT_5             5 // not used
#define FLAGBIT_6             6 // not used
#define FLAGBIT_RESET         7 // reset car alarms (1)

// Structure used for receiving data (data sent from vehicle to here)
struct structRX
  {
  uint8_t     alarms; // see below
  float       currentR; // measured current from Left motor (mA)
  float       currentL; // measured current from Right motor (mA)
  uint16_t    maxCurrent; // max value of motor current (mA)
  float       batteryVoltage; // battery voltage (mV)
  uint8_t     batteryPercent; // battery percent
  float       lastAlarmValue; // last value caused an alarm (current, or voltage => item will be determined by flag)
  uint16_t    frontalObstacleDistance; // distance in cm from frontal obstacle
  };
//bits of alarms property
#define ALARMBIT_RIGHTMOTOR   0 // car right motor over current
#define ALARMBIT_LEFTMOTOR    1 // car left motor over current
#define ALARMBIT_BATTERY      2 // car battery under voltage
#define ALARMBIT_3            3 // not used
#define ALARMBIT_4            4 // not used
#define ALARMBIT_5            5 // not used
#define ALARMBIT_6            6 // not used
#define ALARMBIT_7            7 // not used

// Structures for sending and receiving data over ESP-NOW
structTX payloadToSend;
structRX payloadReceived;