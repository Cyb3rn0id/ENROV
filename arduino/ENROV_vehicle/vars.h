// ENROV - ESP NOW Remote Operated Vehicle by @cyb3rn0id
// Variables and structures used on the vehicle

// DEBUG
//#define DEBUG_ESPNOW_TX // for debugging ESP-Now transmission procedures only
//#define DEBUG_ESPNOW_RX // for debugging ESP-Now receiving procedures only
//#define DEBUG // debugs other than previous two

// PINS
#define MOTOR_L           3 // pin to which left Motors are connected
#define MOTOR_R           2 // pin to which right Motors are connected
#define MOTORS_EN         4 // pin used to enable (high level) motors power supply
#define LIGHTS            5 // pin to which lights are connected
#define SOUND_OUT         6  // tone output
#define MOTOR_I_L         A0 // analog pin used to measure current from left motor
#define MOTOR_I_R         A1 // analog pin used to measure current from right motor
#define BATTERY_V         A2 // analog pin used to monitor car battery voltage
#define SONAR_ECHO        23 // sonar echo (aka A6)
#define SONAR_TRIG        24 // sonar trigger (aka A7)

// TIMER to SONAR interrupt
hw_timer_t *sonarTrigTimer=NULL; // timer used to fire the sonar trigger
#define TICK_RETRIGGER    4600 // 50uS*4500 = 225mS - I explain this stuff in setup and timer-relative ISRs
volatile long sonarDetectedObstacleCm=0; // distance from frontal objects detected by sonar, cm
#define DISTAVGS 3 // average on this number of readings

// LEDC peripheral
// references: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
#define MOTORCHAN_L       0 // ledc channel for left motor
#define MOTORCHAN_R       1 // ledc channel for right motor
#define PWM_FREQ_VAL      30000 // PWM frequency in Hertz (actually 30kHz)
#define PWM_RES_VAL       10 // bits of duty cycle resolution (actually 10bit => 0 to 1023)
#define SOUND_CHAN        6 // ledc channel for tone function

// Constants for Analog Stick to PWM conversion (based on the fact stick on remote and pwm here has same resolution:10bit)
#define PWM_DUTYMAX_VAL   1023 // max PWM duty cycle value
#define PWM_STOP_VAL      512 // 50% of max
#define PWM_DEADZONE_VAL  100 // analog stick dead zone (±)

// ADC TO MOTOR CURRENT CONVERSION
// ADC on car is set to 12bit ad operates to 3.3V, so every bit is: 3.3V/4096 = 0.00080566V/bit = 0.80566mV/bit
// so conversion between ADV value and voltage is V_adc=ADC_value(bits)*0.80566(mv/bit) (value returned in mv)
// on the H-Bridge current is measured thanks to a voltage drop through a 0.5 Ohm resistor
// so current drawn by a motor is: I = V/R = V/0.5 = (0.80566*ADC_value)/0.5 = (0.80566/0.5)*ADC_value = 1.611328*ADC_value
// value is returned in mA since we've used voltage value in mV
// those motors draws at least 700mA each if blocked, there are 2 motors per channel and you must consider
// some spikes at startup when motors passes from stop to moving, and some spikes when battery is under 50%
#define MOTOR_I_MULTIPLIER 1.611328 //ADC to mA conversion
#define MOTOR_I_MAX        1550.0 //mA

// ADC TO BATTERY VOLTAGE CONVERSION
// On the battery there is a voltage divider made by 2 resistors:
// R1=22k and R2=12k, so voltage measured on the ADC is V_adc = V_battery*(12/34) = V_battery*0.35294117 => V_battery = V_adc*2.8333
// but, as said above, V_adc = ADC_value*0.80566 (mv) so V_battery = ADC_value*0.80566*2.8333 = ADC_value*2.2827 (value in mv) => ADC_value*0.0022827 (value in V)
#define BATTERY_V_MULTIPLIER  0.0022827 // ADC to battery voltage conversion (V)
#define BATTERY_V_CORRECTION  0.9 // value to be added: about 0.7 due to diode + 0.2 difference I measured
#define BATTERY_V_MIN         6.2 // 3.1V per cell (used for battery undervoltage alarm)
#define BATTERY_V_MAX         8.3 // could be 4.2V per cell (transmitted to remote as is for drawing the scale and for compute battery percent)

#define CHANNEL 11 // ESP-NOW (WiFi) channel

// Remote Command MAC address
// you must know this before using another program to retrieve it
uint8_t remoteCommandAddress[] = {0x34, 0x85, 0x18, 0x7B, 0xFA, 0xC0};

bool killswitch=false; // flag used for halting motors in emergency situations
bool payloadTransmitted=true; // true => allowed to initiate another transmission
uint32_t lastPacketReceived=0; // last time (millis) we received a packet
#define TIMEOUT   800  // stop motors if passed more than those ms from last packet received
bool soundEngaged=false; // enable sound output

// MULTICORE STUFF
// references: https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/system/freertos.html
TaskHandle_t accessoryTasksHandle; // Task Handle function assigned to Core 0 will be referred to
SemaphoreHandle_t Semaphore; // Semaphore for accessing variables through tasks without causing guru meditations

// Structure used for receiving data (data sent by remote to here)
struct structRX
  {
  uint8_t  flags; // accessories, see below
  uint16_t stickY; // analog stick horizontal position
  uint16_t stickX; // analog stick vertical position
  };
// bits of flag property
#define FLAGBIT_MOTORS        0 // motors enabled (1) / disabled (0)
#define FLAGBIT_LIGHTS        1 // lights on (1) / off (0)
#define FLAGBIT_SOUND         2 // sound on (1) / off (0)
#define FLAGBIT_3             3 // not used
#define FLAGBIT_4             4 // not used
#define FLAGBIT_5             5 // not used
#define FLAGBIT_6             6 // not used
#define FLAGBIT_RESET         7 // reset car alarm (1)

// Structure used for sending data (data sent from here to remote)
struct structTX
  {
  uint8_t     alarms; // see below
  float       currentR; // measured current from Left motor (mA)
  float       currentL; // measured current from Right motor (mA)
  uint16_t    maxCurrent; // max allowed value of motor current (mA - used to set scale on remote)
  float       batteryVoltage; // battery voltage (mV)
  uint8_t     batteryPercent; // battery percent
  float       lastAlarmValue; // last value caused an alarm (current or voltage => item will be determined by flag)
  uint16_t    frontalObstacleDistance; // distance in cm from frontal obstacle
  };
// bits of alarms property
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