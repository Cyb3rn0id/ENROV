/*
ENROV - ESP NOW Remote Operated Vehicle
VEHICLE

Author: Giovanni 'CyB3rn0id' Bernardo
Project started on 15/08/2023
stable version 1.0 on 15/10/2023
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "vars.h"

// I instantiate a structure that will contain infos about the peer device (the remote command)
esp_now_peer_info_t remoteCommandInfo;

/*******************************************************************************************************************************************************************************
                                                                          INTERRUT SERVICE ROUTINES
********************************************************************************************************************************************************************************/
// interrupt on state change of sonar echo pin
void IRAM_ATTR sonarEcho_ISR() 
 {
 static long echo_start = 0; // time at which echo high pulse is arrived
 static long echo_end = 0; // time at which echo low pulse is arrived
 static uint8_t counts = 0; // I'll do an average
 static long tempDist = 0; // temporary buffer
 long echo_duration = 0; // echo lasting in microseconds
 
 switch (digitalRead(SONAR_ECHO)) // check echo pin if is high or low
    {
    case HIGH: // echo just started
      echo_end = 0;
      echo_start = micros(); // save start time
      break;

    case LOW: // echo finished
      echo_end = micros(); // save end time
      if (echo_end > echo_start) // I check that there is no micros() overflow
        {
        echo_duration = echo_end - echo_start; // echo lasting, in microseconds
        tempDist += echo_duration / 58; // distance in cm, global variable
        counts++;
        if (counts==DISTAVGS)
          {
          sonarDetectedObstacleCm = tempDist/DISTAVGS;  
          counts=0;
          tempDist=0;
          }
        }
    break;
    } // \switch echo pin
 } // \sonarEcho_ISR

// This interrupt is called every 50uS and it's used to send pulse signal on sonar trigger pin
void IRAM_ATTR sonarTrig_ISR() 
 {
 static volatile int state = 0; // actual state of pulse signal
 static volatile int trigger_time_count = 1; // countdown used to re-triggering

 // trigger must last 50 uS and we need to wait at least 225ms 
 // between subsequent triggers and this interrupt
 // happens every 50microseconds
 if (!(--trigger_time_count)) // counting up to mS defined by TICK_RETRIGGER
    {
    trigger_time_count = TICK_RETRIGGER; // reload the counter
    state = 1; // we're ready to retrigger
    }

 switch (state) 
    {
    case 0: // does nothing: I'll wait the 225ms min
      break;

    case 1: // send the pulse signal
      digitalWrite(SONAR_TRIG, HIGH); // trigger pin at high level
      state = 2; // go to the next step, see you in 50uS
      break;

    case 2:  // stop the pulse signal
    default:
      digitalWrite(SONAR_TRIG, LOW);  // trigger pin at low level
      state = 0;                   // do nothing, see you in 225mS
      break;
    } // \switch state
 } // \sonarTrig_ISR
/*******************************************************************************************************************************************************************************
                                                                        SETUP
********************************************************************************************************************************************************************************/
void setup()
  {
  Serial.begin(115200);
  #ifdef DEBUG
    delay(2000);
    Serial.print("Setup started @");
    Serial.println(esp_timer_get_time());
  #endif

  // Default leds on the Arduino Nano ESP32
  pinMode(LED_BUILTIN, OUTPUT);  // the yellow led near the USB connector
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // yellow led off
  digitalWrite(LED_RED, HIGH); // red led off : yes, the RGB leds will turn off by putting High
  digitalWrite(LED_GREEN, HIGH); // green led off 
  digitalWrite(LED_BLUE, HIGH); // blue led off

  // other IOs
  pinMode(SONAR_ECHO, INPUT);
  pinMode(SONAR_TRIG, OUTPUT);
  pinMode(LIGHTS, OUTPUT);
  lightsOFF();
  pinMode(SOUND_OUT, OUTPUT); 
  digitalWrite(SOUND_OUT, LOW);
  setToneChannel(SOUND_CHAN); // set LedC channel for Tone function

  // PWM setup
  ledcSetup(MOTORCHAN_L, PWM_FREQ_VAL, PWM_RES_VAL); // Setup ledc channel for left motors at 30kHz and 10 bit resolution
  ledcSetup(MOTORCHAN_R, PWM_FREQ_VAL, PWM_RES_VAL); // Setup ledc channel for right motors at 30kHz and 10 bit resolution
  ledcAttachPin(MOTOR_L, MOTORCHAN_L); // attach ledc channel on pin
  ledcAttachPin(MOTOR_R, MOTORCHAN_R); // attach ledc channel on pin
  parseDuty(PWM_STOP_VAL, PWM_STOP_VAL); // duty cycle at 50% at startup
    
  // other pins
  pinMode(MOTORS_EN, OUTPUT);
  digitalWrite(MOTORS_EN, LOW); // motors disabled
  analogReadResolution(12); // set ADC to 12 bits, anyway this is the default
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.disconnect();

  // I'll copy the device A address in the peer_addr field of the deviceAInfo structure
  // (we will repeat this for additional devices)
  memcpy(remoteCommandInfo.peer_addr, remoteCommandAddress, 6);
  remoteCommandInfo.channel=0; // when you put 0 here, the default SoftAP channel will be used
  remoteCommandInfo.encrypt=false; // no encryption will be used
  
  // ESP-NOW start
  if (esp_now_init() != ESP_OK) 
    {
    // Error starting ESP-NOW
    digitalWrite(LED_RED, LOW); // red led on
    #ifdef DEBUG
      Serial.println("Error starting ESP-NOW");
    #endif
    return; // exit
    }
  #ifdef DEBUG
    Serial.println("ESP-NOW Started");
  #endif
  // Add the peer device (Device A)
  // (we will repeat this for additional devices)
  if (esp_now_add_peer(&remoteCommandInfo) != ESP_OK)
    {
    // Error adding the peer device
    digitalWrite(LED_RED, LOW); // red led on
    #ifdef DEBUG
      Serial.println("Error adding Peer Device");
    #endif
    return; // exit
    }
  #ifdef DEBUG
    Serial.println("Peer Device added");
  #endif
  
  // Register esp-now callbacks
  esp_now_register_recv_cb(payloadReceivedCB);
  esp_now_register_send_cb(payloadSentCB);
  
  // multicore stuff
  Semaphore = xSemaphoreCreateMutex(); // create a mutex for variable access
  // Assign task to core 0
  // (function,name,stack size in words,input parameters,priority,&handle,core number)
  xTaskCreatePinnedToCore(accessoryTasks,"accessoryTasks",10000,NULL,0,&accessoryTasksHandle,0);

  // interruptz
  attachInterrupt(SONAR_ECHO, sonarEcho_ISR, CHANGE); // Attach interrupt to the sensor echo output (arduino input)

  // Timer Setup for interrupt every 50uS (duration of the Sonar Trigger pulse)
  // references: https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/timer.html
  // CPU here runs at 240MHz and the APB (Advanced Peripheral Bus) at 80MHz, clock for peripherals (like timers) comes from this 80MHz
  // Timer_out(S) = ticks*(prescaler/APB_clock(Hz)) = ticks*(80/80000000) = ticks*0.000001
  // Timer_out(mS) = ticks*0.001
  // Timer_out(uS) = ticks
  // we want Timer_out = 50uS so ticks=50
  sonarTrigTimer=timerBegin(0, 80, true); // (timer 0, divider=80, counter increments)
  timerAttachInterrupt(sonarTrigTimer, &sonarTrig_ISR, true); // (timer structure, function, high level (only possibility))
  timerAlarmWrite(sonarTrigTimer, 50, true); // (timer structure, count 50 ticks then fire the interrupt, autoreload)
  timerAlarmEnable(sonarTrigTimer); // enable timer alarm (interrupt)
  timerStart(sonarTrigTimer); // start the timer

  #ifdef DEBUG
    Serial.print("Setup ended @");
    Serial.println(esp_timer_get_time());
  #endif
  } // \setup

/*******************************************************************************************************************************************************************************
                                                                        MAIN
********************************************************************************************************************************************************************************/
// Loop will be executed by Core 1 by default, so no need to assign a further task for this
void loop() 
  {
  static bool blink=false; // used for led blinking when required
  //static uint8_t loopCycle=0; // loop counter, used to give some delay for data transmission
  static uint8_t alarmsFlag=0; // alarms status
  
  uint8_t analogReadings=0;
  float currentL=0;
  float currentR=0;
  float batteryV=0;
  
  blink^=1; // invert blink status for alarms (eventual)

  // read analog values ten times
  while (analogReadings<10)
    {
    currentL += analogRead(MOTOR_I_L);
    delayMicroseconds(5);
    currentR += analogRead(MOTOR_I_R);
    delayMicroseconds(5);
    batteryV += analogRead(BATTERY_V);
    delayMicroseconds(5);
    analogReadings++;
    }

  // average
  currentL/=10;
  currentR/=10;
  batteryV/=10;
  // conversions
  currentL*=MOTOR_I_MULTIPLIER;
  currentR*=MOTOR_I_MULTIPLIER;
  batteryV*=BATTERY_V_MULTIPLIER;
  batteryV+=BATTERY_V_CORRECTION;
  
  // left motor overload
  if (currentR>MOTOR_I_MAX)
    {
    killswitch=true;
    payloadToSend.lastAlarmValue=currentR;
    bitSet(alarmsFlag, ALARMBIT_RIGHTMOTOR); // set high current on right motors
    #ifdef DEBUG
      Serial.println("ALARM: Current too high on right motor");
      Serial.print("Actual value/MAX:");
      Serial.print(currentR);
      Serial.print("/");
      Serial.println(MOTOR_I_MAX);
    #endif
    }
  
  // right motor overload
  if (currentL>MOTOR_I_MAX)
    {
    killswitch=true;
    payloadToSend.lastAlarmValue=currentL;
    bitSet(alarmsFlag, ALARMBIT_LEFTMOTOR); // set high current on left motors
    #ifdef DEBUG
      Serial.println("ALARM: Current too high on left motor");
      Serial.print("Actual value/MAX:");
      Serial.print(currentL);
      Serial.print("/");
      Serial.println(MOTOR_I_MAX);
    #endif
    }

  // low battery
  if (batteryV<=BATTERY_V_MIN)
    {
    killswitch=true;
    payloadToSend.lastAlarmValue=batteryV;
    bitSet(alarmsFlag, ALARMBIT_BATTERY); // set Low Battery
    #ifdef DEBUG
      Serial.println("ALARM: Low Battery");
      Serial.print("Actual value/MIN:");
      Serial.print(batteryV);
      Serial.print("/");
      Serial.println(BATTERY_V_MIN);
    #endif
    }

  // killswitch was set => emergency stop
  if (killswitch)
    {
    motorsDisable();
    digitalWrite(LED_RED, blink); // blink red rgb led (false=low=rgb led on)
    delay(200);
    }
  else // check the motors enable flag only if there are no alarms
    {
    bitRead(payloadReceived.flags, FLAGBIT_MOTORS)?motorsEnable():motorsDisable();
    }
  
  // check flags
  if bitRead(payloadReceived.flags, FLAGBIT_RESET) 
    {
    killswitch=false;
    alarmsFlag=0;
    digitalWrite(LED_RED, HIGH); // led off
    }
  
  // check timeout
  if (millis()<lastPacketReceived) lastPacketReceived=0; // millis() rollover (I don't think this will ever happen for an RC car!!)
  if (millis()-lastPacketReceived>TIMEOUT) motorsStop(); // duty cycle at 50% (motors stopped)
  
  // prepare data to send over ESP-NOW
  payloadToSend.alarms=alarmsFlag;
  payloadToSend.currentL=currentL;
  payloadToSend.currentR=currentR;
  payloadToSend.maxCurrent=MOTOR_I_MAX; 
  payloadToSend.batteryVoltage=batteryV;
  payloadToSend.frontalObstacleDistance=sonarDetectedObstacleCm;
  
  if (batteryV<BATTERY_V_MIN) // otherwise percent will be a negative value
    {
    payloadToSend.batteryPercent=0;
    }
  else
    {
    payloadToSend.batteryPercent=(uint8_t)(((batteryV-BATTERY_V_MIN)/(BATTERY_V_MAX-BATTERY_V_MIN))*100); // calculate percent
    }
  #ifdef DEBUG_ESPNOW_TX
    Serial.println("Sending payload");
  #endif
  // send data only if previous one was sent and we've looped some times (some delay without using delay)
  //if (payloadTransmitted && loopCycle>9)
  if (payloadTransmitted)
    {
    payloadTransmitted=false;
    //loopCycle=0;
    esp_err_t espSend=esp_now_send(remoteCommandAddress, (uint8_t *)&payloadToSend, sizeof(structTX));
    #ifdef DEBUG_ESPNOW_TX
      switch(espSend)
        {
        case ESP_OK:
          Serial.println("Payload sent");
          break;
        case ESP_ERR_ESPNOW_NOT_INIT:
          Serial.println("ESPNOW not initialized");
          break;
        case ESP_ERR_ESPNOW_ARG: 
            Serial.println("Invalid argument");
            break;
        case ESP_ERR_ESPNOW_INTERNAL: 
            Serial.println("Internal error");
            break;
        case ESP_ERR_ESPNOW_NO_MEM: 
            Serial.println("Out of memory");
            break;
        case ESP_ERR_ESPNOW_NOT_FOUND:
            Serial.println("Peer device not found");
            break;
        case ESP_ERR_ESPNOW_IF: 
            Serial.println("Current WiFi interface doesn't match that of peer");
            break;
        default: // maybe for future revisions
            Serial.print("Other Error. Code: ");
            Serial.println(espSend);
            break;
        }
    #endif
    }
  #ifdef DEBUG_ESPNOW_TX
    else
      {
      Serial.println("Still waiting previous payload to be sent/cycle number");
      }
  #endif
  //loopCycle++;
  } // \loop
/*******************************************************************************************************************************************************************************
                                                                        DIGITAL SENSOR READINGS
********************************************************************************************************************************************************************************/
// this task will be executed by core 0
void accessoryTasks(void * pvParameters)
  {
  uint16_t s=0;
  randomSeed(analogRead(A3));
  while(1)  
    {
      xSemaphoreTake(Semaphore, portMAX_DELAY);
      if (soundEngaged)
        {
        soundEngaged=false;
        for (uint8_t i=1; i<16; i++)
          {
          s=random(20,3000);
          tone(SOUND_OUT, s, 50);
          }
        noTone(SOUND_OUT);
        digitalWrite(SOUND_OUT, LOW);
        }
      xSemaphoreGive(Semaphore);
    }
  }

/*******************************************************************************************************************************************************************************
                                                                        ESP-NOW FUNCTIONS
********************************************************************************************************************************************************************************/
// This is a callback function that will be automatically called at the end of data sending procedure
void payloadSentCB(const uint8_t *mac_addr, esp_now_send_status_t status) 
  {
  if (status == ESP_NOW_SEND_SUCCESS)
    {
    // code for success
    digitalWrite(LED_GREEN, LOW); // green led on
    digitalWrite(LED_BLUE, HIGH); // blue led off
    #ifdef DEBUG_ESPNOW_TX
      Serial.println("Payload sent");
    #endif
    }
  else
    {
    // code for fail
    digitalWrite(LED_GREEN, HIGH); // green led off
    digitalWrite(LED_BLUE, LOW); // blue led on
    #ifdef DEBUG_ESPNOW_TX
      Serial.println("Error sending payload");
    #endif
    }
    payloadTransmitted=true;
  } // \payloadSentCB

// This is a callback function that will be automatically called when a payload is received
// Note: the first parameter could change in the future with: const esp_now_recv_info_t *recv_info
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/migration-guides/release-5.x/5.0/networking.html
// see also the example: https://github.com/espressif/esp-idf/blob/4fc2e5cb95/examples/wifi/espnow/main/espnow_example_main.c
void payloadReceivedCB(const uint8_t *mac_addr, const uint8_t *payload, int len) 
  {
  digitalWrite(LED_BUILTIN, HIGH); // yellow led on
  // callback will store the entire payload as a byte sequence in the '*payload' array
  // using memcpy function I'll transfer data from that RAM location to my 'payloadReceived' structure
  // callback stores also the MAC address of the device: this can be useful in case
  // we wanto to connect to more devices and then understand from which one we've received data
  memcpy(&payloadReceived, payload, sizeof(payloadReceived));
  #ifdef DEBUG_ESPNOW_RX
    Serial.print(sizeof(payloadReceived));
    Serial.println(" bytes received");
    Serial.print("Flags: ");
    Serial.println(payloadReceived.flags,BIN);
    Serial.print("Analog Stick - Y: ");
    Serial.println(payloadReceived.stickY);
    Serial.print("Analog Stick - X: ");
    Serial.println(payloadReceived.stickX);
  #endif

  // lights flag
  bitRead(payloadReceived.flags, FLAGBIT_LIGHTS)?lightsON():lightsOFF();

  // sound flag
  xSemaphoreTake(Semaphore, portMAX_DELAY);
  if bitRead(payloadReceived.flags, FLAGBIT_SOUND)
    {
    soundEngaged=true;
    }
  xSemaphoreGive(Semaphore);
  
  // set the duty cycle only if there are no alarms
  if (!killswitch)
    {
    parseReceivedFlags(payloadReceived.flags);
    parseDuty(payloadReceived.stickY, payloadReceived.stickX);
    }

  digitalWrite(LED_BUILTIN, LOW); // yellow led off
  lastPacketReceived=millis(); // store time we received packet (for timeout purposes)
  } // \payloadReceivedCB

/*******************************************************************************************************************************************************************************
                                                                        MOTORS FUNCTIONS
********************************************************************************************************************************************************************************/

// disable motors
void motorsDisable(void)
  {
  digitalWrite(MOTORS_EN,LOW);  
  }

// enable motors
void motorsEnable(void)
  {
  digitalWrite(MOTORS_EN,HIGH);
  }

// simple stop motors but without disabling them
void motorsStop(void)
  {
  ledcWrite(MOTORCHAN_L, PWM_STOP_VAL);
  ledcWrite(MOTORCHAN_R, PWM_STOP_VAL); 
  }

// parse received flags
void parseReceivedFlags(uint8_t flags)
  {
  // bit 0 => motors enabled (1) or disabled (0)
  digitalWrite(MOTORS_EN,flags & 1);
  }

// parse duty cycles
void parseDuty(uint16_t Y, uint16_t X)
  {
  uint16_t dutyL=0;
  uint16_t dutyR=0;

  if (Y > PWM_STOP_VAL + PWM_DEADZONE_VAL) // car moves FORWARD => here big PWM means big speed
	  {
	  if (X < PWM_STOP_VAL - PWM_DEADZONE_VAL) // car moves forward-left
		  {
		  // left motor must have lower speed than right one
			dutyR=Y;  // right goes to full speed
		  // an X near to center (512) means big speed, an X near to 0 means lower speed
		  // so we must subtract a smaller value if X is near to center and a bigger value if X is near to 0 (stick to the left)
		  dutyL = Y - map(X,PWM_STOP_VAL-PWM_DEADZONE_VAL,0,0,PWM_STOP_VAL-PWM_DEADZONE_VAL); 
		  }
	  else if (X > PWM_STOP_VAL + PWM_DEADZONE_VAL) // car moves forward-right
		  {
		  // right motor must have lower speed than left one
		  dutyL=Y; // left goes to full speed
		  // an X near to center (512) means big speed, an X near to 1023 means lower speed
		  // so we must subtract a smaller value if X is near to center and a bigger value if X is near to 1023 (stick to the right)
		  dutyR = Y - map(X,PWM_STOP_VAL+PWM_DEADZONE_VAL,PWM_DUTYMAX_VAL,0,PWM_STOP_VAL-PWM_DEADZONE_VAL);
		  }
	else
		  {
		  // x is in the dead zone: car moves forward with both motors at same speed
		  dutyR=Y;
		  dutyL=Y;
		  }
	  }
  else if (Y < PWM_STOP_VAL - PWM_DEADZONE_VAL) // car moves BACKWARD => here low PWM means big speed
	  {
	  if (X < PWM_STOP_VAL - PWM_DEADZONE_VAL) // car moves backward-left 
		  {
		  // left motor must have lower speed than right one
		  dutyR=Y; // right goes to full speed
		  // so, since 0 is big speed and center is low speed, we must add a smaller value if X is near to center and a bigger value if X is near to 0 (stick to the left)
		  dutyL = Y + map(X,PWM_STOP_VAL-PWM_DEADZONE_VAL,0,0,PWM_STOP_VAL-PWM_DEADZONE_VAL);
		  }
	  else if (X > PWM_STOP_VAL + PWM_DEADZONE_VAL) // car moves backward-right 
		  {
		  // right motor must have lower speed than left one
		  dutyL=Y; // left one goes to full speed
		  // since 0 is big speed and center is low speed, we must add a smaller value if X is near to center and a bigger value if X is near to 1023 (stick to the right)
		  dutyR = Y + map(X,PWM_STOP_VAL+PWM_DEADZONE_VAL,PWM_DUTYMAX_VAL,0,PWM_STOP_VAL-PWM_DEADZONE_VAL);
		  }
	  else
		  {
		  // x is in the dead zone : car moves backward with both motors at same speed
		  dutyR=Y;
		  dutyL=Y;
		  }
	  }
  else // Y is in the dead zone: if stick moved along X, motors in opposite direction
	  {
	  if (X > PWM_STOP_VAL + PWM_DEADZONE_VAL) // rotation clockwise
      {
      dutyR=X;
      dutyL=PWM_DUTYMAX_VAL-X;
      }
    else if (X < PWM_STOP_VAL - PWM_DEADZONE_VAL) // rotation counter-clockwise
      {
      dutyR=X;
      dutyL=PWM_DUTYMAX_VAL-X;
      }
    else // X is in the dead zone
      {
      dutyR=PWM_STOP_VAL;
	    dutyL=PWM_STOP_VAL;
      }
    }
  // set motors PWM
  ledcWrite(MOTORCHAN_L, dutyL);
  ledcWrite(MOTORCHAN_R, dutyR);
  }

void lightsOFF(void)
  {
  digitalWrite(LIGHTS, LOW);
  }

void lightsON(void)
  {
  digitalWrite(LIGHTS, HIGH);
  }