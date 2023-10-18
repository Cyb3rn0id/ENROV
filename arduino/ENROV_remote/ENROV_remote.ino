/*
ENROV - ESP NOW Remote Operated Vehicle
REMOTE COMMAND

Author: Giovanni 'CyB3rn0id' Bernardo
Project started on 15/08/2023
stable version 1.0 on 15/10/2023

This code make use of Adafruit Graphic Library:
since this library is great and is distributed for free
please consider buying something from Adafruit
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "bitmaps.h"
#include "vars.h"

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, DISP_DC, DISP_CS, DISP_RESET);

// I instantiate a structure that will contain infos about the peer device (the vehicle)
esp_now_peer_info_t carInfo;

/*******************************************************************************************************************************************************************************
                                                                        INTERRUPTS
********************************************************************************************************************************************************************************/
void IRAM_ATTR buttonStick_ISR()
  {
  static long lastISR;
  if ((millis()-lastISR>500) && (!digitalRead(BTN_STICK)))
    {
    stickbuttonEngaged=true;
    disableMotors^=1;
    }
  lastISR=millis();
  }

void IRAM_ATTR buttonLights_ISR()
  {
  static long lastISR;
  if ((millis()-lastISR>500) && (!digitalRead(BTN_A)))
    {
    lightsEngaged=true;
    lightsOn^=1;
    }
  lastISR=millis();
  }

void IRAM_ATTR buttonSound_ISR()
  {
  static long lastISR;
  if (millis()-lastISR>250)
    {
    soundEngaged=true;
    soundSignal=true;
    }
  lastISR=millis();
  }

void IRAM_ATTR buttonReset_ISR()
  {
  static long lastISR;
  if (millis()-lastISR>250)
    {
    resetEngaged=true;
    }
  lastISR=millis();
  }

/*******************************************************************************************************************************************************************************
                                                                        SETUP
********************************************************************************************************************************************************************************/
void setup(void)
  {
  Serial.begin(115200);
  #ifdef DEBUG
    delay(2000);
    Serial.print("Setup started @");
    Serial.println(esp_timer_get_time());
  #endif
  
  // default leds on the Arduino Nano ESP32
  pinMode(LED_BUILTIN, OUTPUT);  // the yellow led near the USB connector
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // yellow led off
  digitalWrite(LED_RED, HIGH); // red led off : yes, the RGB leds will turn off by putting High
  digitalWrite(LED_GREEN, HIGH); // green led off 
  digitalWrite(LED_BLUE, HIGH); // blue led off 
  
  // buttonz
  pinMode(BTN_STICK, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
  
  // interruptz  
  attachInterrupt(BTN_STICK, buttonStick_ISR, CHANGE);
  attachInterrupt(BTN_A, buttonLights_ISR, CHANGE);
  attachInterrupt(BTN_B, buttonSound_ISR, FALLING);
  attachInterrupt(BTN_C, buttonReset_ISR, FALLING);
  
  // set ADC to 10 bits, default is 12
  analogReadResolution(10); 
  
  // WiFi mode for ESP-NOW is Station
  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.disconnect();

  // initial payload values
  payloadToSend.flags=0;
  payloadToSend.stickX=STICK_CENTER_VAL;
  payloadToSend.stickY=STICK_CENTER_VAL;
  
  // start display
  tft.begin();
  tft.setRotation(3); // portrait having display pinheader on the left
  
  // show fixed stuff on display
  displayFixedStuff();
  
  // I'll copy the car MAC address in the peer_addr field of the deviceBInfo structure
  memcpy(carInfo.peer_addr, carAddress, 6);
  carInfo.channel=0; // when you put 0 here, the default SoftAP channel will be used
  carInfo.encrypt=false; // no encryption will be used
  
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
  // Add the peer device (the car)
  if (esp_now_add_peer(&carInfo) != ESP_OK)
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
  esp_now_register_recv_cb(payloadReceivedCB); // Register callback on payload received
  esp_now_register_send_cb(payloadSentCB); // Register callback on payload sendt
  
  // multicore stuff
  Semaphore = xSemaphoreCreateMutex(); // create a mutex for variable access
  // Assign task to core 0
  // (function,name,stack size in words,input parameters,priority,&handle,core number)
  xTaskCreatePinnedToCore(displayRefresh,"values",10000,NULL,0,&refreshMainScreen,0);
  
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
  static uint8_t loopCycle=0; // loop counter, used to give some delay for data transmission
  
  int8_t analogReadings=0;
  uint16_t xV=0;
  uint16_t yV=0;
  uint8_t flagsV=1;

  // read the joystick 4 times
  while (analogReadings<4)
    {
    xV+=analogRead(STICK_X);
    delayMicroseconds(5);
    yV+=analogRead(STICK_Y);
    delayMicroseconds(5);
    analogReadings++;
    }

  //rightshift by 2 => division by 2^2 (4)
  xV>>=2;
  yV>>=2;
  
  #ifdef STICK_UD_INVERT
    yV=abs(STICK_MAX_VAL-yV);
  #endif

  #ifdef STICK_LR_INVERT
    xV=abs(STICK_MAX_VAL-xV);
  #endif
  
  // report to center if low difference from center. only for aesthetical purposes on display
  if ((xV<STICK_CENTER_VAL+30) && (xV>STICK_CENTER_VAL-30)) xV=STICK_CENTER_VAL;
  if ((yV<STICK_CENTER_VAL+30) && (yV>STICK_CENTER_VAL-30)) yV=STICK_CENTER_VAL;

  // update global variables for graphic procedures
  xSemaphoreTake(Semaphore, portMAX_DELAY);
  xVal=xV;
  yVal=yV;
  flags=flagsV;
  xSemaphoreGive(Semaphore);

  // set/reset flags
  bitWrite(flagsV, FLAGBIT_MOTORS, !disableMotors);
  bitWrite(flagsV, FLAGBIT_LIGHTS, lightsOn);
  bitWrite(flagsV, FLAGBIT_SOUND, soundSignal);
  bitWrite(flagsV, FLAGBIT_RESET, resetEngaged);
  
  // Preparing data to sent to car
  payloadToSend.flags=flagsV;
  payloadToSend.stickY=yV;
  payloadToSend.stickX=xV;

  // Sending the payload
  #ifdef DEBUG_ESPNOW_TX
    Serial.println("Sending payload");
  #endif
  // send data only if previous one was sent and we've looped some times (some delay without using delay)
  if (payloadTransmitted && loopCycle>5)
    {
    // remember ESP-NOW can send max 250bytes
    payloadTransmitted=false; // reset transmission flag
    loopCycle=0;
    soundSignal=false;
    esp_err_t espSend=esp_now_send(carAddress, (uint8_t *)&payloadToSend, sizeof(structTX));
    #ifdef DEBUG_ESPNOW_TX
      switch(espSend) // this will always return ok even if the receiver fails to receive, this gives only errors about the sending procedure
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
        default: // maybe for future revisions?
          Serial.print("Other Error. Code: ");
          Serial.println(espSend);
          break;
        }
      #endif
    }
  #ifdef DEBUG_ESPNOW_TX
    else
      {
      Serial.println("Still waiting previous payload to be sent");
      }
  #endif
  loopCycle++;
  } // \loop


/*******************************************************************************************************************************************************************************
                                                                        ESP-NOW FUNCTIONS
********************************************************************************************************************************************************************************/
// This is a callback function that will be automatically called at the end of data sending procedure
void payloadSentCB(const uint8_t *mac_addr, esp_now_send_status_t status) 
  {
  // status will be only ok (peer received) or fail (peer not received)
  // but with no info about the reason for fail
  if (status == ESP_NOW_SEND_SUCCESS)
    {
    // code for success
    digitalWrite(LED_GREEN, LOW); // green led on
    digitalWrite(LED_BLUE, HIGH); // blue led off
    #ifdef DEBUG_ESPNOW_TX
    Serial.println("Payload sent");
    #endif
    }
  else // ESP_NOW_SEND_FAIL
    {
    // code for fail
    digitalWrite(LED_GREEN, HIGH); // green led off
    digitalWrite(LED_BLUE, LOW); // blue led on
    #ifdef DEBUG_ESPNOW_TX
    Serial.println("Error sending payload");
    #endif
    }
  payloadTransmitted=true; // this flag will allow sending new data in the loop
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
  // copy payload values in global variables
  alarms=payloadReceived.alarms;
  currentL=payloadReceived.currentL;
  currentR=payloadReceived.currentR;
  maxMotorCurrent=payloadReceived.maxCurrent;
  batteryVoltage=payloadReceived.batteryVoltage;
  lastAlarmValue=payloadReceived.lastAlarmValue;
  frontalObstacleDistance=payloadReceived.frontalObstacleDistance;
  batteryPercent=payloadReceived.batteryPercent;
  #ifdef DEBUG_ESPNOW_RX
    Serial.print(sizeof(payloadReceived));
    Serial.println(" bytes received");
    Serial.print("Alarms: ");
    Serial.println(alarms,BIN);
    Serial.print("Current from LEFT motor: ");
    Serial.println(currentL);
    Serial.print("Current from RIGHT motor: ");
    Serial.println(currentR);
    Serial.print("Maximum allowed current: ");
    Serial.println(maxMotorCurrent);
    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage);
    Serial.print("Battery Percent: ");
    Serial.println(batteryPercent);
    Serial.print("Last Alarm value: ");
    Serial.println(lastAlarmValue);
    Serial.print("Frontal obstacle at ");
    Serial.print(frontalObstacleDistance)
    Serial.println("cm");
  #endif
  #ifdef BATTERYMAX100
    if (batteryPercent>100) batteryPercent=100;
  #endif
  digitalWrite(LED_BUILTIN, LOW); // yellow led off
  lastPacketReceived=millis();
  } // \payloadReceivedCB

/*******************************************************************************************************************************************************************************
                                                                        DISPLAY FUNCTIONS
********************************************************************************************************************************************************************************/
// fixed stuff on display will be printed only once
void displayFixedStuff(void)
  {
  // paint screen in black
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
 
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.print("Y  : ");
  tft.setCursor(0, FONT_H);
  tft.print("X  : ");
  tft.setCursor(0, FONT_H*2);
  tft.print("IR : ");
  tft.setCursor(0, FONT_H*3);
  tft.print("IL : ");
  tft.setCursor(0, FONT_H*4);
  tft.print("BAT: ");
  tft.setCursor(0, FONT_H*5);
  tft.print("OBS: ");
  tft.setCursor(0, FONT_H*6);
  tft.print("ALM: ");
 
  // writings in analog instruments
  tft.setTextSize(1);
  tft.setCursor(259, 36);
  tft.print("RIGHT");
  tft.setCursor(262, 118);
  tft.print("LEFT");
  tft.setCursor(253, 196);
  tft.print("BATTERY");

  // outer rectangle for distance meter
  tft.drawRect(209, 0, 20, 240, ILI9341_GREY);

  // tft.drawBitmap(X, Y, ARRAY, WIDTH, HEIGHT, COLOR RGB565);
  tft.drawBitmap(10, 171, bmp_ArduinoLogo, 100, 47, ILI9341_ARDUINO);
  
  // my name (please, follow me on Instagram ;))
  tft.setTextSize(2);
  tft.setCursor(0, 240-FONT_H);
  tft.setTextColor(ILI9341_ORANGE,ILI9341_BLACK);
  tft.print("@CYB3RN0ID");
  }


// this task will be executed by Core 0
void displayRefresh(void * pvParameters)
  {
  static uint16_t dx,dy;
  while (1) // code here must be looped as in the loop!
    {
    char buffer[10]; // buffer used for "sprintf-ing" stuff
    // accessing global variables
    xSemaphoreTake(Semaphore, portMAX_DELAY);
    uint16_t y=yVal;
    uint16_t x=xVal;
    float cr=currentR;
    float cl=currentL;
    uint8_t alms=alarms;
    uint16_t cmax=maxMotorCurrent;
    float batVolt=batteryVoltage;
    uint8_t batPerc=batteryPercent;
    float lastAlarm=lastAlarmValue;
    xSemaphoreGive(Semaphore);

    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    // joystick values reprinted only if different than previous plus/minus 5 for avoiding flickering
    if ((y<=(dy-5)) || (y>=(dy+5)))
      {
      tft.setCursor(FONT_W*5, 0);
      sprintf(buffer, "%04d", y);
      tft.print(buffer);
      dy=y;
      }
    if ((x<=(dx-5)) || (x>=(dx+5)))
      {
      tft.setCursor(FONT_W*5, FONT_H);
      sprintf(buffer, "%04d", x);
      tft.print(buffer);
      dx=x;
      }

    // right car motor current
    tft.setCursor(FONT_W*5, FONT_H*2);
    float vts=0; // value to show
    bool isBig=false; // to select if show mA or A
    if bitRead(alms,ALARMBIT_RIGHTMOTOR)
      {
      vts=lastAlarm;
      tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
      }
    else
      {
      vts=cr;
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      }
    if (vts>=1000)
      {
      vts/=1000;
      isBig=true;
      }
    if (isBig)
      {
      sprintf(buffer, "%.2f", vts); // 2 decimals
      tft.print(buffer);
      tft.print("A ");
      }
    else
      {
      sprintf(buffer, "%.0f", vts); // no decimals
      tft.print(buffer);
      tft.print("mA  ");
      }
    if (vts<10.0) tft.print(" "); // additional space for erasing
    
    // left car motor current
    tft.setCursor(FONT_W*5, FONT_H*3);
    isBig=false;
    if bitRead(alms,ALARMBIT_LEFTMOTOR)
      {
      vts=lastAlarm;
      tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
      }
    else
      {
      vts=cl;
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      }
    if (isBig)
      {
      sprintf(buffer, "%.2f", vts); // 2 decimals
      tft.print(buffer);
      tft.print("A ");
      }
    else
      {
      sprintf(buffer, "%.0f", vts); // no decimals
      tft.print(buffer);
      tft.print("mA  ");
      }
    if (vts<10.0) tft.print(" "); // additional space for erasing
    
    // car battery voltage
    tft.setCursor(FONT_W*5, FONT_H*4);
    if bitRead(alms,ALARMBIT_BATTERY)
      {
      sprintf(buffer, "%.2f", lastAlarm);
      tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
      }
    else
      {
      sprintf(buffer, "%.2f", batVolt);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      }
    tft.print(buffer);
    tft.print("V ");
    
    // obstacle
    tft.setCursor(FONT_W*5, FONT_H*5);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.print(frontalObstacleDistance);
    tft.print("cm  ");

    // car alarms
    tft.setCursor(FONT_W*5, FONT_H*6);
    if (alms)
      {
      tft.setTextColor(ILI9341_RED, ILI9341_BLACK); 
      tft.print("YES");
      }
    else
      {
      tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK); 
      tft.print("NO ");
      }
    
    drawMeters(cr, cl, cmax, batPerc);
    distanceMeter(frontalObstacleDistance, 50, 21, 11, 238);
    
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(1);
    tft.setCursor(266, 206);
    tft.print(batPerc);
    tft.print("%  ");

    // check buttons
    if (stickbuttonEngaged)
      {
      tft.fillCircle(134,229,8,ILI9341_GREEN);
      if(digitalRead(BTN_STICK)) stickbuttonEngaged=false;
      }
    else
      {
      tft.fillCircle(134,229,8,ILI9341_GREY);  
      }

    if (lightsEngaged)
      {
      tft.fillCircle(154,229,8,ILI9341_YELLOW);
      if(digitalRead(BTN_A)) lightsEngaged=false;
      }
    else
      {
      tft.fillCircle(154,229,8,ILI9341_GREY);  
      }

    if (soundEngaged)
      {
      tft.fillCircle(174,229,8,ILI9341_BLUE);
      if(digitalRead(BTN_B)) soundEngaged=false;
      }
    else
      {
      tft.fillCircle(174,229,8,ILI9341_GREY);  
      }
    
    if (resetEngaged)
      {
      tft.fillCircle(194,229,8,ILI9341_RED);
      if (digitalRead(BTN_C)) resetEngaged=false;
      }
    else
      {
      tft.fillCircle(194,229,8,ILI9341_GREY);  
      }

    // check toggle flags requiring icons
    if (lightsOn)
      {
      tft.drawBitmap(150, 58, bmp_ParkingLights, 55, 34, ILI9341_HARLEQUIN);  
      }
    else
      {
      tft.fillRect(150, 58, 55, 34, ILI9341_BLACK);
      }

    if (disableMotors)
      {
      tft.drawBitmap(150, 0, bmp_StopIcon, 55, 55, ILI9341_RED);  
      }
    else
      {
      tft.fillRect(150, 0, 55, 55, ILI9341_BLACK);
      }

    // check timeout for eventually stopping motors
    if (millis()<lastPacketReceived) lastPacketReceived=0; // millis() rollover (I don't think this will ever happen for an RC car!!)
    tft.setTextSize(2);
    tft.setCursor(0, FONT_H*7);
    if (millis()-lastPacketReceived>TIMEOUT)
      {
      tft.setTextColor(ILI9341_BLACK, ILI9341_RED);   
      tft.print(" TIMEOUT ");
      }
    else
      {
      // glyphs in size 2 are 12px large
      tft.fillRect(0, FONT_H*7, 9*12, FONT_H, ILI9341_BLACK);  
      }
    } // \while(1)
  }

// draw meters
void drawMeters(uint16_t cr, uint16_t cl, uint16_t cmax, uint8_t batPerc)
  {
  // ringMeter(value, vmin, vmax, x, y, radius, color scheme)
  ringMeter(cr, 0, cmax, 233, 1, 40, 4);
  ringMeter(cl, 0, cmax, 233, 83, 40, 4);
  ringMeter(batPerc, 0, 100, 233, 164, 40, 5);
  }


// draw the vertical bar representing distance from frontal obstacle
void distanceMeter(uint16_t value, uint16_t max, uint16_t yellowZone, uint16_t redZone, uint16_t barLength)
  {
  // value must be scaled in the range 0 - barLength
  // 2 rectangles will be painted: empty (black) on top and colored (green) on bottom
  // greater the distance from an obstacle, greater the colored bar
  if (value>max) value=max;
  uint16_t height_g=map(value,0,max,0,barLength); // height of the green bar
  uint16_t height_b=238-height_g; // height of blank bar
  yellowZone=map(yellowZone,0,max,0,barLength);
  redZone=map(redZone,0,max,0,barLength);
  // paint the black bar
  if (height_b>0) tft.fillRect(210, 1, 18, height_b, ILI9341_BLACK);  
  // paint the green bar (but can be yellow or red too!)
  uint16_t barColor=ILI9341_GREEN;
  if (height_g<yellowZone) barColor=ILI9341_YELLOW;
  if (height_g<redZone) barColor=ILI9341_RED;
  tft.fillRect(210, 1+height_b, 18, height_g, barColor);
  }


// Draw the dial on the screen
// taken from https://www.instructables.com/Arduino-analogue-ring-meter-on-colour-TFT-display/
void ringMeter(int value, int vmin, int vmax, int x, int y, int r, uint8_t scheme)
  {
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
  x += r; y += r;   // Calculate coords of centre of ring
  int w = r / 4;    // Width of outer ring is 1/4 of radius
  int angle = 150;  // Half the sweep angle of meter (300 degrees)
  int text_colour = 0; // To hold the text colour
  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
  uint8_t seg = 5; // Segments are 5 degrees wide = 60 segments for 300 degrees
  uint8_t inc = 5; // Draw segments every 5 degrees, increase to 10 for segmented ring

  // Draw colour blocks every inc degrees
  for (int i = -angle; i < angle; i += inc) 
    {
    // Choose colour from scheme
    int colour = 0;
    switch (scheme) 
      {
      case 0: colour = ILI9341_RED; break; // Fixed colour
      case 1: colour = ILI9341_GREEN; break; // Fixed colour
      case 2: colour = ILI9341_BLUE; break; // Fixed colour
      case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
      case 4: colour = rainbow(map(i, -angle, angle, 63, 127)); break; // Green to red (high temperature etc)
      case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
      default: colour = ILI9341_BLUE; break; // Fixed colour
      }

    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) 
      { // Fill in coloured segments with 2 triangles
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
      text_colour = colour; // Save the last colour drawn
      }
    else // Fill in blank segments
      {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, 0x742F);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, 0x742F);
      }
    }
  }

// returns 16bit RGR565 color for rainbow dial
unsigned int rainbow(uint8_t value)
  {
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red
  uint8_t R=0; // Red is the top 5 bits of a 16 bit colour value
  uint8_t G=0;// Green is the middle 6 bits
  uint8_t B=0; // Blue is the bottom 5 bits
  uint8_t quadrant=value>>5;
  switch (quadrant)
    {
    case 0:  
       R=0;
       G=2*(value%32);
       B=31;
       break;
    
    case 1:      
      R=0;
      G=63;
      B=31-(value%32);
      break;
    
    case 2:
      R=value%32;
      G=63;
      B=0;
      break;

    case 3:
      R=31;
      G=63-2*(value%32);
      B=0;
      break;
    }
  return (R<<11)|(G<<5)|B; // RGB565
  }