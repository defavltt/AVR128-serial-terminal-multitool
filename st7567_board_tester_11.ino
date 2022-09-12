////////////////////
/*nrf24 controller*/
////////////////////
//needs fix!

#include <avr/sleep.h>

#include "RF24.h"
RF24 nrf24(PIN_PC2, PIN_PC0); //ce, cs
uint8_t nrf24addresses[][6] = {"corp1","corp2"};

#include <RDA5807.h>
RDA5807 radio;

#include "si5351.h"
#include "Wire.h"

Si5351 si5351; 
//when jumping big frequency hops it doesnt work directly, it works only when a ~120mhz frequency 
//is set before going from a higher to the lower

//-0.018 correction needed

#include <MicroNMEA.h>
char nmeaBuffer[200];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

#define RADIOLIB_GODMODE

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include "xbm.h"

U8G2_ST7567_OS12864_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ PIN_PB5, /* dc=*/ PIN_PB3, /* reset=*/ PIN_PB4);

#include <RadioLib.h>

SX1276 rfm95 = new Module(PIN_PC0, PIN_PC2, PIN_PD6/*unused*/, PIN_PD7/*unused*/);
  
#define DR(port, pin) !((port >> pin) & 0b00000001) //fast pin reading

#define button0(state) if(DR(PORTD.IN, 0) == HIGH && DR(PORTD.IN, 0) != state) { statebuffer = 1; } else { statebuffer = 0; }  state = DR(PORTD.IN, 0);
#define button1(state) if(DR(PORTD.IN, 1) == HIGH && DR(PORTD.IN, 1) != state) { statebuffer = 1; } else { statebuffer = 0; }  state = DR(PORTD.IN, 1);
#define button2(state) if(DR(PORTD.IN, 2) == HIGH && DR(PORTD.IN, 2) != state) { statebuffer = 1; } else { statebuffer = 0; }  state = DR(PORTD.IN, 2);
#define button3(state) if(DR(PORTD.IN, 3) == HIGH && DR(PORTD.IN, 3) != state) { statebuffer = 1; } else { statebuffer = 0; }  state = DR(PORTD.IN, 3);
#define button4(state) if(DR(PORTD.IN, 4) == HIGH && DR(PORTD.IN, 4) != state) { statebuffer = 1; } else { statebuffer = 0; }  state = DR(PORTD.IN, 4);

#define hchars 21 //max horizontal characters on screen
#define vchars 8 //max vertical characters on screen
#define vcharsterminal 7 //max vertical characters on terminal screen

#define terminaltextfieldlength hchars*3

//#define screenbuffsize (128*64)/8

#define menuoptionsnumber 23 //(mark-A)
#define macrosnumber 17
/*(mark-B)*/
#define rfm95configoptionsnumber 6
#define serialconfigoptionsnumber 4
#define rda5807radiooptionsnumber 4
#define si5351optionsnumber 3

#define serialportswap 1 //1 - swapped

#if serialportswap == 1
HardwareSerial *port1 = &Serial4;
HardwareSerial *port2 = &Serial2;
#else
HardwareSerial *port1 = &Serial2;
HardwareSerial *port2 = &Serial4;
#endif

bool statebuffer;

bool keystate[5];

bool b0;
bool b1;
bool b2;
bool b3;
bool b4;

bool left;
bool right;
bool up;
bool down;
bool enter;

bool menubutton;

bool globalbuttons;

bool updatescreen = 1;

uint8_t textinputbuffer[hchars*vcharsterminal];
uint8_t textinputbufferptr;
uint8_t lastlinenumber;
bool terminalscrollingstarted;

uint8_t terminalfieldbuffer[terminaltextfieldlength];
uint8_t terminalfieldptr;
bool halfterminal;
bool terminalkeybenable;

bool serialspeedtoggle = 1; //1 - 115200, 0 - 9600

uint32_t serialtimeoutmillis;
uint8_t serialrefreshspeed;
bool serialconcentrate;

bool serialswaptoggle;

uint8_t lineselect;
uint8_t lastlineselect;
float lineanim;
bool menuanimdirection;
uint8_t optionselect;
uint8_t optionwindow;
uint8_t lastoptionwindow;
float windowanim;
uint8_t menulength[] = { //(mark-C)
  menuoptionsnumber,
  macrosnumber,
  rfm95configoptionsnumber,
  serialconfigoptionsnumber,
  rda5807radiooptionsnumber,
  si5351optionsnumber,
};
uint8_t menulengthselect;
bool halfmenu;
bool blockmenunavigation;

uint8_t lastmenuselect;
uint8_t menuselect;

typedef enum { //(mark-D)
  desktop__ = 0,
  apps__,
  serialterminal__,
  serialmacros__,
  serialconfigurator__,
  rfm95chat__,
  rfm95receive__,
  rfm95configurator__,
  rfm95analyzer__,
  cc1101datarecplay__,
  cc1101configurator__,
  cc1101analyzer__,
  nrf24txrx__,
  nrf24control__,
  i2cscanner__,
  i2ceepromdumper__,
  rda5807radio__,
  si5351vfo__,
  nmeaparser__,
  gpsmap__,
  lcd__,
  setclock__,
  soundsettings__,
  sleepsettings__
};

bool sidepanelenable;
bool lastsidepanelenable;

float sidepanelsanim;
bool sidepanelsanimdirection;

uint8_t writemsgbuffer[hchars*3]; //21*3
uint8_t writemsgbufferptr;
uint8_t keybrowselect;
uint8_t keybcolselect;
bool keybcaps;

float uifilterbufx;
float uifilterbufy;

float letterjumpanim;

uint8_t rfm95header[4] = {255, 255, 0, 0};

//uint8_t rfm95recvbuffer[255];

uint8_t rfm95recvtext[hchars*vchars];
uint8_t rfm95recvtextptr;
uint16_t rfm95recvcount;
bool rfm95recvscrollingstarted;

float rfm95frequency = 860.0;
bool rfm95contmode = 0;
uint8_t rfm95spreadingfactor = 9;
uint8_t rfm95txbw = 5;
int8_t rfm95txpower = 15;
uint16_t rfm95preamble = 32;

uint8_t freqdigitcursor;

bool backlightenable = 1;

bool soundenable = 1;

float latitude;
float longitude;

float latitude1;
float latitude2;
float longitude1;
float longitude2;

uint8_t gpsypos;
uint8_t gpsxpos;

uint8_t lastgpsypos;
uint8_t lastgpsxpos;

uint8_t gpsmappointsx[256];
uint8_t gpsmappointsy[256];
uint8_t gpsmappointsptr;

bool gpsmaplinetoggle;

float gpsdirection;

bool screenblink;
uint32_t screenblinkmillis;

float radiofrequency = 88.1;
uint8_t radiovolume = 15;
uint8_t radioosc = CLOCK_12M;
bool radioosctype = OSCILLATOR_TYPE_REFCLK;

float vfofrequency = 50.0;
uint8_t vfodrive;
uint8_t vfopresetselect;
uint8_t vfoscan; //0 - no scan, 1 - scan up, 2 - scan down

float vfopresets[] = {
  24.574,
  3.551,
  2.325,
  3.847,
  3.583,
  4.299,
  4.312,
  5.244,
  5.431,
  5.757,
  5.815,
  6.233,
  11.576,
  11.092,
  15.104,
  8.091,
  8.036,
  7.746,
  2.320,
  6.473,
  133.600,
  133.425,
  132.375,
};

#define vfopresetsnumber sizeof(vfopresets) / 4


uint8_t nrf24controlparamselect; //0 - servo, 1 - motor, 2 - control
bool nrf24controlparamchanging; //blocks navigation while true
int16_t nrf24controlservopwmvalue;

struct nrf24controltxbuf_ {
  uint8_t servopwm;
  uint8_t motorctrl;  //0 - free, 1 - up, 2 - down, 3 - brake
  uint8_t bitctrl = 0b00000001; //(bit 0 - set:wake/clr:sleep)
} nrf24controltxbuf;

struct nrf24controlrxbuf_ {
  uint8_t battvoltage;
} nrf24controlrxbuf;






//char lettersrow1[21] = {'1','2','3','4','5','6','7','8','9','0',    '!','@','#','$','%','^','&','*','(',')'}; //(10*2)+1
//char lettersrow2[21] = {'q','w','e','r','t','y','u','i','o','p',    'Q','W','E','R','T','Y','U','I','O','P'}; //(10*2)+1
//char lettersrow3[21] = {'a','s','d','f','g','h','j','k','l',';',    'A','S','D','F','G','H','J','K','L',':'}; //(10*2)+1
//char lettersrow4[21] = {'z','x','c','v','b','n','m',',','.','/',    'Z','X','C','V','B','N','M','<','>','?'}; //(10*2)+1

uint8_t keybletters[4][10*2] = {{'1','2','3','4','5','6','7','8','9','0',    '!','@','#','$','%','^','&','*','(',')'},
                                {'q','w','e','r','t','y','u','i','o','p',    'Q','W','E','R','T','Y','U','I','O','P'},
                                {'a','s','d','f','g','h','j','k','l',';',    'A','S','D','F','G','H','J','K','L',':'},
                                {'z','x','c','v','b','n','m',',','.','/',    'Z','X','C','V','B','N','M','<','>','?'}};

String menuoptiontitles[menuoptionsnumber] = { //(mark-E)
  "serial terminal",
  "serial macros",
  "serial configurator",
  "RFM95 send",
  "RFM95 receive",
  "RFM95 configurator",
  "RFM95 analyzer",
  "CC1101 data rec/play",
  "CC1101 configurator",
  "CC1101 analyzer",
  "NRF24 tx/rx",
  "NRF24 control",
  "i2c scanner",
  "i2c eeprom dumper",
  "rda5807 radio",
  "si5351 vfo",
  "nmea parser",
  "gps map",
  "-------------system-",
  "LCD",
  "set clock",
  "sound settings",
  "sleep settings", //21
};

String serialmacros[macrosnumber] = {
  "test123",
  "corp123",
  "FREQ",
  "MODEM",
  
  "SF",
  "PWR",
  "BW",
  "CTRANS",
  
  "RTRANS",
  "PROMISC",
  "SAVE",
  "LIST",
  
  "HELP",
  "ADDR",
  "BAUD",
  "88",
  
  "89", //17
};

String rfm95configoptions[rfm95configoptionsnumber] = {
  "Frequency:",
  "Continuous TX:",
  "Spreading factor:",
  "TX bandwidth:",
  "TX power:",
  "Preamble:",
};

String serialconfigoptions[rfm95configoptionsnumber] = {
  "Swap serial ports",
  "---",
  "---",
  "---",
};

void setup() {
  Serial2.swap();
  Serial2.begin(9600); //serial1

//  Serial3.pins(PIN_PE0, PIN_PE1);
  Serial4.begin(9600); //serial2
//  Serial4.setTimeout(100); //1000 default

//  port1 = &Serial2;
//  port2 = &Serial4;

  pinMode(PIN_PD0, INPUT_PULLUP); //b0
  pinMode(PIN_PD1, INPUT_PULLUP); //b1
  pinMode(PIN_PD2, INPUT_PULLUP); //b2
  pinMode(PIN_PD3, INPUT_PULLUP); //b3
  pinMode(PIN_PD4, INPUT_PULLUP); //b4

  pinMode(PIN_PC4, OUTPUT); //speaker

//  pinMode(PIN_PC5, OUTPUT); //irda power down (set low to power down)

  pinMode(PIN_PE2, INPUT); //batt level
  pinMode(PIN_PE3, INPUT); //batt gnd (set low to measure level)

  pinMode(PIN_PC3, OUTPUT); //backlight
  digitalWrite(PIN_PC3, HIGH);

  PORTMUX.TCAROUTEA = 0b00000010; //TCA0 PORTC(0x2) //for brightness control
  TCA0.SPLIT.CTRLB = 0b00010000; //HCMP0EN
  TCA0.SPLIT.HPER = 0x00FF; //8bit
  TCA0.SPLIT.CTRLA = 0b00000001; //enable

  TCA0.SPLIT.HCMP0 = 255; //brightness

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x12_tr);
//  u8g2.setFont(u8g2_font_nokiafc22_tr);
//  u8g2.setFont(u8g2_font_oskool_tr);
  u8g2.setDrawColor(1);
  u8g2.setFlipMode(1);
  u8g2.setContrast(30);

  delay(1000);

//  rfm95.begin(); //lora mode
//  rfm95.setFrequency(860.011);
//  rfm95.setPreambleLength(32);
//  rfm95.setBandwidth(31.25);
//  rfm95.setCodingRate(8); //4/8 coding rate
//  rfm95.setSpreadingFactor(9); //512 chips
//  rfm95.setCRC(1, 0); //CCITT
//  rfm95.forceLDRO(0); //low data rate optimization off

//  tone(PIN_PC4, 1000);

  windowanim = 8; //reset animation
  lineanim = 8; //reset animation
}

void loop() {
  button0(keystate[0]) if(statebuffer) { //(up)
    b0 = 1;
    up = 1;
//    port1.println("up");
    globalbuttons = 1;
  }
  button1(keystate[1]) if(statebuffer) { //(down)
    b1 = 1;
    down = 1;
//    port1.println("down");
    globalbuttons = 1;
  }
  button2(keystate[2]) if(statebuffer) { //(left)
    b2 = 1;
    left = 1;
    globalbuttons = 1;
  }
  button3(keystate[3]) if(statebuffer) { //(right)
    b3 = 1;
    right = 1;
    globalbuttons = 1;
  }
  button4(keystate[4]) if(statebuffer) { //(enter)
    b4 = 1;
    enter = 1;
    globalbuttons = 1;
  }

  if(0 < port2->available()) { //serial port 2 keyboard event
    uint8_t serialread = port2->read();
    
    switch(serialread) {
      case 136: //backspace
        break;
      case 252: //left
        left = 1;
        break;
      case 253: //right
        right = 1;
        break;
      case 254: //up
        up = 1;
        break;
      case 255: //down
        down = 1;
        break;
      case '\n': //enter
        enter = 1;
        break;
      case 200: //esc
        if(menuselect == serialterminal__ || menuselect == serialmacros__) {
          cleardatabuffer(textinputbuffer, sizeof(textinputbuffer), textinputbufferptr);
        }
        break;
      case 201: //f1
        up = 1;
        b0 = 1;
        break;
      case 202: //f2
        down = 1;
        b1 = 1;
        break;
      case 203: //f3
        left = 1;
        b2 = 1;
        break;
      case 204: //f4
        right = 1;
        b3 = 1;
        break;
      case 205: //f5
        enter = 1;
        b4 = 1;
        break;
      case 219: //home button
        menuselect = desktop__; //go to desktop
        break;
      case 222: //menu button
        menubutton = 1;
        break;
    }

    if(!enter) { //prevents from adding a character before sending
      switch(menuselect) {
        case serialterminal__: //serial terminal
          terminaloutput(terminalfieldbuffer, sizeof(terminalfieldbuffer), serialread, terminalfieldptr); //write to text field
          break;
        case rfm95chat__: //rfm95 chat
          terminaloutput(writemsgbuffer, sizeof(writemsgbuffer), serialread, writemsgbufferptr); //write to text field
          break;
      }
    }
//    port1.println(serialread);
    globalbuttons = 1;
  }

  if(menuselect == serialterminal__ || menuselect == serialmacros__ || menuselect == nmeaparser__ || menuselect == gpsmap__) { //only in serial terminal menus
    serialconcentrate = 1; //make sure everytime, its executed at least once 
    while(serialconcentrate) { //if continuous serial data is flowing only this while loop runs, so no data is missed
      if(0 < port1->available()) { //serial port 1 input event //used only in apps requiring serial
        uint8_t serialread = port1->read();
        if(menuselect == serialterminal__ || menuselect == serialmacros__) { //serial terminal & macros
          switch(serialread) {
            case 8: //BS (backspace)
              textinputbufferptr--;
              if(textinputbufferptr == 255) {
                textinputbufferptr = 0; //cap value
              }
              break;
            case 136: //backspace
              textinputbufferptr--;
              if(textinputbufferptr == 255) {
                textinputbufferptr = 0; //cap value
              }
              break;
            case 200: //esc
              cleardatabuffer(textinputbuffer, sizeof(textinputbuffer), textinputbufferptr);
              break;
          }
          serialtoterminal(textinputbuffer, sizeof(textinputbuffer), serialread, textinputbufferptr); //display contents from serial port 1 input
        }
        if(menuselect == nmeaparser__ || menuselect == gpsmap__) { //nmea parser //gps map
          nmea.process((char)serialread);
          updatescreen = 1;
        }
        serialtimeoutmillis = millis(); //reset
      }
      serialconcentrate = 0;
      if(millis() > serialtimeoutmillis + 50) { //if serial data is coming faster than 50ms a byte, only this loop runs
        serialconcentrate = 0;
      } else {
        serialconcentrate = 1;
      }
    }
  }

  /*button logic*/
  if(!blockmenunavigation) {
    if(up) {
      optionselect--;
      lineselect--;
      if(lineselect == 255) { //when top is hit
        lineselect++; //dont move
        optionwindow--; //but start moving window
        if(optionwindow == 255) {
          optionwindow = 0; //cap value
        }
      }
      if(optionselect == 255) { //when first line is hit
        optionselect = menulength[menulengthselect]-1; //loop around
        if(halfmenu) {
          optionwindow = menulength[menulengthselect]-4;
          if(optionwindow >= 248 && optionwindow <= 255) { //bodge fix
            optionwindow = 0;
          }
          if(menulength[menulengthselect]-1 > 3) { //reset lineselect
            lineselect = 3; //limit
          } else {
            lineselect = menulength[menulengthselect]-1;
          }
        } else {
          optionwindow = menulength[menulengthselect]-8;
          if(optionwindow >= 248 && optionwindow <= 255) { //bodge fix
            optionwindow = 0;
          }
          if(menulength[menulengthselect]-1 > 7) { //reset lineselect
            lineselect = 7; //limit
          } else {
            lineselect = menulength[menulengthselect]-1;
          }
        }
      }
      menuanimdirection = 1;
    }
  
    if(down) {
      optionselect++;
      lineselect++;
      if(halfmenu) {
        if(lineselect > 3) { //when bottom is hit
          lineselect--; //dont move
          optionwindow++; //but start moving window
        }
      } else {
        if(lineselect > 7) { //when bottom is hit
          lineselect--; //dont move
          optionwindow++; //but start moving window
        }
      }
      if(optionselect > menulength[menulengthselect]-1) { //when last line is hit
        optionselect = 0; //loop around
        optionwindow = 0; //set window at beginning
        lineselect = 0; //reset lineselect
      }
      menuanimdirection = 0;
    }
  }

  if(globalbuttons) {
//    port1.print(optionselect); port1.print("  ");
//    port1.print(lineselect); port1.print("  ");
//    port1.println(optionwindow);
    toneclick();
    updatescreen = 1;
  }

  /*(mark-F)*/
  if(lastmenuselect != menuselect) { //when menus change
    optionselect = 0; //reset
    lineselect = 0;
    optionwindow = 0;
    blockmenunavigation = 0;
    switch(menuselect) {
      case apps__: //apps
        menulengthselect = 0;
        break;
      case serialmacros__: //serial macros
        menulengthselect = 1;
        break;
      case serialconfigurator__: //serial configurator
        menulengthselect = 3;
        break;
      case rfm95configurator__: //rfm95 configurator
        menulengthselect = 2;
        break;
      case rda5807radio__: //rda5807 radio
        menulengthselect = 4;
        break;
      case si5351vfo__: //si5351 vfo
        menulengthselect = 5;
        break;
    }
  }
  lastmenuselect = menuselect;

  /*menu navigation and application logic*/
  switch(menuselect) {
    /*(mark-H)*/
    case desktop__: //desktop
      if(/*b0 || b1 || b2 || b3 || */b4 || up || down || left || right || enter) {
        menuselect = apps__; //go to apps
      }
      break;
    case apps__: //apps (navigate to submenus)
      halfmenu = 0;
      halfterminal = 0;
      if(enter) {
        switch(optionselect) {
          /*(mark-G)*/
          case 0: //serial terminal
            menuselect = serialterminal__; //serial terminal
            break;
          case 1: //serial macros
            menuselect = serialmacros__; //serial macros
            halfmenu = 1;
            halfterminal = 1;
            break;
          case 2: //serial configurator
            menuselect = serialconfigurator__; //serial configurator
            break;
          case 3: //rfm95 chat
            menuselect = rfm95chat__; //rfm95 chat
            initrfm95();
            break;
          case 4: //rfm95 receive
            menuselect = rfm95receive__; //rfm95 receive
            initrfm95();
            break;
          case 5: //rfm95 configurator
            menuselect = rfm95configurator__; //rfm95 configurator
            break;
          case 11: //nrf24 control
            menuselect = nrf24control__; //nrf24 control
            initnrf24(); //init
            break;
          case 14: //rda5807 radio
            menuselect = rda5807radio__; //rda5807 radio
            radio.setup(radioosc, radioosctype); //init
            break;
          case 15: //si5351 vfo
            menuselect = si5351vfo__; //si5351 vfo
            initsi5351vfo(); //init
            break;
          case 16: //nmea parser
            menuselect = nmeaparser__; //nmea parser
            initgps();
            break;
          case 17: //gps map
            menuselect = gpsmap__; //gps map
            break;
          case 18: break; //placeholder for apps menu dividing line
          case 19: //LCD
            backlightenable = !backlightenable;
            if(backlightenable) {
              TCA0.SPLIT.CTRLA = 0b00000001; //enable
            } else {
              TCA0.SPLIT.CTRLA = 0b00000000; //disable
            }
            digitalWrite(PIN_PC3, backlightenable);
            break;
          case 21: //soundsettings
            soundenable = !soundenable;
            break;
        }
      }
      if(b2) {
        menuselect = desktop__; //go to desktop
      }
      break;
    case serialterminal__: //serial terminal
      if(!terminalkeybenable) { //while keyboard is not open in the terminal
        if(enter) {
          terminaltx(terminalfieldbuffer, sizeof(terminalfieldbuffer), terminalfieldptr, 1);
        }
        if(sidepanelenable) {
          if(b0) {
            cleardatabuffer(textinputbuffer, sizeof(textinputbuffer), textinputbufferptr);
            sidepanelenable = 0; //close sidepanel
          }
          if(b1) {
            menuselect = apps__; //go to apps
            sidepanelenable = 0; //close sidepanel
          }
          if(b2) {
            serialspeedtoggle = !serialspeedtoggle;
            changeserialspeed(serialspeedtoggle); //toggle serial speed
            sidepanelenable = 0; //close sidepanel
          }
          if(b3) { //keyboard
            terminalkeybenable = 1;
            sidepanelenable = 0; //close sidepanel
          }
          if(b4) { //cancel
            sidepanelenable = 0; //close sidepanel
          }
        } else {
           if(b3) {
             sidepanelenable = 1; //open sidepanel
           }
        }
        if(menubutton) {
          sidepanelenable = !sidepanelenable;
        }
      } else { //while in terminal keyboard
        keyboardinterface(terminalfieldbuffer, sizeof(terminalfieldbuffer), terminalfieldptr);
        if(keybrowselect == 4) { //only last row
          if(enter) {
            switch(keybcolselect) {
              case 0: //caps
                keybcaps = !keybcaps; //toggle
                break;
              case 1: //space
                addspace(terminalfieldbuffer, terminalfieldptr);
                break;
              case 2: //exit
                terminalkeybenable = 0; //exit out to terminal
                break;
              case 3: //send
                terminaltx(terminalfieldbuffer, sizeof(terminalfieldbuffer), terminalfieldptr, 0);
                break;
              case 4: //clear
                cleardatabuffer(terminalfieldbuffer, sizeof(terminalfieldbuffer), terminalfieldptr);
                break;
              case 5: //backspace
                removelastchar(terminalfieldbuffer, terminalfieldptr);
                break;
            }
          }
        }
      }
      break;
    case serialmacros__: //serial macros
      if(enter) {
        port1->write(serialmacros[optionselect].c_str(), strlen(serialmacros[optionselect].c_str())); //send serial macro
      }
      if(sidepanelenable) {
        if(b0) {
          cleardatabuffer(textinputbuffer, sizeof(textinputbuffer), textinputbufferptr);
          sidepanelenable = 0; //close sidepanel
        }
        if(b1) {
          menuselect = apps__; //go to apps
          sidepanelenable = 0; //close sidepanel
        }
        if(b2) {
          serialspeedtoggle = !serialspeedtoggle;
          changeserialspeed(serialspeedtoggle); //toggle serial speed
          sidepanelenable = 0; //close sidepanel
        }
        if(b3) {
          sidepanelenable = 0; //close sidepanel
        }
        if(b4) {
          sidepanelenable = 0; //close sidepanel
        }
      } else {
        if(b3) {
          sidepanelenable = 1; //open sidepanel
        }
      }
      if(menubutton) {
        sidepanelenable = !sidepanelenable;
      }
      break;
    case serialconfigurator__: //serial configurator
      if(sidepanelenable) {
        if(b0) {
          menuselect = apps__; //go to apps
          sidepanelenable = 0; //close sidepanel
        }
        if(b1 || b2 || b3 || b4) {
          sidepanelenable = 0; //close sidepanel
        }
      } else {
        if(enter) {
          switch(optionselect) {
            case 0:
              swapserialports();
              break;
          }
        }
        if(b3) {
          sidepanelenable = 1;
        }
      }
      break;
    case rfm95chat__: //rfm95 chat
      keyboardinterface(writemsgbuffer, sizeof(writemsgbuffer), writemsgbufferptr);
      if(keybrowselect == 4) { //only last row
        if(enter) {
          switch(keybcolselect) {
            case 0: //caps
              keybcaps = !keybcaps; //toggle
              break;
            case 1: //space
              addspace(writemsgbuffer, writemsgbufferptr);
              break;
            case 2: //exit
              menuselect = apps__; //go to apps
              break;
            case 3: //send
              rfm95addheader(writemsgbuffer, sizeof(writemsgbuffer));
              rfm95.transmit(writemsgbuffer, strlen((char*)writemsgbuffer));
              break;
            case 4: //clear
              cleardatabuffer(writemsgbuffer, sizeof(writemsgbuffer), writemsgbufferptr);
              break;
            case 5: //backspace
              removelastchar(writemsgbuffer, writemsgbufferptr);
              break;
          }
        }
      }
      break;
    case rfm95receive__: //rfm95 receive
      if(rfm95.receive(rfm95recvtext) == RADIOLIB_ERR_NONE) { //needs fixing
        port1->println((char*)rfm95recvtext);
        rfm95recvmsgs(rfm95recvtext, sizeof(rfm95recvtext), rfm95recvtextptr);
      }
      break;
    case rfm95configurator__: //rfm95 configurator
      if(sidepanelenable) {
        if(b0) { //set settings to default
          sidepanelenable = 0; //close sidepanel
        }
        if(b1) {
          menuselect = apps__; //go to apps
          sidepanelenable = 0; //close sidepanel
        }
        if(b2 || b3 || b4) {
          sidepanelenable = 0; //close sidepanel
        }
      } else {
        if(b3) {
          if(!blockmenunavigation) { //dont open panel while changing parameters
            sidepanelenable = 1; //open sidepanel
          }
        }
        if(enter) {
          blockmenunavigation = !blockmenunavigation;
        }
        if(blockmenunavigation) { //while in option
          switch(optionselect) {
            case 0: //freq
              if(left) {
                freqdigitcursor--;
                if(freqdigitcursor == 255) {
                  freqdigitcursor = 5; //loop around
                }
              }
              if(right) {
                freqdigitcursor++;
                if(freqdigitcursor > 5) {
                  freqdigitcursor = 0; //loop around
                }
              }
              if(up) {
                switch(freqdigitcursor) {
                case 0:
                  rfm95frequency += 100.0;
                  break;
                case 1:
                  rfm95frequency += 10.0;
                  break;
                case 2:
                  rfm95frequency += 1.0;
                  break;
                case 3:
                  rfm95frequency += 0.1;
                  break;
                case 4:
                  rfm95frequency += 0.01;
                  break;
                case 5:
                  rfm95frequency += 0.001;
                  break;
                }
              }
              if(down) {
                switch(freqdigitcursor) {
                case 0:
                  rfm95frequency -= 100.0;
                  break;
                case 1:
                  rfm95frequency -= 10.0;
                  break;
                case 2:
                  rfm95frequency -= 1.0;
                  break;
                case 3:
                  rfm95frequency -= 0.1;
                  break;
                case 4:
                  rfm95frequency -= 0.01;
                  break;
                case 5:
                  rfm95frequency -= 0.001;
                  break;
                }
              }
              if(globalbuttons) {
                rfm95.setFrequency(rfm95frequency);
              }
              break;
            case 1: //contmode
              blockmenunavigation = 0;
              rfm95contmode = !rfm95contmode;
              break;
            case 2: //sf
              if(up) {
                rfm95spreadingfactor++;
                if(rfm95spreadingfactor > 12) {
                  rfm95spreadingfactor = 6; //loop around
                }
              }
              if(down) {
                rfm95spreadingfactor--;
                if(rfm95spreadingfactor < 6) {
                  rfm95spreadingfactor = 12; //loop around
                }
              }
              if(globalbuttons) {
                rfm95.setSpreadingFactor(rfm95spreadingfactor);
              }
              break;
            case 3: //bw
              if(up) {
                rfm95txbw++;
                if(rfm95txbw > 9) {
                  rfm95txbw = 0; //loop around
                }
              }
              if(down) {
                rfm95txbw--;
                if(rfm95txbw == 255) {
                  rfm95txbw = 9; //loop around
                }
              }
              uint8_t bwvalue;
              switch(rfm95txbw) {
                case 0:
                  bwvalue = RADIOLIB_SX1278_BW_7_80_KHZ;
                  break;
                case 1:
                  bwvalue = RADIOLIB_SX1278_BW_10_40_KHZ;
                  break;
                case 2:
                  bwvalue = RADIOLIB_SX1278_BW_15_60_KHZ;
                  break;
                case 3:
                  bwvalue = RADIOLIB_SX1278_BW_20_80_KHZ;
                  break;
                case 4:
                  bwvalue = RADIOLIB_SX1278_BW_31_25_KHZ;
                  break;
                case 5:
                  bwvalue = RADIOLIB_SX1278_BW_41_70_KHZ;
                  break;
                case 6:
                  bwvalue = RADIOLIB_SX1278_BW_62_50_KHZ;
                  break;
                case 7:
                  bwvalue = RADIOLIB_SX1278_BW_125_00_KHZ;
                  break;
                case 8:
                  bwvalue = RADIOLIB_SX1278_BW_250_00_KHZ;
                  break;
                case 9:
                  bwvalue = RADIOLIB_SX1278_BW_500_00_KHZ;
                  break;
              }
              if(globalbuttons) {
                rfm95.SX1278::setBandwidthRaw(bwvalue);
              }
              break;
            case 4: //txpwr
              if(up) {
                rfm95txpower++;
                if(rfm95txpower > 15) {
                  rfm95txpower = -3; //loop around
                }
              }
              if(down) {
                rfm95txpower--;
                if(rfm95txpower < -3) {
                  rfm95txpower = 15; //loop around
                }
              }
              if(globalbuttons) {
                rfm95.setOutputPower(rfm95txpower);
              }
              break;
            case 5: //prmb
              if(up) {
                rfm95preamble++;
              }
              if(down) {
                rfm95preamble--;
              }
              if(left) {
                rfm95preamble -= 16;
              }
              if(right) {
                rfm95preamble += 16;
              }
              if(globalbuttons) {
                rfm95.setPreambleLength(rfm95preamble);
              }
              break;
          }
        }
      }
      break;
    case nrf24control__:
      if(sidepanelenable) { //only while sidepanel is open
        if(b0) {
          menuselect = apps__; //go to apps
          sidepanelenable = 0; //close sidepanel
        }
        if(b1 || b2 || b3 || b4) {
          sidepanelenable = 0; //close sidepanel
        }
      } else { //rest of code
        if(b3) { //open sidepanel
          if(!nrf24controlparamchanging) { //dont open panel while changing parameters
            sidepanelenable = 1; //open sidepanel
          }
        }
        if(enter) {
          nrf24controlparamchanging = !nrf24controlparamchanging; //toggle
        }
        if(nrf24controlparamchanging) {
          switch(nrf24controlparamselect) {
            /*select control parameter*/
            case 0: //servo
              if(up) {
                nrf24controlservopwmvalue += 3;
                if(nrf24controlservopwmvalue > 255) {
                  nrf24controlservopwmvalue = 255; //limit
                }
                nrf24controltxbuf.servopwm = nrf24controlservopwmvalue;
              }
              if(down) {
                nrf24controlservopwmvalue -= 3;
                if(nrf24controlservopwmvalue < 0) {
                  nrf24controlservopwmvalue = 0; //limit
                }
                nrf24controltxbuf.servopwm = nrf24controlservopwmvalue;
              }
              break;
            case 1: //motor
              if(up) {
                nrf24controltxbuf.motorctrl = 1; //forward
              }
              if(down) {
                nrf24controltxbuf.motorctrl = 2; //backward
              }
              if(left) {
                nrf24controltxbuf.motorctrl = 3; //brake
              }
              if(right) {
                nrf24controltxbuf.motorctrl = 0; //free
              }
              break;
            case 2: //control
              if(up) {
                nrf24controltxbuf.bitctrl = 0b00000001; //wake up
              }
              if(down) {
                nrf24controltxbuf.bitctrl = 0b00000000; //go to sleep
              }
              break;
          }
          if(globalbuttons) { //send data
            nrf24.stopListening();
            nrf24.write(&nrf24controltxbuf, sizeof(nrf24controltxbuf)); 
            nrf24.startListening();
          }
        } else {
          if(down) {
            nrf24controlparamselect--;
            if(nrf24controlparamselect == 255) {
              nrf24controlparamselect = 2; //loop around
            }
          }
          if(up) {
            nrf24controlparamselect++;
            if(nrf24controlparamselect > 2) {
              nrf24controlparamselect = 0; //loop around
            }
          }
        }
      }
      while (nrf24.available()) {
        nrf24.read(&nrf24controlrxbuf, sizeof(nrf24controlrxbuf));
        updatescreen = 1;
      }
      break;
    case rda5807radio__:
      if(sidepanelenable) {
        if(b0) {
          menuselect = apps__; //go to apps
          sidepanelenable = 0; //close sidepanel
        }
        if(b1 || b2 || b3 || b4) {
          sidepanelenable = 0; //close sidepanel
        }
      } else {
        if(b3) {
          if(!blockmenunavigation) { //dont open panel while changing parameters
            sidepanelenable = 1; //open sidepanel
          }
        }
        
        switch(optionselect) {
          case 0: //freq adjust
            if(enter) {
              blockmenunavigation = !blockmenunavigation;
            }
            if(blockmenunavigation) { //while in option
              if(left) {
                freqdigitcursor--;
                if(freqdigitcursor == 255) {
                  freqdigitcursor = 5; //loop around
                }
              }
              if(right) {
                freqdigitcursor++;
                if(freqdigitcursor > 5) {
                  freqdigitcursor = 0; //loop around
                }
              }
              if(up) {
                switch(freqdigitcursor) {
                case 0:
                  radiofrequency += 100.0;
                  break;
                case 1:
                  radiofrequency += 10.0;
                  break;
                case 2:
                  radiofrequency += 1.0;
                  break;
                case 3:
                  radiofrequency += 0.1;
                  break;
                case 4:
                  radiofrequency += 0.01;
                  break;
                case 5:
                  radiofrequency += 0.001;
                  break;
                }
              }
              if(down) {
                switch(freqdigitcursor) {
                case 0:
                  radiofrequency -= 100.0;
                  break;
                case 1:
                  radiofrequency -= 10.0;
                  break;
                case 2:
                  radiofrequency -= 1.0;
                  break;
                case 3:
                  radiofrequency -= 0.1;
                  break;
                case 4:
                  radiofrequency -= 0.01;
                  break;
                case 5:
                  radiofrequency -= 0.001;
                  break;
                }
              }
            }
            break;
          case 1: //volume
            if(right) {
              radiovolume++;
              if(radiovolume > 15) {
                radiovolume = 0; //loop around
              }
            }
            if(left) {
              radiovolume--;
              if(radiovolume == 255) {
                radiovolume = 15; //loop around
              }
            }
            break;
          case 2: //osc freq
            if(right) {
              radioosc++;
              if(radioosc > 7) {
                radioosc = 0; //loop around
              }
              if(radioosc == 4) radioosc++; //skip 4
            }
            if(left) {
              radioosc--;
              if(radioosc == 255) {
                radioosc = 7; //loop around
              }
              if(radioosc == 4) radioosc--; //skip 4
            }
            break;
          case 3: //osc type
            if(enter || left || right) {
              radioosctype = !radioosctype;
            }
            break;
        }
        if(optionselect == 2 || optionselect == 3) { //when changing crystal settings
          if(globalbuttons) {
//            radio.softReset();
//            delay(1);
            radio.setup(radioosc, radioosctype);
          }
        }
        if(globalbuttons) { //when changing volume or frequency
          radio.setFrequency((uint32_t)(radiofrequency*100.0));
          radio.setVolume(radiovolume);
        }
      }
      break;
    case si5351vfo__:
      if(sidepanelenable) { //only while sidepanel is open
        if(b0) {
          menuselect = apps__; //go to apps
          sidepanelenable = 0; //close sidepanel
        }
        if(b1) {
          vfoscan = 1; //scan up
          sidepanelenable = 0; //close sidepanel
        }
        if(b2) {
          vfoscan = 2; //scan down
          sidepanelenable = 0; //close sidepanel
        }
        if(b3 || b4) {
          sidepanelenable = 0; //close sidepanel
        }
      } else {
        if(b3) { //open sidepanel
          if(!blockmenunavigation) { //dont open panel while changing parameters
            sidepanelenable = 1; //open sidepanel
          }
        }
        if(enter) {
          blockmenunavigation = !blockmenunavigation;
          if(optionselect == 0) {
            vfoscan = 0; //stop scan
          }
        }
        switch(optionselect) {
        /*select options*/
          case 0: //change frequency
            if(blockmenunavigation) { //while in option
              if(left) {
                freqdigitcursor--;
                if(freqdigitcursor == 255) {
                  freqdigitcursor = 7; //loop around
                }
              }
              if(right) {
                freqdigitcursor++;
                if(freqdigitcursor > 7) {
                  freqdigitcursor = 0; //loop around
                }
              }
              if(up) {
                switch(freqdigitcursor) {
                case 0:
                  vfofrequency += 100.0;
                  break;
                case 1:
                  vfofrequency += 10.0;
                  break;
                case 2:
                  vfofrequency += 1.0;
                  break;
                case 3:
                  vfofrequency += 0.1;
                  break;
                case 4:
                  vfofrequency += 0.01;
                  break;
                case 5:
                  vfofrequency += 0.001;
                  break;
                case 6:
                  vfofrequency += 0.0001;
                  break;
                case 7:
                  vfofrequency += 0.00001;
                  break;
                }
                if(vfofrequency > 225.0) {
                  vfofrequency = 225.0; //cap value
                }
                si5351setfrequenecy(vfofrequency);
              }
              if(down) {
                switch(freqdigitcursor) {
                case 0:
                  vfofrequency -= 100.0;
                  break;
                case 1:
                  vfofrequency -= 10.0;
                  break;
                case 2:
                  vfofrequency -= 1.0;
                  break;
                case 3:
                  vfofrequency -= 0.1;
                  break;
                case 4:
                  vfofrequency -= 0.01;
                  break;
                case 5:
                  vfofrequency -= 0.001;
                  break;
                case 6:
                  vfofrequency -= 0.0001;
                  break;
                case 7:
                  vfofrequency -= 0.00001;
                  break;
                }
                if(vfofrequency < 0.008) {
                  vfofrequency = 0.008; //cap value
                }
                si5351setfrequenecy(vfofrequency);
              }
            }
            break;
          case 1: //change drive power
            if(blockmenunavigation) {
              if(left || down) {
                vfodrive--;
                if(vfodrive == 255) {
                  vfodrive = 3; //loop around
                }
                si5351.drive_strength(SI5351_CLK0, vfodrive);
              }
              if(right || up) {
                vfodrive++;
                if(vfodrive > 3) {
                  vfodrive = 0; //loop around
                }
                si5351.drive_strength(SI5351_CLK0, vfodrive);
              }
            }
            break;
          case 2: //scroll frequency presets
            if(blockmenunavigation) {
              if(left || down) {
                vfopresetselect--;
                if(vfopresetselect == 255) {
                  vfopresetselect = vfopresetsnumber-1;
                }
                vfofrequency = vfopresets[vfopresetselect];
                si5351setfrequenecy(vfofrequency);
              }
              if(right || up) {
                vfopresetselect++;
                if(vfopresetselect > vfopresetsnumber-1) {
                  vfopresetselect = 0;
                }
                vfofrequency = vfopresets[vfopresetselect];
                si5351setfrequenecy(vfofrequency);
              }
            }
            break;
        }
      }
      switch(vfoscan) {
        case 1: //scan up
          vfofrequency += 0.001;
          
          break;
        case 2: //scan down
          vfofrequency -= 0.001;
          break;
      }
      if(vfoscan) {
        si5351setfrequenecy(vfofrequency);
        updatescreen = 1;
        delay(10);
      }
      break;
    case nmeaparser__: //nmea parser
      if(sidepanelenable) {
        if(b0) { //exit
          menuselect = apps__; //go to apps
          sidepanelenable = 0; //close sidepanel
        }
        if(b1) { //measure distance from here
          sidepanelenable = 0; //close sidepanel
          latitude1 = latitude;
          longitude1 = longitude;
        }
        if(b2 || b3 || b4) {
          sidepanelenable = 0; //close sidepanel
        }
      } else {
        if(b3) {
          sidepanelenable = 1;
        }
      }
      break;
    case gpsmap__: //gps map
      if(sidepanelenable) {
        if(b0) { //exit
          menuselect = apps__; //go to apps
          sidepanelenable = 0; //close sidepanel
        }
        if(b1) { //measure distance from here
          sidepanelenable = 0; //close sidepanel
          latitude1 = latitude;
          longitude1 = longitude;
        }
        if(b2) {
          sidepanelenable = 0; //close sidepanel
          gpsmaplinetoggle = !gpsmaplinetoggle;
        }
        if(b3 || b4) {
          sidepanelenable = 0; //close sidepanel
        }
      } else {
        if(b3) {
          sidepanelenable = 1;
        }
      }
      break;
//    case : //
//      break;
//    case : //
//      break;
//    case : //
//      break;
//    case : //
//      break;
//    case : //
//      break;
//    case : //
//      break;
//    case : //
//      break;
  }

  /*menu animation*/
  if(lineselect != lastlineselect) { //check if lineselect has changed
    lineanim = 8; //reset animation
  }
  lastlineselect = lineselect;
  if(lineanim > 0.5) { //keep updating the screen while the animation is going
    lineanim *= 0.7;
    updatescreen = 1;
  }

  if(lastoptionwindow != optionwindow) { //check if optionwindow has changed
    windowanim = 8; //reset animation
  }
  lastoptionwindow = optionwindow;
  if(windowanim > 0.5) { //keep updating the screen while the animation is going
    windowanim *= 0.7;
    updatescreen = 1;
  }

  if(lastsidepanelenable != sidepanelenable) {
    sidepanelsanim = 64;  //reset animation
  }
  lastsidepanelenable = sidepanelenable;
  if(sidepanelsanim > 0.5) { //keep updating the screen while the animation is going
    sidepanelsanim *= 0.8;
    updatescreen = 1;
  }

  sidepanelsanimdirection = sidepanelenable;

  if(uifilterbufx > 0.5 || uifilterbufy > 0.5) { //keep updating the screen while the animation is going
    updatescreen = 1;
  }

  if(letterjumpanim > 0.5) { //keep updating the screen while the animation is going
    letterjumpanim *= 0.8;
    updatescreen = 1;
  }

  if(menuselect == gpsmap__) { //only in select menus
    if(millis() > screenblinkmillis + 700) { //periodically update screen
      screenblinkmillis = millis();
      screenblink = !screenblink;
      updatescreen = 1;
    }
  }

  if(updatescreen) {
    u8g2.clearBuffer();

    switch(menuselect) {
      /*(mark-I)*/
      case desktop__ : //desktop
        desktopscreen();
        break;
      case apps__: //apps
        menuscreen(menuoptiontitles);
        break;
      case serialterminal__: //serial terminal
        if(!terminalkeybenable) {
          terminalscreen();
        } else {
          keyboardscreen(terminalfieldbuffer, sizeof(terminalfieldbuffer), terminalfieldptr);
        }
        break;
      case serialmacros__: //serial macros
        menuscreen(serialmacros);
        terminalscreen();
        u8g2.drawHLine(0, 32, 128); //divider
        break;
      case serialconfigurator__: //serial configurator
        menuscreen(serialconfigoptions);
        break;
      case rfm95chat__: //rfm95 chat
        keyboardscreen(writemsgbuffer, sizeof(writemsgbuffer), writemsgbufferptr);
        break;
      case rfm95receive__: //rfm95 receive
//        rfm95recvscreen();
        u8g2.drawStr(0, 7, rfm95recvtext);
        break;
      case rfm95configurator__: //rfm95 configurator
        u8g2.setCursor(62, 7); u8g2.print(rfm95frequency, 3);
        u8g2.setCursor(86, 15); (rfm95contmode ? u8g2.print("On") : u8g2.print("Off"));
        u8g2.setCursor(103, 23); u8g2.print(rfm95spreadingfactor);
        u8g2.setCursor(80, 31); u8g2.print(rfm95txbw);
        u8g2.setCursor(56, 39); u8g2.print(rfm95txpower);
        u8g2.setCursor(56, 47); u8g2.print(rfm95preamble);
        if(blockmenunavigation && optionselect == 0) {
          uint8_t cursorpos;
          switch(freqdigitcursor) {
            case 0:
              cursorpos = 0;
              break;
            case 1:
              cursorpos = 6;
              break;
            case 2:
              cursorpos = 12;
              break;
            case 3:
              cursorpos = 24;
              break;
            case 4:
              cursorpos = 30;
              break;
            case 5:
              cursorpos = 36;
              break;
          }
          if(rfm95frequency >= 1000.0) {
            cursorpos += 6; //offset cursor when value is above 1000
          }
          u8g2.setDrawColor(2);
          u8g2.drawBox(62+cursorpos, 0, 6, 8); //selection box
          u8g2.setDrawColor(1);
        }
        menuscreen(rfm95configoptions);
        break;
      case nrf24control__: //nrf24 control
        u8g2.drawStr(0, 7, "NRF24 hook");
        float nrf24hookbattlvl = (nrf24controlrxbuf.battvoltage + 305) / 100.0; //calculate voltage
        if(nrf24hookbattlvl < 3.07) {
          u8g2.drawStr(0, 23, "not connected");
        } else {
          u8g2.setCursor(0, 23);
          u8g2.print(nrf24hookbattlvl, 2); u8g2.print('V');
        }
        switch(nrf24controlparamselect) {
          case 0: //servo
            u8g2.drawStr(0, 63, "SERVO");
            break;
          case 1: //motor
            u8g2.drawStr(0, 63, "MOTOR");
            break;
          case 2: //control
            u8g2.drawStr(0, 63, "CONTROL");
            break;
        }
        if(nrf24controlparamchanging) {
          u8g2.drawVLine(78, 0, 64); //divider
          switch(nrf24controlparamselect) {
            case 0: //servo
              u8g2.drawStr(90, 10, "up");
              u8g2.drawStr(90, 27, "dwn");
  //            u8g2.drawStr(68, 43, "");
  //            u8g2.drawStr(68, 59, "");
  
              u8g2.drawStr(0, 39, "servo >");
              u8g2.setCursor(0, 47);
              u8g2.print(nrf24controltxbuf.servopwm);
              break;
            case 1: //motor
              u8g2.drawStr(80, 10, "roll up");
              u8g2.drawStr(80, 27, "roll dwn");
              u8g2.drawStr(80, 43, "brake");
              u8g2.drawStr(80, 59, "free");
  
              u8g2.drawStr(0, 39, "motor >");
              switch(nrf24controltxbuf.motorctrl) {
                case 0:
                  u8g2.drawStr(0, 47, "free");
                  break;
                case 1:
                  u8g2.drawStr(0, 47, "rolling up");
                  break;
                case 2:
                  u8g2.drawStr(0, 47, "rolling down");
                  break;
                case 3:
                  u8g2.drawStr(0, 47, "braking");
                  break;
                
              }
              break;
            case 2: //control
              u8g2.drawStr(80, 10, "wake up");
              u8g2.drawStr(80, 27, "sleep");
  //            u8g2.drawStr(68, 43, "");
  //            u8g2.drawStr(68, 59, "");
              break;
            
          }
        }
        break;
      case rda5807radio__: //rda5807 radio
        u8g2.setFont(u8g2_font_VCR_OSD_tn);
        u8g2.setCursor(0, 19); 
        if(radiofrequency < 100.000) {
          u8g2.print('0');
        }
        u8g2.print(radiofrequency, 3);
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.setCursor(0, 39); u8g2.print("VOLUME: "); u8g2.print(radiovolume);
        u8g2.setCursor(0, 55); u8g2.print("OSC: "); 
        switch(radioosc) {
          case 0:
            u8g2.print("32k");
            break;
          case 1:
            u8g2.print("12M");
            break;
          case 2:
            u8g2.print("13M");
            break;
          case 3:
            u8g2.print("19.2M");
            break;
          case 5:
            u8g2.print("24M");
            break;
          case 6:
            u8g2.print("26M");
            break;
          case 7:
            u8g2.print("38.4M");
            break;
        }
        u8g2.setCursor(0, 63); u8g2.print("TYPE: "); (radioosctype ? u8g2.print("REF") : u8g2.print("XTAL"));
        if(blockmenunavigation && optionselect == 0) {
          uint8_t cursorpos;
          switch(freqdigitcursor) {
            case 0:
              cursorpos = 0;
              break;
            case 1:
              cursorpos = 12;
              break;
            case 2:
              cursorpos = 24;
              break;
            case 3:
              cursorpos = 48;
              break;
            case 4:
              cursorpos = 60;
              break;
            case 5:
              cursorpos = 72;
              break;
          }
          u8g2.setDrawColor(2);
          u8g2.drawBox(cursorpos, 4, 12, 14); //selection box
          u8g2.setDrawColor(1);
        } else {
          u8g2.setDrawColor(2);
          switch(optionselect) {
            case 0: //freq adjust
              u8g2.drawBox(0, 4, 84, 14); //selection box
              break;
            case 1: //volume
              u8g2.drawBox(0, 32, 64, 8); //selection box
              break;
            case 2: //osc freq
              u8g2.drawBox(0, 48, 64, 8); //selection box
              break;
            case 3: //osc type
              u8g2.drawBox(0, 56, 64, 8); //selection box
              break;
          }
          u8g2.setDrawColor(1);
        }
        break;
      case si5351vfo__: //si5351 vfo
        u8g2.setFont(u8g2_font_VCR_OSD_tn);
        u8g2.setCursor(0, 19); 
        if(vfofrequency < 100.000) {
          u8g2.print('0');
        }
        if(vfofrequency < 10.000) {
          u8g2.print('0');
        }
        u8g2.print(vfofrequency, 5);
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.setCursor(0, 39); u8g2.print("DRIVE POWER: "); u8g2.print(vfodrive);
        u8g2.setCursor(0, 47); u8g2.print("PRESET: "); u8g2.print(vfopresetselect+1); u8g2.print('>'); u8g2.print(vfopresets[vfopresetselect], 5);

        if(blockmenunavigation && optionselect == 0) {
          uint8_t cursorpos;
          switch(freqdigitcursor) {
            case 0:
              cursorpos = 0;
              break;
            case 1:
              cursorpos = 12;
              break;
            case 2:
              cursorpos = 24;
              break;
            case 3:
              cursorpos = 48;
              break;
            case 4:
              cursorpos = 60;
              break;
            case 5:
              cursorpos = 72;
              break;
            case 6:
              cursorpos = 84;
              break;
            case 7:
              cursorpos = 96;
              break;
          }
          u8g2.setDrawColor(2);
          u8g2.drawBox(cursorpos, 4, 12, 14); //selection box
          u8g2.setDrawColor(1);
        } else {
          u8g2.setDrawColor(2);
          if(blockmenunavigation) {
            switch(optionselect) {
              case 0: //freq adjust
                u8g2.drawBox(0, 4, 108, 14); //selection box
                break;
              case 1: //drive power
                u8g2.drawBox(78, 32, 127, 8); //selection box
                break;
              case 2: //frequency presets
                u8g2.drawBox(48, 40, 127, 8); //selection box
                break;
            }
          } else {
            switch(optionselect) {
              case 0: //freq adjust
                u8g2.drawBox(0, 4, 108, 14); //selection box
                break;
              case 1: //drive power
                u8g2.drawBox(0, 32, 127, 8); //selection box
                break;
              case 2: //frequency presets
                u8g2.drawBox(0, 40, 127, 8); //selection box
                break;
            }
          }
          u8g2.setDrawColor(1);
        }
        break;
      case nmeaparser__: //nmea parser
        nmeascreen();
        break;
      case gpsmap__: //gps map
        gpsmapscreen();
        break;
//      case : //
//        break;
//      case : //
//        break;
//      case : //
//        break;
//      case : //
//        break;
//      case : //
//        break;
//      case : //
//        break;
//      case : //
//        break;
//      case : //
//        break;
    }

    sidepanelspopup(); //infront of all layers

    u8g2.sendBuffer();
//    delay(100);
  }
//  port1.println(menuselect);
  resetstates();
}

void resetstates() {
  b0 = 0;
  b1 = 0;
  b2 = 0;
  b3 = 0;
  b4 = 0;
  left = 0;
  right = 0;
  up = 0;
  down = 0;
  enter = 0;
  menubutton = 0;
  
  globalbuttons = 0;
  updatescreen = 0;
}

void desktopscreen() {
  u8g2.drawXBMP(0, 0, 128, 64, bckgnd1);
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 0, 128, 8);
  u8g2.setDrawColor(1);

  pinMode(PIN_PE3, OUTPUT);
  delay(200);
  float battlvl = map(analogRead(PIN_PE2), 513, 647, 3300, 4200) / 1000.0;
  pinMode(PIN_PE3, INPUT);
  u8g2.setCursor(0, 7);
  u8g2.print(battlvl, 2);
  u8g2.print('v');
}

void menuscreen(String *titles) {
  for(byte i = 0; i < menulength[menulengthselect]; i++) { //displaying multiple option lines
    int8_t rangelow;
    if(halfmenu) {
      rangelow = i - 3;
    } else {
      rangelow = i - 7;
    }
    if(rangelow < 0) {
      rangelow = 0; //cap value
    }
    if(optionwindow >= rangelow && optionwindow <= 7+i) {
//        u8g2.drawStr(0, (8*((1+i)-optionwindow))-1, titles[i]); //non animated
      if(menuanimdirection) {
        u8g2.drawStr(0, ((8*((1+i)-optionwindow))-8)+(8-windowanim), titles[i].c_str());
      } else {
        u8g2.drawStr(0, ((8*((1+i)-optionwindow))-1)+windowanim, titles[i].c_str());
      }
    }
  }
  
//  u8g2.drawGlyph(120, 11, '^');
//  u8g2.drawGlyph(120, 23, 'v');
//  u8g2.setCursor(120, 15);
//  u8g2.print(optionselect);

  if(!(blockmenunavigation && optionselect == 0 && menuselect == rfm95configurator__)) {
    u8g2.setDrawColor(2);
    if(menuanimdirection) {
      u8g2.drawBox(0, (lineselect*8)+lineanim, 128, 8); //selection box
    } else {
      u8g2.drawBox(0, ((lineselect*8)-7)+(8-lineanim), 128, 8); //selection box
    }
    u8g2.setDrawColor(1);
  }
}

void terminalscreen() {
 for(byte i = 0; i < (sizeof(textinputbuffer)/hchars)-(halfterminal ? 3 : 0); i++) { //displaying text with wrapping
    u8g2.setCursor(0, ((i+1)*8)+(halfterminal ? 32 : 0));
    for(byte k = 0; k < hchars; k++) {
      u8g2.print((char)textinputbuffer[k+(hchars*i)]);
    }
  }
  if(!halfterminal) {
    u8g2.setCursor(0, 63);
    for(byte i = 0; i < hchars; i++) { //show text input field
      int8_t offset = terminalfieldptr - hchars;
      if(offset < 0) {
        offset = 0;
      }
      u8g2.print((char)terminalfieldbuffer[offset+i]);
    }
    u8g2.setDrawColor(2);
    u8g2.drawBox(0, 56, 128, 8); //text input field
    u8g2.setDrawColor(1);
  }
}

void sidepanelspopup() {
  uint8_t sidepanelsanim_;
  if(sidepanelsanimdirection) {
    sidepanelsanim_ = sidepanelsanim;
  } else {
    sidepanelsanim_ = 64 - sidepanelsanim; //invert
  }
  if(sidepanelenable || sidepanelsanim > 1) { //disappear only when animation has finished
    u8g2.setDrawColor(0);
    u8g2.drawBox(64+sidepanelsanim_, 0, 64, 64); //clear background
    u8g2.setDrawColor(1);
    u8g2.drawXBMP(64+sidepanelsanim_,0, 2, 64, sidepanels); //panel edges
    u8g2.drawHLine(66+sidepanelsanim_, 0, 62); //panel seams
    for(byte i = 0; i < 4; i++) {
      u8g2.drawHLine(66+sidepanelsanim_, (i*16)+15, 62); //panel seams
    }

    switch(menuselect) {
      case serialterminal__: //serial terminal
        u8g2.drawStr(68+sidepanelsanim_, 10, "Clear");
        u8g2.drawStr(68+sidepanelsanim_, 27, "Exit");
        if(!serialspeedtoggle) {
          u8g2.drawStr(68+sidepanelsanim_, 43, "115200");
        } else {
          u8g2.drawStr(68+sidepanelsanim_, 43, "9600");
        }
        u8g2.drawStr(68+sidepanelsanim_, 59, "Keyb.");
        break;
      case serialmacros__: //serial macros
        u8g2.drawStr(68+sidepanelsanim_, 10, "Clear");
        u8g2.drawStr(68+sidepanelsanim_, 27, "Exit");
        if(!serialspeedtoggle) {
          u8g2.drawStr(68+sidepanelsanim_, 43, "115200");
        } else {
          u8g2.drawStr(68+sidepanelsanim_, 43, "9600");
        }
        break;
      case serialconfigurator__: //serial configurator
        u8g2.drawStr(68+sidepanelsanim_, 10, "Exit");
        break;
      case rfm95configurator__: //rfm95 configurator
        u8g2.drawStr(68+sidepanelsanim_, 10, "Reset");
        u8g2.drawStr(68+sidepanelsanim_, 27, "Exit");
        u8g2.drawStr(68+sidepanelsanim_, 43, "---");
        u8g2.drawStr(68+sidepanelsanim_, 59, "---");
        break;
      case nrf24control__: //nrf24 control
        u8g2.drawStr(68+sidepanelsanim_, 10, "Exit");
        break;
      case rda5807radio__: //rda5807 radio
        u8g2.drawStr(68+sidepanelsanim_, 10, "Exit");
        break;
      case si5351vfo__:
        u8g2.drawStr(68+sidepanelsanim_, 10, "Exit");
        u8g2.drawStr(68+sidepanelsanim_, 27, "Scan >");
        u8g2.drawStr(68+sidepanelsanim_, 43, "Scan <");
        break;
      case nmeaparser__: //nmea parser
        u8g2.drawStr(68+sidepanelsanim_, 10, "Exit");
        u8g2.drawStr(68+sidepanelsanim_, 27, "Measure");
        break;
      case gpsmap__: //gps map
        u8g2.drawStr(68+sidepanelsanim_, 10, "Exit");
        u8g2.drawStr(68+sidepanelsanim_, 27, "Measure");
        u8g2.drawStr(68+sidepanelsanim_, 43, "Line");
        break;
    }
  }
}

void movetextlinesup(uint8_t *buf, size_t len) {
  for(byte k = 0; k < (len/21)-1; k++) { //for every line
    for(byte i = 0; i < hchars; i++) {
      uint8_t charbuf = buf[(i+hchars)+(k*hchars)]; //copy from top row
      buf[i+(k*hchars)] = charbuf; //to the one under it
    }
  }
}

void serialtoterminal(uint8_t *buf, size_t len, uint8_t input, uint8_t &ptr) {
  if(input == '\n' /*|| input == '\r'*/) { //new line 
//    ptr += hchars-(ptr%hchars); //only used if non scrolling terminal
    if(terminalscrollingstarted) {
      ptr = len - (halfterminal ? 63 : 0); //overthrow ptr so it activates scrolling newline
    } else {
      ptr += hchars-(ptr%hchars);
    }
  }
  
  if(input >= 32 && input <= 126) { //write only valid characters to buffer
    buf[ptr] = input;
    ptr++; 
  }
  if(ptr > (len-1) - (halfterminal ? 63 : 0)) {
//    ptr = 0; //reset pointer //only used if non scrolling terminal

    terminalscrollingstarted = 1;
    ptr = (len-hchars) - (halfterminal ? 63 : 0); //reset pointer to last row
    movetextlinesup(buf, len);
    for(byte i = 0; i < hchars; i++) { //clear last line
      buf[ptr+i] = 0;
    }
  }
  updatescreen = 1;
}

void terminaloutput(uint8_t *buf, size_t len, uint8_t input, uint8_t &ptr) {
//  if(enter) {
//    port1.write(buf, strlen((char*)buf));
//    memset(buf, 0, len); //clear input field
//    ptr = 0; //reset
//  }
  if(input == 136) { //backspace //remove char
    removelastchar(buf, ptr);
  }
  if(input >= 32 && input <= 126) {
    buf[ptr] = input;
    ptr++;
  }
  if(ptr > len-1) { //prevent overflowing the array
    ptr = len-1;
  }
}

void keyboardinterface(uint8_t *buf, size_t len, uint8_t &ptr) {
  if(up) {
    keybrowselect--;
    if(keybrowselect == 255) {
      keybrowselect = 4; //loop around
    }
    if(keybrowselect == 4 && keybcolselect > 3) { //fix going outside the keyboard on last row
      keybcolselect = 3; //cap value
    }
  }
  if(down) {
    keybrowselect++;
    if(keybrowselect > 4) {
      keybrowselect = 0; //loop around
    }
    if(keybrowselect == 4 && keybcolselect > 3) { //fix going outside the keyboard on last row
      keybcolselect = 3; //cap value
    }
  }
  if(left) {
    keybcolselect--;
    if(keybcolselect == 255) {
      keybcolselect = (keybrowselect == 4) ? 5 : 9; //loop around //only when on last row loop around on 4 steps
    }
  }
  if(right) {
    keybcolselect++;
    if(keybcolselect > ((keybrowselect == 4) ? 5 : 9)) { //only when on last row loop around on 4 steps
      keybcolselect = 0; //loop around
    }
  }
  if(enter) { //write letter
    if(keybrowselect < 4) { //only write while in keyboard region
      buf[ptr] = keybletters[keybrowselect][keybcolselect+(keybcaps ? 10 : 0)];
      ptr++;
      if(ptr > len-1) { //prevent overflowing the array
        ptr = len-1;
      }
      letterjumpanim = 8; //start letter animation
    }
  }
}

void keyboardscreen(uint8_t *buf, size_t len, uint8_t ptr) {
  displaywrappingtext(buf, len);
  u8g2.drawGlyph((ptr%21)*6, (((ptr/21)+1)*8)-3, '_'); //text cursor
  
  for(byte k = 0; k < 4; k++) { //keyboard interface
    for(byte i = 0; i < 10; i++) {
      if(keybcolselect == i && keybrowselect == k) { //only selected letter should jump
        u8g2.drawGlyph((i*12)+(k*4), ((8*(k+4))-1)-letterjumpanim, keybletters[k][i+(keybcaps ? 10 : 0)]);
      } else {
        u8g2.drawGlyph((i*12)+(k*4), ((8*(k+4))-1), keybletters[k][i+(keybcaps ? 10 : 0)]);
      }
    }
  }
  u8g2.drawStr(1, 63, "  spc exit tx clear <");
  u8g2.drawGlyph(1, 66, '^');
//  u8g2.drawHLine(0, 23, 128); //divider
  u8g2.drawHLine(0, 56, 128); //divider
  
  u8g2.setDrawColor(2);
  if(keybrowselect < 4) {
    u8g2.drawBox(uifilterx((keybcolselect*12)+(keybrowselect*4)), uifiltery((keybrowselect*8)+24), 6, 8); //selector
  } else {
    switch(keybcolselect) {
      case 0: //caps
        u8g2.drawBox(1, 57, 6, 8); //selector
        break;
      case 1: //space
        u8g2.drawBox(13, 57, 18, 8); //selector
        break;
      case 2: //exit
        u8g2.drawBox(37, 57, 24, 8); //selector
        break;
      case 3: //tx
        u8g2.drawBox(67, 57, 12, 8); //selector
        break;
      case 4: //clear
        u8g2.drawBox(85, 57, 30, 8); //selector
        break;
      case 5: //backspace
        u8g2.drawBox(121, 57, 6, 8); //selector
        break;
    }
  }
  u8g2.setDrawColor(1);
}

uint8_t uifilterx(uint8_t inx) {
  uifilterbufx = uifilterbufx + (0.5 * (inx - uifilterbufx));
  return (uint8_t)uifilterbufx;
}

uint8_t uifiltery(uint8_t iny) {
  uifilterbufy = uifilterbufy + (0.5 * (iny - uifilterbufy));
  return (uint8_t)uifilterbufy;
}

void terminaltx(uint8_t *buf, size_t len, uint8_t &ptr, bool clearbuffer) {
  if(ptr == 0) {
    return;
  }
  port1->write(buf, strlen((char*)buf));
  if(clearbuffer) {
    cleardatabuffer(buf, len, ptr);
  }
}

void cleardatabuffer(uint8_t *buf, size_t len, uint8_t &ptr) {
  memset(buf, 0, len); //fill with null
  ptr = 0;
}

void removelastchar(uint8_t *buf, uint8_t &ptr) {
  ptr--;
  if(ptr == 255) {
    ptr = 0; //cap value
  }
  buf[ptr] = 0;
}

void addspace(uint8_t *buf, uint8_t &ptr) {
  buf[ptr] = ' ';
  ptr++;
}

void changeserialspeed(bool speed) {
  port1->end();
  if(speed) {
    port1->begin(115200);
  } else {
    port1->begin(9600);
  }
}

void toneclick() {
  if(soundenable) {
    tone(PIN_PC4, 1000, 10);
  }
}

void movecharstoright(uint8_t *data, size_t len) {
  uint8_t buf = 0;
  for(byte i = len-1; i > 0; i--) {
    buf = data[i-1];
    data[i] = buf;
  }
}

void rfm95addheader(uint8_t *data, size_t len) {
  for(byte i = 0; i < 4; i++) {
    movecharstoright(data, len); //free up 4 bytes in the beginning
    data[i] = rfm95header[0]; //copy header to msg buffer
  }
}

bool initrfm95() {
//  SPI.begin();
//  pinMode(PIN_PC0, OUTPUT);
//  digitalWrite(PIN_PC0, LOW);
//  SPI.transfer()
//  spi.beginTransaction(_spiSettings);

//  uint8_t val = rfm95.getModemStatus();
//  if(val != 255) { //lora mode
//    u8g2.clearBuffer();
//    u8g2.drawStr(0, 7, "no rfm95 module found");
//    u8g2.setCursor(0, 15);
//    u8g2.print(val);
//    u8g2.sendBuffer();
//    delay(1000);
//    updatescreen = 1;
//    menuselect = apps__;
//    return 0;
//  }
//  port1->println("asd");
  
  rfm95.begin();
  rfm95.setFrequency(860.011);
  rfm95.setPreambleLength(32);
  rfm95.setBandwidth(31.25);
  rfm95.setCodingRate(8); //4/8 coding rate
  rfm95.setSpreadingFactor(9); //512 chips
  rfm95.setCRC(1, 0); //CCITT
  rfm95.forceLDRO(0); //low data rate optimization off
  return 1;
}

void rfm95recvscreen() {
  displaywrappingtext(rfm95recvtext, sizeof(rfm95recvtext));
}

void rfm95recvmsgs(uint8_t *data, size_t len, uint8_t &ptr) {
  if(terminalscrollingstarted) { //new line 
    ptr = len; //overthrow ptr so it activates scrolling newline
  } else {
    ptr += hchars-(ptr%hchars);
  }
  char msgnumber[8];
  sprintf(msgnumber, "%u-", rfm95recvcount);
  uint8_t msgnumberlen = strlen(msgnumber);
  uint8_t screenlen = hchars*vchars;
  for(byte i = 0; i < msgnumberlen; i++) { //add msg number before msg
    rfm95recvtext[ptr] = msgnumber[i];
    ptr++;
  }
  for(byte i = 0; i < len; i++) { //for every character from message
    rfm95recvtext[ptr] = data[i];
    ptr++; 
    if(ptr > screenlen-1) {
      terminalscrollingstarted = 1;
      ptr = (screenlen-hchars); //reset pointer to last row
      movetextlinesup(rfm95recvtext, screenlen);
      for(byte i = 0; i < hchars; i++) { //clear last line
        rfm95recvtext[ptr+i] = 0;
      }
    }
  }
  rfm95recvcount++;
  updatescreen = 1;
}

void displaywrappingtext(uint8_t *data, size_t len) {
  for(byte i = 0; i < (len/hchars); i++) { //displaying text with wrapping
    u8g2.setCursor(0, ((i+1)*8)); //change line
    for(byte k = 0; k < hchars; k++) {
      u8g2.print((char)data[k+(hchars*i)]); //char by char
    }
  }
}

void swapserialports() {
  u8g2.clearBuffer();
  
  serialswaptoggle = !serialswaptoggle;
  if(serialswaptoggle) {
    port1 = &Serial4;
    port2 = &Serial2;
    
    u8g2.drawStr(0, 16, "port1");
    u8g2.drawStr(0, 24, "(serial");
    u8g2.drawStr(0, 32, "input)");
    
    u8g2.drawStr(74, 8, "port2");
    u8g2.drawStr(74, 16, "(serial");
    u8g2.drawStr(74, 24, "control)");
  } else {
    port1 = &Serial2;
    port2 = &Serial4;

    u8g2.drawStr(0, 16, "port2");
    u8g2.drawStr(0, 24, "(serial");
    u8g2.drawStr(0, 32, "control)");
    
    u8g2.drawStr(74, 8, "port1");
    u8g2.drawStr(74, 16, "(serial");
    u8g2.drawStr(74, 24, "input)");
  }
  
  u8g2.drawStr(0, 40, "+-gnd");
  u8g2.drawStr(0, 48, "+-vcc");
  u8g2.drawStr(0, 56, "+-rx");
  u8g2.drawStr(0, 64, "+-tx");
  
  u8g2.drawStr(74, 32, "r     v g");
  u8g2.drawStr(74, 40, "e t r c n");
  u8g2.drawStr(74, 48, "s x x c d");
  u8g2.drawStr(74, 56, "| | | | |");
  u8g2.drawStr(74, 64, "+ + + + +");
  
  u8g2.sendBuffer();
  delay(4000);
}

void initgps() {
  MicroNMEA::sendSentence(*port1, "$PORZB");

  // Send only RMC and GGA messages.
  MicroNMEA::sendSentence(*port1, "$PORZB,RMC,1,GGA,1");

  // Disable compatability mode (NV08C-CSM proprietary message) and
  // adjust precision of time and position fields
  MicroNMEA::sendSentence(*port1, "$PNVGNME,2,9,1");
}

void nmeascreen() {
  latitude = nmea.getLatitude() / 1000000.0; //update coords
  longitude = nmea.getLongitude() / 1000000.0;
  u8g2.setCursor(0, 7);
  u8g2.print("lat: "); u8g2.print(latitude, 6);
  u8g2.setCursor(0, 15);
  u8g2.print("lon: "); u8g2.print(longitude, 6);
  u8g2.setCursor(0, 23);
  u8g2.print("satellites: "); u8g2.print(nmea.getNumSatellites());
  u8g2.setCursor(0, 31);
//  u8g2.print("nav system: "); u8g2.print(nmea.getNavSystem());
  u8g2.print("direction: "); u8g2.print(nmea.getCourse(), 4);
  u8g2.setCursor(0, 39);
  u8g2.print("hdop: "); u8g2.print(nmea.getHDOP()/10.0);

  u8g2.setCursor(0, 47); u8g2.print("distance from "); 
  u8g2.setCursor(0, 55); u8g2.print(latitude1, 6); u8g2.print(", "); u8g2.print(longitude1, 6);
  u8g2.setCursor(0, 63); u8g2.print("to here: "); u8g2.print(gpsmeasuredistance(latitude1, longitude1, latitude, longitude), 3); u8g2.print("km");
}

float gpsmeasuredistance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371; // Radius of the earth in km
  float dLat = deg2rad(lat2-lat1);
  float dLon = deg2rad(lon2-lon1); 
  float a = 
    sin(dLat/2) * sin(dLat/2) +
    cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * 
    sin(dLon/2) * sin(dLon/2); 
  float c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  float d = R * c; // Distance in km
  return d;
}

float deg2rad(float deg) {
  return deg * (PI/180);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t calcgpslon2lcdposx(float lon) {
  return mapfloat(lon, 24.683327, 24.810207, 0, 127); //calculate coords to screen position
}

uint8_t calcgpslat2lcdposy(float lat) {
  return mapfloat(lat, 42.166393, 42.118721, 0, 64); //calculate coords to screen position
}

float calc_degrees(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
  degrees = atan2(y2 - y1, x2 - x1) * 180 / PI;
  return degrees;
}

uint16_t cap(int16_t input) {
  if(input < 0) {
    input = 0;
  }
  return input;
}

void arrow(uint16_t x, uint16_t y, uint16_t deg) {
  #define inner_radius 8
  #define outer_radius 16
  
  deg-=90;
  
  int16_t a_sin = sin(5.7595 + deg2rad(deg)) * inner_radius;
  int16_t a_cos = cos(5.7595 + deg2rad(deg)) * inner_radius;

  int16_t b_sin = sin(1.5707 + deg2rad(deg)) * outer_radius;
  int16_t b_cos = cos(1.5707 + deg2rad(deg)) * outer_radius;

  int16_t c_sin = sin(3.6651 + deg2rad(deg)) * inner_radius;
  int16_t c_cos = cos(3.6651 + deg2rad(deg)) * inner_radius;

//  /*non-inverted*/
//  u8g2.drawLine(x,y,x+a_sin,y+a_cos);
//  u8g2.drawLine(x+a_sin,y+a_cos,x+b_sin,y+b_cos);
//  u8g2.drawLine(x+b_sin,y+b_cos,x+c_sin,y+c_cos);
//  u8g2.drawLine(x+c_sin,y+c_cos,x,y);

//  /*inverted*/
//  u8g2.drawLine(x,y,x+a_cos,y+a_sin);
//  u8g2.drawLine(x+a_cos,y+a_sin,x+b_cos,y+b_sin);
//  u8g2.drawLine(x+b_cos,y+b_sin,x+c_cos,y+c_sin);
//  u8g2.drawLine(x+c_cos,y+c_sin,x,y);

  /*non-inverted & limited*/
  u8g2.drawLine(x,y,cap(x+a_sin),cap(y+a_cos));
  u8g2.drawLine(cap(x+a_sin),cap(y+a_cos),cap(x+b_sin),cap(y+b_cos));
  u8g2.drawLine(cap(x+b_sin),cap(y+b_cos),cap(x+c_sin),cap(y+c_cos));
  u8g2.drawLine(cap(x+c_sin),cap(y+c_cos),x,y);

//  /*inverted & limited*/
//  u8g2.drawLine(x,y,cap(x+a_cos),cap(y+a_sin));
//  u8g2.drawLine(cap(x+a_cos),cap(y+a_sin),cap(x+b_cos),cap(y+b_sin));
//  u8g2.drawLine(cap(x+b_cos),cap(y+b_sin),cap(x+c_cos),cap(y+c_sin));
//  u8g2.drawLine(cap(x+c_cos),cap(y+c_sin),x,y);
}

void gpsmapscreen() {
  latitude = nmea.getLatitude() / 1000000.0; //update coords
  longitude = nmea.getLongitude() / 1000000.0;
  
  gpsypos = calcgpslat2lcdposy(latitude); 
  gpsxpos = calcgpslon2lcdposx(longitude);

  if(lastgpsxpos != gpsxpos || lastgpsypos != gpsypos) { //check for changes
    gpsmappointsx[gpsmappointsptr] = gpsxpos;
    gpsmappointsy[gpsmappointsptr] = gpsypos;
    gpsmappointsptr++;
  }
  lastgpsxpos = gpsxpos;
  lastgpsypos = gpsypos;

  for(uint16_t i = 0; i < gpsmappointsptr+1; i++) { //draw every point
    u8g2.drawPixel(gpsmappointsx[i], gpsmappointsy[i]);
  }

//  gpsypos = 31;
//  gpsxpos = 63;

  
  float degrees = calc_degrees(gpsmappointsx[gpsmappointsptr-4], gpsmappointsy[gpsmappointsptr-4], gpsxpos, gpsypos);
  if(screenblink) { //blink
//    u8g2.drawHLine(gpsxpos -4, gpsypos, 9); //cross
//    u8g2.drawVLine(gpsxpos, gpsypos -4, 9);
    arrow(gpsxpos, gpsypos, degrees);
  }

//  gpsdirection+=0.01;
//  if(gpsdirection > 3.14159*2) {
//    gpsdirection = 0;
//  }
//  
//
//  u8g2.drawLine(gpsxpos, gpsypos, gpsxpos - (sin(gpsdirection)*20), gpsypos - (cos(gpsdirection)*20));

  if(gpsmaplinetoggle) {
    u8g2.drawLine(calcgpslon2lcdposx(longitude1), calcgpslat2lcdposy(latitude1), calcgpslon2lcdposx(longitude), calcgpslat2lcdposy(latitude)); //draw distance line
  }

  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.setCursor(0, 5);
  u8g2.print(latitude, 6);
  u8g2.print(' ');
  u8g2.print(longitude, 6);
  u8g2.print('/');
  u8g2.print(gpsmeasuredistance(latitude1, longitude1, latitude, longitude), 3); u8g2.print("km");
  u8g2.setFont(u8g2_font_6x12_tr);
}

void initsi5351vfo() {
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  si5351.set_correction(90200, SI5351_PLL_INPUT_XO);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.set_freq((uint64_t)100000000.0 * vfofrequency, SI5351_CLK0);
}

void si5351setfrequenecy(float frequency) {
  uint64_t freqlong = 100000000.0 * frequency;
  si5351.set_freq(freqlong, SI5351_CLK0);
}

void initnrf24() {
  nrf24.begin();
  nrf24.setPALevel(RF24_PA_MAX);
  
  nrf24.openWritingPipe(nrf24addresses[1]);
  nrf24.openReadingPipe(1,nrf24addresses[1]);

  nrf24.startListening();

  nrf24controlrxbuf.battvoltage = 0; //reset
}
