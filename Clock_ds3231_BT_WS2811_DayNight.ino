
#include <Wire.h>
#include "ds3231.h"
#include "rtc_ds3231.h"

#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#define PIN 7

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);





#define BUFF_MAX 128
//#define SERIAL_OUTPUT

uint8_t time[8];
char recv[BUFF_MAX];
unsigned int recv_size = 0;
unsigned long prev, interval = 1000;
struct ts t;

struct _customtime {
  unsigned int uihour;
  unsigned int uiminute;
};

_customtime _Sleepy_time;
_customtime _Wakeup_time;
_customtime _Sleepy_time_W;
_customtime _Wakeup_time_W;
_customtime _Sleepy_time_WE;
_customtime _Wakeup_time_WE;

bool bIsItWeekEnd;
bool bSleepy_activated;
bool bTurnOnLED;
bool bDSTOn;
void MAJSleepAndWakeHours()
{
#ifdef SERIAL_OUTPUT
  Serial.println("MAJSleepAndWakeHours\n");
#endif
  
  if ((t.wday == 6)||(t.wday == 7))
  {
    _Sleepy_time.uihour=_Sleepy_time_WE.uihour;
    _Sleepy_time.uiminute=_Sleepy_time_WE.uiminute;
    _Wakeup_time.uihour=_Wakeup_time_WE.uihour;
    _Wakeup_time.uiminute=_Wakeup_time_WE.uiminute;
  }
  else
  {

    _Sleepy_time.uihour=_Sleepy_time_W.uihour;
    _Sleepy_time.uiminute=_Sleepy_time_W.uiminute;
    _Wakeup_time.uihour=_Wakeup_time_W.uihour;
    _Wakeup_time.uiminute=_Wakeup_time_W.uiminute;

  }
}
void IsSleepyOrWakeUpTime()
{
  if ((bSleepy_activated==false) && ((t.hour == _Sleepy_time.uihour) && (t.min >= _Sleepy_time.uiminute)))
  {
    //Allumer la lumière de nuit

    bSleepy_activated = true;
#ifdef SERIAL_OUTPUT
    Serial.println("Allumer la lumière de nuit");
#endif

  }
  else if ((bSleepy_activated==true) && ((t.hour == _Wakeup_time.uihour) && (t.min >= _Wakeup_time.uiminute)))
  {
    //Allumer la lumière de jour

    bSleepy_activated = false;
    MAJSleepAndWakeHours();
#ifdef SERIAL_OUTPUT
    Serial.println("Allumer la lumière de réveil");
#endif

  }

  /*if ((bSleepy_activated==false) && ((hour() >= _Wakeup_time.uihour+2) && ((hour() <= _Sleepy_time.uihour-2)&&(minute() >= _Sleepy_time.uiminute))))
  {
    bTurnOnLED = false;
    Serial.println("eteindre la LED");
  }
  else
  {
    bTurnOnLED = true;
    Serial.println("Allumer la LED");
  }*/




}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void setLED(bool bSleepy_Mode)
{
  if (bSleepy_Mode == true)
  {

    colorWipe(strip.Color(0, 1, 150), 50);

  }
  else
  {

    if (bTurnOnLED == true)
    {
      colorWipe(strip.Color(150, 20, 20), 50);
    }
    else
    {
     colorWipe(strip.Color(0, 0, 0), 50);
    }

  }
}
void setup()
{
    Serial.begin(9600);
    Wire.begin();
    DS3231_init(DS3231_INTCN);
    memset(recv, 0, BUFF_MAX);
    Serial.println("GET time");
	
	strip.begin();
	strip.show(); // Initialize all pixels to 'off'
	
	_Sleepy_time_W.uihour=20;
	_Sleepy_time_W.uiminute=30;
	_Wakeup_time_W.uihour=7;
	_Wakeup_time_W.uiminute=30;

	_Sleepy_time_WE.uihour=21;
	_Sleepy_time_WE.uiminute=00;
	_Wakeup_time_WE.uihour=8;
	_Wakeup_time_WE.uiminute=30;
	
	bSleepy_activated = false;
	bTurnOnLED = true;
	MAJSleepAndWakeHours();
	if ((t.hour >= _Sleepy_time.uihour) || (t.hour <= _Wakeup_time.uihour))
  {
    bSleepy_activated = true;

#ifdef SERIAL_OUTPUT
    Serial.println("Allumer la lumière de nuit");
#endif
  }
  else
  {
    bSleepy_activated = false;

#ifdef SERIAL_OUTPUT
    Serial.println("Allumer la lumière de réveil");
#endif
  }
  setLED(bSleepy_activated);
}

void loop()
{
    char in;
    char buff[BUFF_MAX];
    unsigned long now = millis();
    

    // show time once in a while
    if ((now - prev > interval) && (Serial.available() <= 0)) {
        DS3231_get(&t);
		IsSleepyOrWakeUpTime();
		setLED(bSleepy_activated);
        
        prev = now;
    }

    if (Serial.available() > 0) {
        in = Serial.read();

        if ((in == 10 || in == 13) && (recv_size > 0)) {
            parse_cmd(recv, recv_size);
            recv_size = 0;
            recv[0] = 0;
        } else if (in < 48 || in > 122) {;       // ignore ~[0-9A-Za-z]
        } else if (recv_size > BUFF_MAX - 2) {   // drop lines that are too long
            // drop
            recv_size = 0;
            recv[0] = 0;
        } else if (recv_size < BUFF_MAX - 2) {
            recv[recv_size] = in;
            recv[recv_size + 1] = 0;
            recv_size += 1;
        }

    }
}

void parse_cmd(char *cmd, int cmdsize)
{
    uint8_t i;
    uint8_t reg_val;
    char buff[BUFF_MAX];
    struct ts t;

    //snprintf(buff, BUFF_MAX, "cmd was '%s' %d\n", cmd, cmdsize);
    //Serial.print(buff);

    // TssmmhhWDDMMYYYY aka set time
    if (cmd[0] == 84 && cmdsize == 16) {
        //T355720619112011
        t.sec = inp2toi(cmd, 1);
        t.min = inp2toi(cmd, 3);
        t.hour = inp2toi(cmd, 5);
        t.wday = inp2toi(cmd, 7);
        t.mday = inp2toi(cmd, 8);
        t.mon = inp2toi(cmd, 10);
        t.year = inp2toi(cmd, 12) * 100 + inp2toi(cmd, 14);
        DS3231_set(t);
        Serial.println("OK");
    } else if (cmd[0] == 49 && cmdsize == 1) {  // "1" get alarm 1
        DS3231_get_a1(&buff[0], 59);
        Serial.println(buff);
    } else if (cmd[0] == 50 && cmdsize == 1) {  // "2" get alarm 1
        DS3231_get_a2(&buff[0], 59);
        Serial.println(buff);
    } else if (cmd[0] == 51 && cmdsize == 1) {  // "3" get aging register
        Serial.print("aging reg is ");
        Serial.println(DS3231_get_aging(), DEC);
    } else if (cmd[0] == 65 && cmdsize == 9) {  // "A" set alarm 1
        DS3231_set_creg(DS3231_INTCN | DS3231_A1IE);
        //ASSMMHHDD
        for (i = 0; i < 4; i++) {
            time[i] = (cmd[2 * i + 1] - 48) * 10 + cmd[2 * i + 2] - 48; // ss, mm, hh, dd
        }
        boolean flags[5] = { 0, 0, 0, 0, 0 };
        DS3231_set_a1(time[0], time[1], time[2], time[3], flags);
        DS3231_get_a1(&buff[0], 59);
        Serial.println(buff);
    } else if (cmd[0] == 66 && cmdsize == 7) {  // "B" Set Alarm 2
        DS3231_set_creg(DS3231_INTCN | DS3231_A2IE);
        //BMMHHDD
        for (i = 0; i < 4; i++) {
            time[i] = (cmd[2 * i + 1] - 48) * 10 + cmd[2 * i + 2] - 48; // mm, hh, dd
        }
        boolean flags[5] = { 0, 0, 0, 0 };
        DS3231_set_a2(time[0], time[1], time[2], flags);
        DS3231_get_a2(&buff[0], 59);
        Serial.println(buff);
    } else if (cmd[0] == 67 && cmdsize == 1) {  // "C" - get temperature register
        Serial.print("temperature reg is ");
        Serial.println(DS3231_get_treg(), DEC);
    } else if (cmd[0] == 68 && cmdsize == 1) {  // "D" - reset status register alarm flags
        reg_val = DS3231_get_sreg();
        reg_val &= B11111100;
        DS3231_set_sreg(reg_val);
    } else if (cmd[0] == 70 && cmdsize == 1) {  // "F" - custom fct
        reg_val = DS3231_get_addr(0x5);
        Serial.print("orig ");
        Serial.print(reg_val,DEC);
        Serial.print("month is ");
        Serial.println(bcdtodec(reg_val & 0x1F),DEC);
    } else if (cmd[0] == 71 && cmdsize == 1) {  // "G" - set aging status register
        DS3231_set_aging(0);
    } else if (cmd[0] == 83 && cmdsize == 1) {  // "S" - get status register
        Serial.print("status reg is ");
        Serial.println(DS3231_get_sreg(), DEC);
	} else if (cmd[0] == 84 && cmdsize == 1) {  // "T" - display
        snprintf(buff, BUFF_MAX, "%d.%02d.%02d %02d:%02d:%02d", t.year,
             t.mon, t.mday, t.hour, t.min, t.sec);
        Serial.println(buff);
	} else if (cmd[0] == 85 && cmdsize == 1) {  // "U" - Get Actual Clock Configuration
  		snprintf(buff, BUFF_MAX, "W:Sleep:%02d:%02d\nW:WakeUp:%02d:%02d\nWE:Sleep:%02d:%02d\nWE:WakeUp:%02d:%02d\n:", _Sleepy_time_W.uihour, _Sleepy_time_W.uiminute,_Wakeup_time_W.uihour, _Wakeup_time_W.uiminute,_Sleepy_time_WE.uihour, _Sleepy_time_WE.uiminute,_Wakeup_time_WE.uihour, _Wakeup_time_WE.uiminute);
        Serial.println(buff);
    } else if (cmd[0] == 86 && cmdsize == 6) {  // "W" Set Sleep W
        //VWMMHH
        /*for (i = 0; i < 4; i++) {
            time[i] = (cmd[2 * i + 1] - 48) * 10 + cmd[2 * i + 2] - 48; // mm, hh, dd
        }
        */
        
        Serial.println(buff);        
    } else {
        Serial.print("unknown command prefix ");
        Serial.println(cmd[0]);
        Serial.println(cmd[0], DEC);
    }
}
