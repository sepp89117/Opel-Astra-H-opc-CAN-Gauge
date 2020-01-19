/*
   Autor: Sebastian Balzer
   Version: 1.0
   Date: 19.01.2020

   Hardware: Teensy 4.0, Waveshare CAN Board SN65HVD230, ILI9341 320x240 TFT with XPT2046 Touch, LM2596 DC/DC Converter - set 5.0 Volt out
   Car: Opel Astra H opc
   Interface: HSCAN 500 kBaud (Pin 6 and 14 on OBD connector)
*/

/* TODO
    Replace all String with char
    Acoustic alarm when limit values are exceeded
    Create an opcButton class with is-touched function
    Switch off/on ESP by pressing the opc logo in the top right corner
    Test read out several trouble codes
*/

#include <XPT2046_Touchscreen.h>
#include "SPI.h"
#include "ILI9341_t3.h"
#include <font_Arial.h>
#include <opcNumFont.h>
#include <FlexCAN_T4.h>
#include "picture.c"
#include "TeensyThreads.h"

class info {
  public:
    String Text;
    uint16_t TextColor;
    int Prio;
    int Dur;

    info(String text, uint16_t color, int prio, int dur) {
      if (text.length() >= 27) { //wenn Text zu lang für Infozeile ist wird er gekürzt
        Text = text.substring(0, 26);
      } else {
        Text = text;
      }
      TextColor = color;
      Prio = prio;
      Dur = dur;
    };

    info() {
      Text = "";
      TextColor = 0;
      Prio = 0;
      Dur = 0;
    };

    bool operator == (info cInfo) {
      if (Text == cInfo.Text) return true;
      else return false;
    };

    bool operator != (info cInfo) {
      if (Text != cInfo.Text) return true;
      else return false;
    };
};

//Threads
int tech2ThreadID;
int displayDataThreadID;
int heartBeatThreadID;

//Touchscreen config
#define CS_PIN  8
XPT2046_Touchscreen ts(CS_PIN);
#define TIRQ_PIN  2
// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 400
#define TS_MINY 240
#define TS_MAXX 3940
#define TS_MAXY 3780

//TFT config
#define TFT_DC       9
#define TFT_CS      10
#define TFT_RST    255  // 255 = unused, connect to 3.3V
#define TFT_MOSI    11
#define TFT_SCLK    13
#define TFT_MISO    12
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

//Can config
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_32> opcCan;
uint32_t canBaud = (uint32_t)500000;
int responseTimeout = 1000;
int lastResponseTime = responseTimeout * 2;

//Definition of CAN-IDs
#define CANID_REPLY     0x7E8
#define CANID_REQUEST   0x7E0
#define CANID_DATAREPLY 0x5E8

//Display Views
#define HOME_VIEW  0
#define FC_VIEW    1
#define MAX_VIEW   2
#define DASH_VIEW  3
#define DASH_VIEW2 4
int actualView = HOME_VIEW;

//Styles
uint16_t valueTextColor = 0xFFFF; //weiß
uint16_t titelTextColor = 0xF7EF; //hellgelb
uint16_t backColor = ILI9341_BLACK; //schwarz
int buttonWidth = 84;

//Motordaten-Variablen
int ect = 0;
float vBatt = 0;
int boost = 0;
int iat = 0;
float maf = 0;
float mafVolt = 0;
int ign = 0;
int sft = 0;
int fcs = 0;
int rpm = 0;
float inj = 0.0f;
bool isMilOn = false;
//berechnete Daten
int injAuslast = 0;
int power = 0;
float consume = 0;
float afr = 0;
//Alte Motordaten-Variablen für das überschreiben auf dem Display
float oldvBatt = 1;
int oldfcs = 0;
int oldect = 1;
int oldboost = 1;
int oldiat = 1;
float oldmaf = 1;
float oldmafVolt = 1;
int oldign = 1;
int oldsft = 1;
int oldrpm = 1;
float oldinj = 1.0f;
int oldpower = 1;
int oldmoment = 1;
int oldinjAuslast = 1;
bool oldisMilOn = true;
float oldconsume;
float oldafr;
//max-reached-vals
int rMAXect = 0;
int rMAXboost = 0;
int rMAXiat = 0;
int rMAXmaf = 0;
int rMAXign = 0;
int rMAXrpm = 0;
float rMAXinj = 0.0f;
int rMAXinjAuslast = 0;
int rMAXpower = 0;
int rMAXmoment = 0;
int rMAXconsume = 0;
float rMAXafr = 0;

//Programm-Variablen
char bcVersion[4] = "1.0";
bool ecuConnected = false;
bool listSet = false;
bool response = false;
int noResponseCount = 0;
info actualInfo;
info oldInfo;
int infoWritten;
bool dashViewLoad = true;
bool sportActive = false;
bool answered = false;

void setup() {
  Serial.begin(9600);

  //init TFT
  tft.begin();
  tft.fillScreen(backColor);
  tft.setRotation(3);
  ts.setRotation(3);
  Serial.println("TFT init okay");

  //init CAN
  opcCan.begin();
  opcCan.setBaudRate(canBaud);
  opcCan.setClock(CLK_60MHz);
  for (int i = 0; i < 10; i++) {
    opcCan.setMB(FLEXCAN_MAILBOX(i), RX, STD);
  }
  opcCan.enableFIFO();
  opcCan.enableFIFOInterrupt();
  opcCan.setFIFOFilter(REJECT_ALL);
  opcCan.setFIFOFilter(0, CANID_DATAREPLY, STD);
  opcCan.setFIFOFilter(1, CANID_REPLY, STD);
  opcCan.onReceive(handleDataMsg);
  Serial.print("CAN init okay with ");
  opcCan.mailboxStatus();

  //init touch
  if (!ts.begin()) {
    Serial.println("Couldn't start touchscreen controller");
  } else {
    Serial.println("Touchscreen init okay");
  }

  //starte Kommunikation mit ECU
  tech2ThreadID = threads.addThread(tech2);

  //show bootlogo
  animateBoot();

  //init Homeview
  tft.fillScreen(backColor);

  //sport-button grey
  tft.fillRect(282, 0, 16, 8, 0x7BCF);
  tft.drawRect(282, 0, 16, 8, 0x3CDF);
  tft.drawRect(281, 0, 18, 8, 0x3CDF);
  tft.writeRect(264, 12, 52, 20, (uint16_t*)opc20px);

  drawHomeView();

  //start threads
  displayDataThreadID = threads.addThread(displayData);
  threads.addThread(touch);
}

void animateBoot() {
  tft.setCursor(0, 0);
  tft.setFont(Arial_12);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("Author: S. Balzer");
  tft.print("Version: ");
  tft.println(bcVersion);

  tft.writeRect(28, 54, 263, 133, (uint16_t*)opc); //x 28 y 54
  delay(3000);
}

void loop(void) {
  opcCan.events();
}

void touch() {
  threads.delay(200); //wait 200 ms before the first start, otherwise there will be problems
  while (1) {
    if (ts.touched()) {
      TS_Point p = ts.getPoint();
      //Serial.print((String)p.x + ", " + (String)p.y); //for Calibration
      p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
      p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
      //Serial.println(" = " + (String)p.x + ", " + (String)p.y); //for Calibration

      if (p.x >= 0 && p.x <= 320 && p.y >= 0 && p.y <= 240) { //Berührung im Bildbereich
        if (p.x >= (buttonWidth + 2) * 3  && p.y <= 34 && actualView != DASH_VIEW && actualView != DASH_VIEW2) {
          sportActive = !sportActive;
        }
        if (p.x <= buttonWidth && p.y <= 34) { //Wurde switch view / back button gedrückt?
          if (actualView == HOME_VIEW) {
            //switch to FC_VIEW
            actualView = FC_VIEW;
            refreshView();
          } else if (actualView == FC_VIEW) {
            //reset variables
            ecuConnected = false;
            listSet = false;
            tech2ThreadID = threads.addThread(tech2);
            actualView = HOME_VIEW;
            refreshView();
          } else if (actualView == MAX_VIEW || actualView == DASH_VIEW || actualView == DASH_VIEW2) {
            actualView = HOME_VIEW;
            refreshView();
          }
        } else if (p.x >= buttonWidth + 2 && p.x <= (buttonWidth + 2) * 2  && p.y <= 34) { //Wurde Max gedrückt?
          if (actualView == HOME_VIEW) {
            //switch to MAX_VIEW
            actualView = MAX_VIEW;
            refreshView();
          } else if (actualView == MAX_VIEW) {
            rMAXect = 0;
            rMAXboost = 0;
            rMAXiat = 0;
            rMAXmaf = 0;
            rMAXign = 0;
            rMAXrpm = 0;
            rMAXinj = 0.0f;
            rMAXinjAuslast = 0;
            rMAXpower = 0;
            rMAXmoment = 0;

            drawMaxView();
          } else if (actualView == FC_VIEW) {
            clearFCs();
            fcs = 0;
          }
        } else if (p.x >= 2 * buttonWidth + 4 && p.x <= 3 * buttonWidth + 4 && p.y <= 34) { //Wurde Dash gedrückt?
          if (actualView == HOME_VIEW) {
            Serial.println("PUSH Show Dash");
            //switch to DASH_VIEW
            actualView = DASH_VIEW;
            refreshView();
          }
        } else if (p.x >= 319 - buttonWidth && p.y <= 34) {
          if (actualView == DASH_VIEW) {
            actualView = DASH_VIEW2;
            refreshView();
          } else if (actualView == DASH_VIEW2) {
            actualView = DASH_VIEW;
            refreshView();
          }
        }
        threads.delay(350); //avoids bouncing
      }
    }
    threads.delay(1); //delay for touch controller
    threads.yield();
  }
}

void displayData() {
  int xPos;
  int yPos;
  while (1) {
    consume = getConsum();
    afr = getAFR();
    if (consume > rMAXconsume) rMAXconsume = consume;
    if (actualView != DASH_VIEW && actualView != DASH_VIEW2) {
      if (sportActive) {
        tft.fillRect(282, 0, 16, 8, ILI9341_RED);
        tft.drawRect(282, 0, 16, 8, 0x3CDF);
        tft.drawRect(281, 0, 18, 8, 0x3CDF);
      } else {
        tft.fillRect(282, 0, 16, 8, 0x7BCF);
        tft.drawRect(282, 0, 16, 8, 0x3CDF);
        tft.drawRect(281, 0, 18, 8, 0x3CDF);
      }
    }
    if (actualView == HOME_VIEW) {
      drawInfo(actualInfo);
      //Fehlercodes Button
      if (isMilOn != oldisMilOn) {
        oldisMilOn = isMilOn;
        if (oldisMilOn == false) {
          drawOpcButton(0, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"DTCs");
        } else {
          drawOpcButton(0, 0, buttonWidth, 34, ILI9341_RED, (char*)"DTCs");
        }
      }

      //--- Batteriespannung info ---
      if (vBatt != oldvBatt) {
        uint16_t vBattColor = ILI9341_WHITE;
        if (vBatt <= 9.0) {
          vBattColor = ILI9341_RED;
        } else if (vBatt > 9.0 && vBatt < 12.0) {
          vBattColor = ILI9341_YELLOW;
        } else {
          vBattColor = ILI9341_GREEN;
        }
        info thisInfo("vBatt = " + (String)vBatt + " Volt", vBattColor, 2, 500);
        actualInfo = thisInfo;
      }

      //--- Motortemp ---
      yPos = 91;
      if (ect != oldect) {
        tft.setFont(opcNumFont_28);
        xPos = 63 - (tft.strPixelLen(intToChar(oldect)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(backColor);
        tft.println(oldect);
        oldect = ect;
        xPos = 63 - (tft.strPixelLen(intToChar(oldect)) / 2);
        tft.setCursor(xPos, yPos);
        if (oldect <= 65) {
          tft.setTextColor(0x3CDF);
        } else if (oldect <= 109 && oldect >= 66) {
          tft.setTextColor(valueTextColor);
        } else {
          tft.setTextColor(ILI9341_RED);
        }
        tft.println(oldect);
      }

      //--- Zündwinkel ---
      //xPos = 63;
      yPos = 151;
      if (ign != oldign) {
        tft.setFont(opcNumFont_28);
        xPos = 63 - (tft.strPixelLen(intToChar(oldign)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(backColor);
        tft.println(oldign);
        oldign = ign;
        xPos = 63 - (tft.strPixelLen(intToChar(oldign)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(valueTextColor);
        tft.println(oldign);
      }

      //--- Gemischanpassung ---
      //xPos = 63;
      yPos = 211;
      if (sft != oldsft) {
        tft.setFont(opcNumFont_28);
        xPos = 63 - (tft.strPixelLen(intToChar(oldsft)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(backColor);
        tft.println(oldsft);
        oldsft = sft;
        xPos = 63 - (tft.strPixelLen(intToChar(oldsft)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(valueTextColor);
        tft.println(oldsft);
      }

      //--- Ladedruck ---
      //xPos = 170;
      yPos = 91;
      if (boost != oldboost) {
        tft.setFont(opcNumFont_28);
        xPos = 170 - (tft.strPixelLen(intToChar(oldboost)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(backColor);
        tft.println(oldboost);
        oldboost = boost;
        xPos = 170 - (tft.strPixelLen(intToChar(oldboost)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(valueTextColor);
        tft.println(oldboost);
      }

      //--- Ladelufttemp ---
      //xPos = 170;
      yPos = 151;
      if (iat != oldiat) {
        tft.setFont(opcNumFont_28);
        xPos = 170 - (tft.strPixelLen(intToChar(oldiat)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(backColor);
        tft.println(oldiat);
        oldiat = iat;
        xPos = 170 - (tft.strPixelLen(intToChar(oldiat)) / 2);
        tft.setCursor(xPos, yPos);
        if (oldiat <= 59) {
          tft.setTextColor(valueTextColor);
        } else {
          tft.setTextColor(ILI9341_RED);
        }
        tft.println(oldiat);
      }

      //--- Luftmasse ---
      //xPos = 170;
      yPos = 211;
      if (maf != oldmaf) {
        tft.setFont(opcNumFont_28);
        xPos = 170 - (tft.strPixelLen(intToChar(oldmaf)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(backColor);
        tft.println(oldmaf, 0);
        oldmaf = maf;
        xPos = 170 - (tft.strPixelLen(intToChar(oldmaf)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(valueTextColor);
        tft.println(oldmaf, 0);
      }

      //--- Düsenauslastung ---
      //xPos = 277;
      yPos = 91;
      //injAuslast = inj * rpm / 1200;
      if (injAuslast != oldinjAuslast) {
        tft.setFont(opcNumFont_28);
        xPos = 277 - (tft.strPixelLen(intToChar(oldinjAuslast)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(backColor);
        tft.println((String)oldinjAuslast);
        oldinjAuslast = injAuslast;
        xPos = 277 - (tft.strPixelLen(intToChar(oldinjAuslast)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(valueTextColor);
        tft.println((String)oldinjAuslast);
      }

      //--- Leistung ---
      //xPos = 277;
      yPos = 151;
      if (power != oldpower) {
        tft.setFont(opcNumFont_28);
        xPos = 277 - (tft.strPixelLen(intToChar(oldpower)) / 2);
        tft.setCursor(xPos, yPos);
        //tft.fillRect(160,175,160,28,ILI9341_BLACK);
        tft.setTextColor(backColor);
        tft.println((String)oldpower);
        oldpower = power;
        xPos = 277 - (tft.strPixelLen(intToChar(oldpower)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(valueTextColor);
        tft.println((String)oldpower);
      }

      //--- Drehmoment ---
      //xPos = 277;
      yPos = 211;
      int rpmS = rpm / 60;
      int moment = (power / 1.36 * 1000) / (2 * 3.1415926 * rpmS);
      if (moment != oldmoment) {
        if (moment > rMAXmoment)rMAXmoment = moment;
        tft.setFont(opcNumFont_28);
        xPos = 277 - (tft.strPixelLen(intToChar(oldmoment)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(backColor);
        tft.println((String)oldmoment);
        oldmoment = moment;
        xPos = 277 - (tft.strPixelLen(intToChar(oldmoment)) / 2);
        tft.setCursor(xPos, yPos);
        tft.setTextColor(valueTextColor);
        tft.println((String)oldmoment);
      }

      oldfcs = fcs;

      threads.delay(200);
    } else if (actualView == FC_VIEW) {
      //Fehlercodes anzeigen

      threads.delay(500);
    } else if (actualView == MAX_VIEW) {
      drawMaxView();
      threads.delay(5000);
    } else if (actualView == DASH_VIEW) {
      drawDashView();
      threads.delay(20);
    } else if (actualView == DASH_VIEW2) {
      drawDashView2();
      threads.delay(20);
    }
    threads.yield();
  }
}

char bufInt[10]; //global variable created because malloc cannot be used here
char* intToChar(int value) {
  itoa(value, bufInt, 10);

  return bufInt;
}

char bufFloat[10]; //global variable created because malloc cannot be used here
char* floatToChar(float value) {
  byte len = 4; if (value >= 10) len = 5; if (value >= 100) len = 6; if (value >= 1000) len = 7;
  dtostrf(value, len, 2, bufFloat);
  return bufFloat;
}

void refreshView() {
  threads.kill(displayDataThreadID);

  //clear screen
  tft.fillScreen(backColor);

  if (actualView == HOME_VIEW) {
    tft.writeRect(264, 12, 52, 20, (uint16_t*)opc20px);
    oldect = 1;
    oldboost = 1.0f;
    oldiat = 1;
    oldmaf = 1;
    oldign = 1;
    oldsft = 1;
    oldfcs = 0;
    oldinj = 1;
    oldrpm = 1;
    oldpower = 1;
    oldmoment = 1;
    oldinjAuslast = 1;

    drawHomeView();
    info emptyInfo;
    oldInfo = emptyInfo;
  } else if (actualView == FC_VIEW) {
    tft.writeRect(264, 12, 52, 20, (uint16_t*)opc20px);
    tft.fillRect(0, 35, 320, 205, backColor);
    drawOpcButton(0, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"Home");
    drawOpcButton(buttonWidth + 3, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"Del");

    tft.setCursor(0, 38);
    tft.setFont(Arial_12);
    tft.setTextColor(valueTextColor);

    requestFCs();
  } else if (actualView == MAX_VIEW) {
    tft.writeRect(264, 12, 52, 20, (uint16_t*)opc20px);
    drawOpcButton(0, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"Home");
    drawOpcButton(buttonWidth + 3, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"Rst");

    drawMaxView();
  } else if (actualView == DASH_VIEW) {
    tft.writeRect(0, 17, 320, 77, (uint16_t*)dashBG);
    tft.fillRect(0, 94, 320, 240 - 95, backColor);
    drawOpcButton(0, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"Home");
    drawOpcButton(319 - buttonWidth, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"2>>");
    dashViewLoad = true;
    drawDashView();
  } else if (actualView == DASH_VIEW2) {
    tft.writeRect(0, 17, 320, 77, (uint16_t*)dashBG);
    tft.fillRect(0, 94, 320, 240 - 95, backColor);
    drawOpcButton(0, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"Home");
    drawOpcButton(319 - buttonWidth, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"<<1");
    dashViewLoad = true;
    drawDashView2();
  }
  displayDataThreadID = threads.addThread(displayData);
}

void drawMaxView() {
  //max textlength = 23 chars
  tft.fillRect(200, 45, 47, 195, backColor);
  tft.setFont(Arial_10);
  tft.setTextColor(ILI9341_WHITE);

  int y = 40;
  tft.setCursor(2, y);
  tft.print("Motortemperatur");
  tft.setCursor(200, y);
  tft.print(rMAXect);
  tft.setCursor(248, y);
  tft.print("*C");

  y += 19;
  tft.setCursor(2, y);
  tft.print("Saugrohrdruck");
  tft.setCursor(200, y);
  tft.print(rMAXboost);
  tft.setCursor(248, y);
  tft.print("kPa");

  y += 19;
  tft.setCursor(2, y);
  tft.println("Ladelufttemp.");
  tft.setCursor(200, y);
  tft.println(rMAXiat);
  tft.setCursor(248, y);
  tft.println("*C");

  y += 19;
  tft.setCursor(2, y);
  tft.println("Motordrehzahl");
  tft.setCursor(200, y);
  tft.println(rMAXrpm);
  tft.setCursor(248, y);
  tft.println("U / min");

  y += 19;
  tft.setCursor(2, y);
  tft.println("Zuend-Fruehverstellung");
  tft.setCursor(200, y);
  tft.println(rMAXign);
  tft.setCursor(248, y);
  tft.println("* vor OT");

  y += 19;
  tft.setCursor(2, y);
  tft.println("Luftmasse");
  tft.setCursor(200, y);
  tft.println(rMAXmaf);
  tft.setCursor(248, y);
  tft.println("kg / h");

  y += 19;
  tft.setCursor(2, y);
  tft.println("Injektorauslastung");
  tft.setCursor(200, y);
  tft.println(rMAXinjAuslast);
  tft.setCursor(248, y);
  tft.println("%");

  y += 19;
  tft.setCursor(2, y);
  tft.println("Einspritzzeit");
  tft.setCursor(200, y);
  tft.println(rMAXinj);
  tft.setCursor(248, y);
  tft.println("ms");

  y += 19;
  tft.setCursor(2, y);
  tft.println("Power");
  tft.setCursor(200, y);
  tft.println(rMAXpower);
  tft.setCursor(248, y);
  tft.println("PS");

  y += 19;
  tft.setCursor(2, y);
  tft.println("Drehmoment");
  tft.setCursor(200, y);
  tft.println(rMAXmoment);
  tft.setCursor(248, y);
  tft.println("Nm");

  y += 19;
  tft.setCursor(2, y);
  tft.println("Verbrauch");
  tft.setCursor(200, y);
  tft.println(rMAXconsume);
  tft.setCursor(248, y);
  tft.println("l / h");
}

void drawHomeView() {
  //Button
  drawOpcButton(0, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"DTCs");
  drawOpcButton(buttonWidth + 2, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"Max");
  drawOpcButton(2 * buttonWidth + 4, 0, buttonWidth, 34, ILI9341_BLACK, (char*)"Dash");

  tft.writeRect(0, 73, 23, 167, (uint16_t*)opcBar);
  tft.writeRect(107, 73, 23, 167, (uint16_t*)opcBar);
  tft.writeRect(214, 73, 23, 167, (uint16_t*)opcBar);

  tft.setCursor(27, 73);
  tft.setFont(Arial_9);
  tft.setTextColor(titelTextColor);
  tft.println("Motortemp.");

  tft.setCursor(27, 133);
  tft.setFont(Arial_9);
  tft.setTextColor(titelTextColor);
  tft.println("Zuendwinkel");

  tft.setCursor(27, 193);
  tft.setFont(Arial_9);
  tft.setTextColor(titelTextColor);
  tft.println("Gemischanp.");

  tft.setCursor(134, 73);
  tft.setFont(Arial_9);
  tft.setTextColor(titelTextColor);
  tft.println("Ladedruck");

  tft.setCursor(134, 133);
  tft.setFont(Arial_9);
  tft.setTextColor(titelTextColor);
  tft.println("Ladelufttemp.");

  tft.setCursor(134, 193);
  tft.setFont(Arial_9);
  tft.setTextColor(titelTextColor);
  tft.println("Luftmasse");

  tft.setCursor(241, 73);
  tft.setFont(Arial_9);
  tft.setTextColor(titelTextColor);
  tft.println("Duty Cycle");

  tft.setCursor(241, 133);
  tft.setFont(Arial_9);
  tft.setTextColor(titelTextColor);
  tft.println("Leistung");

  tft.setCursor(241, 193);
  tft.setFont(Arial_9);
  tft.setTextColor(titelTextColor);
  tft.println("Drehmoment");
}

void drawDashView() {
  if (ect != oldect || dashViewLoad) {
    oldect = ect;
    //ringMeter(val, min, max, x , y , rad, unit, crit, title, angle, maxVal, decis)
    ringMeter(oldect, 40, 120, 115, 23, 45, (char*)"*C", 100, (char*)"ECT", 120, rMAXect, 0); //Motortemp
  }

  if (boost != oldboost || dashViewLoad) {
    oldboost = boost;
    ringMeter(oldboost, 0, 160, 0, 41, 60, (char*)"kPa", 130, (char*)"Boost", 300, rMAXboost, 0); //Ladedruck
  }

  if (injAuslast != oldinjAuslast || dashViewLoad) {
    oldinjAuslast = injAuslast;
    ringMeter(oldinjAuslast, 0, 100, 199, 41, 60, (char*)"%", 85, (char*)"Injektor", 300, rMAXinjAuslast, 0); //Injektorauslastung
  }

  if (iat != oldiat || dashViewLoad) {
    oldiat = iat;
    ringMeter(oldiat, 0, 70, 98, 120, 62, (char*)"*C", 60, (char*)"IAT", 300, rMAXiat, 0); //Ladelufttemp
  }

  // Print consume
  int x = 47; //Mittelpunkt
  int y = 195; //Mittelpunkt
  int tXsize;

  if (consume != oldconsume || dashViewLoad) {
    tft.fillRect(x - 47, y - 7, 94, 22, backColor);
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_20);
    tft.setCursor(x - 27, y - 7);
    oldconsume = consume;
    tft.print(oldconsume, 1);

    tft.setTextColor(titelTextColor);
    tft.setFont(Arial_12);
    tXsize = tft.strPixelLen((char*)"Verbrauch");
    tft.setCursor(x - (tXsize / 2), y - 28);
    tft.print("Verbrauch");

    tXsize = tft.strPixelLen((char*)"l/h");
    tft.setCursor(x - (tXsize / 2), y + 22);
    tft.print("l/h");
  }

  x = 273;
  if (afr != oldafr || dashViewLoad) {
    tft.fillRect(x - 47, y - 7, 94, 22, backColor);
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_20);
    tft.setCursor(x - 28, y - 7);
    oldafr = afr;
    float lambda = oldafr / 14.7f;
    String gemisch;
    if (lambda > 1.1) {
      tft.setTextColor(ILI9341_RED);
      gemisch = "mager";
      tft.fillRect(x - 47, y + 22, 94, 22, backColor);
      tft.print(">1.1");
    } else if (lambda > 1.05 && lambda <= 1.1) {
      tft.setTextColor(ILI9341_RED);
      gemisch = "mager";
      tft.fillRect(x - 47, y + 22, 94, 22, backColor);
      tft.print(lambda, 2);
    } else if (lambda < 0.6) {
      tft.setTextColor(ILI9341_GREEN);
      gemisch = "fett";
      tft.fillRect(x - 47, y + 22, 94, 22, backColor);
      tft.print("<0.6");
    } else if (lambda < 0.95) {
      tft.setTextColor(ILI9341_GREEN);
      gemisch = "fett";
      tft.fillRect(x - 47, y + 22, 94, 22, backColor);
      tft.print(lambda, 2);
    } else if (lambda != lambda) {
      tft.setTextColor(ILI9341_WHITE);
      tft.fillRect(x - 47, y + 22, 94, 22, backColor);
      tft.print("---");
      gemisch = "na";
    } else {
      tft.fillRect(x - 47, y + 22, 94, 22, backColor);
      gemisch = "stoe.";
      tft.print(lambda, 1);
    }

    tft.setTextColor(titelTextColor);
    tft.setFont(Arial_12);
    tXsize = tft.strPixelLen((char*)"Lambda");
    tft.setCursor(x - (tXsize / 2), y - 28);
    tft.print("Lambda");


    tft.setCursor(x - 20, y + 22);
    tft.print(gemisch);
  }
  dashViewLoad = false;
}

void drawDashView2() {
  int tXsize;

  if (iat != oldiat || dashViewLoad) {
    oldiat = iat;
    //ringMeter(val, min, max, x , y , rad, unit, crit, title, angle, maxVal, decis)
    ringMeter(oldiat, 0, 70, 115, 23, 45, (char*)"*C", 60, (char*)"IAT", 120, rMAXiat, 0); //iat
  }

  if (rpm != oldrpm || dashViewLoad) {
    oldrpm = rpm;
    ringMeter(oldrpm, 0, 7000, 0, 41, 60, (char*)"1/min", 6000, (char*)"Drehzahl", 300, rMAXrpm, 0); //rpm
  }

  if (inj != oldinj || dashViewLoad) {
    oldinj = inj;
    ringMeter(oldinj, 0, 25, 199, 41, 60, (char*)"ms", 18, (char*)"Injektor", 300, rMAXinj, 1); //inj
  }

  if (maf != oldmaf || dashViewLoad) {
    oldmaf = maf;
    ringMeter(oldmaf, 0, 900, 98, 120, 62, (char*)"kg/h", 880, (char*)"Luftmasse", 300, rMAXmaf, 0); //maf
    //maf volt
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_12);
    oldmafVolt = mafVolt;
    tXsize = tft.strPixelLen(floatToChar(oldmafVolt));
    tft.fillRect(135, 225, 48, 12, backColor);
    tft.setCursor(155 - (tXsize / 2), 225);
    tft.print(oldmafVolt);
    tft.print("V");
  }

  // Print Fruehzuend
  int x = 47; //Mittelpunkt
  int y = 195; //Mittelpunkt

  if (ign != oldign || dashViewLoad) {
    tft.fillRect(x - 47, y - 7, 94, 22, backColor);
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_20);
    oldign = ign;
    tXsize = tft.strPixelLen(intToChar(oldign));
    tft.setCursor(x - (tXsize / 2), y - 7);
    tft.print(oldign);

    tft.setTextColor(titelTextColor);
    tft.setFont(Arial_12);
    tXsize = tft.strPixelLen((char*)"Fruehzuend");
    tft.setCursor(x - (tXsize / 2), y - 28);
    tft.print("Fruehzuend");

    tXsize = tft.strPixelLen((char*)"* vor OT");
    tft.setCursor(x - (tXsize / 2), y + 22);
    tft.print("* vor OT");
  }

  x = 273;
  if (injAuslast != oldinjAuslast || dashViewLoad) {
    tft.setFont(Arial_20);
    oldinjAuslast = injAuslast;
    tXsize = tft.strPixelLen(intToChar(oldinjAuslast));
    tft.setCursor(x - (tXsize / 2), y - 7);
    if (oldinjAuslast > 85) tft.setTextColor(ILI9341_RED);
    else                    tft.setTextColor(ILI9341_WHITE);
    tft.fillRect(x - 47, y - 7, 94, 22, backColor);
    tft.print(oldinjAuslast);

    tft.setTextColor(titelTextColor);
    tft.setFont(Arial_12);
    tXsize = tft.strPixelLen((char*)"Duty Cycle");
    tft.setCursor(x - (tXsize / 2), y - 28);
    tft.print("Duty Cycle");

    tXsize = tft.strPixelLen((char*)"%");
    tft.setCursor(x - (tXsize / 2), y + 22);
    tft.print("%");
  }
  dashViewLoad = false;
}

void drawOpcButton(int x, int y, int w, int h, uint16_t textColor, char* text) {
  tft.setFont(Arial_20);
  int tXsize = tft.strPixelLen(text);

  //Background
  tft.fillRect(x + 2, y + 2, w - 4, h - 4, 0xFFFF);
  //Frame
  tft.drawRoundRect(x, y, w, h, 3, 0x3CDF);
  tft.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 2, 0x3CDF);
  //Text
  tft.setCursor(x + (w / 2) - (tXsize / 2), y + 6);
  tft.setTextColor(textColor);
  tft.print(text);
}

void drawInfo(info theInfo) {
  if (theInfo.Prio >= oldInfo.Prio || (int)millis() >= oldInfo.Dur + infoWritten) { //wenn neu info-prio höher als alte info-prio, oder alte Info abgelaufen
    if (actualView == HOME_VIEW && theInfo != oldInfo) { //wenn homeView & neue Info
      oldInfo = theInfo;
      tft.fillRect(5, 38, 315, 20, backColor);
      tft.setFont(Arial_16);
      tft.setCursor(5, 38);
      tft.setTextColor(theInfo.TextColor);
      tft.print(theInfo.Text);
      infoWritten = millis();
    }
  }
}

float getConsum() {
  int injSize = 470;  // [ccm/min] @ injPress
  int injPress = 300; // injSize @ [kPa]
  float newSize;

  if (rpm > 1000) {
    newSize = injSize * sqrt((injPress + boost) / injPress);
  } else {
    newSize = injSize * sqrt((injPress - 70.0f) / injPress); // -70kPa Saugrogrdruck im Leerlauf
  }
  float consum = (inj - 0.6f) * rpm * newSize * 0.12f / 60000.0f; // 0,6 = verzögerung beim Öffnen
  if (consum < 0) consum = 0.0f;

  return consum;
}

float getAFR() {
  float afrC = 14.7f;
  float rohBenzin = 0.748f;
  float kgBenzin = rohBenzin * consume;

  afrC = maf / kgBenzin;

  return afrC;
}

void ringMeter(float value, int vmin, int vmax, int x, int y, int r, char *units, int vCritical, char *title, int angle, int maxVal, int decis) {
  x += r; y += r;   // Calculate coords of centre of ring

  int w = r / 4;    // Width of outer ring is 1/4 of radius

  angle = angle / 2;  // Half the sweep angle of meter (300 degrees)

  uint16_t text_colour = ILI9341_WHITE; // To hold the text colour

  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
  int vC = map(vCritical, vmin, vmax, -angle, angle); // Map the vCritical to an angle vC
  int vMax = map(maxVal, vmin, vmax, -angle, angle); // Map the maxVal to an angle vMax

  byte seg = 2; // Segments are 5 degrees wide = 60 segments for 300 degrees
  byte inc = 2; // Draw segments every 5 degrees, increase to 10 for segmented ring

  // Draw background blocks every inc degrees
  for (int i = -angle; i < angle; i += inc) {
    w = map(i, -angle, angle, r / 7, r / 4); // Map the value to an angle v

    // Calculate pair of coordinates for segment start
    float sx = cos(((float)i - 90.0) * 0.01745329251);
    float sy = sin(((float)i - 90.0) * 0.01745329251);//0.01745329251

    uint16_t x0 = sx * (r - w) + x; //innerer Ring
    uint16_t y0 = sy * (r - w) + y; //
    uint16_t x1 = sx * r + x;       //äuserer Ring
    uint16_t y1 = sy * r + y;       //

    // Calculate pair of coordinates for segment end
    float sx2 = cos(((float)i + seg - 90.0) * 0.01745329251);
    float sy2 = sin(((float)i + seg - 90.0) * 0.01745329251);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;


    if (i < v) { //Anzeigewert
      if (i > vC) { //Grenzwertüberschreitung
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, ILI9341_RED);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, ILI9341_RED);
        text_colour = ILI9341_RED;
      } else {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, 0x3CDF);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, 0x3CDF);
        text_colour = ILI9341_WHITE;
      }
    } else {
      //Hintergrund
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, ILI9341_GREY);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, ILI9341_GREY);
    }
    if (i == vMax || i == vMax + 1) {
      if (i > vC) { //Grenzwertüberschreitung
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, ILI9341_RED);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, ILI9341_RED);
      } else {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, ILI9341_ORANGE);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, ILI9341_ORANGE);
      }
    }
  }

  // Convert value to a string
  char buf[10];
  byte len = 3; if (value > 99) len = 4;
  dtostrf(value, len, decis, buf);

  tft.setTextColor(text_colour);
  int tXsize;

  // Print value
  tft.setFont(Arial_20);
  tXsize = tft.strPixelLen(buf);
  tft.fillRect(x - 40, y - 7, 80, 22, backColor);
  tft.setCursor(x - (tXsize / 2) - 5, y - 7);
  tft.print(buf);

  tft.setTextColor(titelTextColor);
  tft.setFont(Arial_12);
  tXsize = tft.strPixelLen(title);
  tft.setCursor(x - (tXsize / 2), y - 28);
  tft.print(title);

  tXsize = tft.strPixelLen(units);
  tft.setCursor(x - (tXsize / 2), y + 22);
  tft.print(units);
}

//-------------------------------------------------------------//
//------------------------OBD-Functions------------------------//
//-------------------------------------------------------------//
void tech2() {
  int sendingTime;
  int timeOut;
  CAN_message_t can_MsgRx;

  while (1) { //Thread-Schleife
    if (ecuConnected == false) { //wenn nicht verbunden, sende "7E0 01 20 0 0 0 0 0 0" bis "7E8 01 60 0 0 0 0 0 0" geantwortet wird
      Serial.println("Nicht verbunden.");
      while (ecuConnected == false) {//Solange nicht verbunden-Schleife
        Serial.println("Sende Verbindungsanfrage...");

        info thisActualInfo("Sende Verbindungsanfrage...", ILI9341_WHITE, 3, 500);
        actualInfo = thisActualInfo;

        sendEcuData(0x01, 0x20, 0, 0, 0, 0, 0, 0);
        sendingTime = millis();

        timeOut = 500;
        //recive response
        while ((int)millis() <= sendingTime + timeOut) { //solange kein timeout, auf Antwort warten

        }
        if (ecuConnected == true) {
          Serial.println("Verbunden mit ECU.");

          info thisActualInfo("Verbunden mit ECU.", ILI9341_WHITE, 1, 500);
          actualInfo = thisActualInfo;

          threads.delay(15); //Warte 15ms, danach wird die Liste konfiguriert
          sendingTime = 0;
        }
      }
    } else { //wenn verbunden
      if (listSet == false) { //wenn Datenliste noch nicht konfiguriert
        sendEcuData(0x10, 0x09, 0xAA, 0x04, 0x10, 0x11, 0x12, 0x16); //Pakete 10, 11, 12, 16
        threads.delay(15); //Warte 15ms
        sendEcuData(0x21, 0x14, 0x19, 0x03, 0x00, 0x00, 0x00, 0x00); //Pakete 14, 19, 3
        listSet = true;

        //Starte Herzschlag / Verbindungsüberwachung
        heartBeatThreadID = threads.addThread(heartBeat);
      }
      else {
        while (ecuConnected) {

        }
      }
    }
  }
  threads.yield();
}

void handleDataMsg(const CAN_message_t &can_MsgRx) {
  if (can_MsgRx.id == CANID_DATAREPLY) { //Motordaten empfangen von 0x5E8
    switch (can_MsgRx.buf[0]) { //Datenpaketnummer
      case 0x03:
        isMilOn = can_MsgRx.buf[7];
        break;

      case 0x10:
        vBatt = can_MsgRx.buf[1] / 10.0f;
        rpm = ((can_MsgRx.buf[2] * 256.0f) + can_MsgRx.buf[3]) / 4.0f;

        if (rpm > rMAXrpm)rMAXrpm = rpm;
        break;

      case 0x11:
        iat = can_MsgRx.buf[4] - 40;
        ect = can_MsgRx.buf[2] - 40;

        if (iat > rMAXiat)rMAXiat = iat;
        if (ect > rMAXect)rMAXect = ect;
        break;

      case 0x12:
        maf = ((can_MsgRx.buf[2] * 256.0f) + can_MsgRx.buf[3]) / 100.0f * 3.6f;
        mafVolt = can_MsgRx.buf[1] / 51.0f;
        boost = can_MsgRx.buf[7] - 101;
        power = maf * 0.383;


        if (power > rMAXpower)rMAXpower = power;
        if (maf > rMAXmaf)rMAXmaf = maf;
        if (boost > rMAXboost)rMAXboost = boost;
        break;

      case 0x16:
        ign = can_MsgRx.buf[2] - 36;

        if (ign > rMAXign)rMAXign = ign;
        break;

      case 0x14:
        //Injektor
        inj = can_MsgRx.buf[7] / 10.0f;

        injAuslast = inj * rpm / 1200.0f;

        if (injAuslast > rMAXinjAuslast)rMAXinjAuslast = injAuslast;
        if (inj > rMAXinj)rMAXinj = inj;
        break;

      case 0x19:
        //Lambdasonde
        sft = (can_MsgRx.buf[1] - 128.0f) / 1.28;
        break;

      case 0x81:
        answered = true;
        //Fehlercodes
        if (can_MsgRx.buf[4] == 0xFF) {
          //Ende der Übertragung
        } else {
          fcs++; //Fehleranzahl zählen
          //          Serial.println(can_MsgRx.id, HEX);
          //          Serial.println(can_MsgRx.buf[0], HEX);
          //          Serial.println(can_MsgRx.buf[1], HEX);
          //          Serial.println(can_MsgRx.buf[2], HEX);
          //          Serial.println(can_MsgRx.buf[3], HEX);
          //          Serial.println(can_MsgRx.buf[4], HEX);
          //          Serial.println(can_MsgRx.buf[5], HEX);
          //          Serial.println(can_MsgRx.buf[6], HEX);
          //          Serial.println(can_MsgRx.buf[7], HEX);
          getFCs(can_MsgRx.buf, 0);
        }
        break;
      case 0xA9: //TODO test 0xA9
        answered = true;
        //Fehlercodes
        if (can_MsgRx.buf[4] == 0xFF) {
          //ende der Übertragung
        } else {
          fcs++; //Fehleranzahl zählen
          getFCs(can_MsgRx.buf, 1);
        }
        break;
    }
  } else if (can_MsgRx.id == CANID_REPLY) {
    if (can_MsgRx.buf[0] == 0x01) {
      ecuConnected = true;
    }
    switch (can_MsgRx.buf[1]) {
      case 0x7E: //Antwort auf Herzschlag
        response = true;
        break;
    }
  }
}

void heartBeat() {
  int heartBeatTimeOut = 419; //alle 419ms Herzschlag senden
  threads.delay(heartBeatTimeOut);
  while (ecuConnected) {
    sendEcuData(0x01, 0x3E, 0, 0, 0, 0, 0, 0);
    response = false;
    threads.delay(heartBeatTimeOut);

    if (response == false) { //alles zurücksetzen
      noResponseCount++;
      Serial.println("Keine Antwort von ECU zum " + (String)noResponseCount + ". mal!");
      if (noResponseCount >= 6) {
        Serial.println("Verbindung getrennt!");

        info thisActualInfo("Verbindung getrennt!", ILI9341_WHITE, 0, 500);
        actualInfo = thisActualInfo;

        ect = 0;
        boost = 0.0;
        iat = 0;
        maf = 0;
        ign = 0;
        sft = 0;
        fcs = 0;
        rpm = 0;
        inj = 0;
        vBatt = 0;
        injAuslast = 0;
        power = 0;
        listSet = false;
        ecuConnected = false;
        threads.kill(heartBeatThreadID);
        break;
      }
    } else { //prog laufen lassen
      noResponseCount = 0;
    }
    threads.yield();
  }
}

void sendEcuData(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7) {
  CAN_message_t TxMsg;
  TxMsg.buf[0] = byte0;
  TxMsg.buf[1] = byte1;
  TxMsg.buf[2] = byte2;
  TxMsg.buf[3] = byte3;
  TxMsg.buf[4] = byte4;
  TxMsg.buf[5] = byte5;
  TxMsg.buf[6] = byte6;
  TxMsg.buf[7] = byte7;
  TxMsg.flags.extended = 0;
  TxMsg.flags.remote = 0;
  TxMsg.flags.overrun  = 0;
  TxMsg.flags.reserved = 0;
  TxMsg.len = 8;
  TxMsg.id = CANID_REQUEST;

  //send request
  if (!opcCan.write(TxMsg)) {
    Serial.println("Senden fehlgeschlagen");
  }
}

void clearFCs() {
  fcs = 0;
  sendEcuData(0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  if (actualView == FC_VIEW) {
    //clear screen
    tft.fillRect(0, 35, 320, 240, backColor);
    tft.setCursor(0, 38);
    tft.setFont(Arial_12);
    tft.setTextColor(valueTextColor);
    tft.println("Loesche Fehlercodes...");

    threads.delay(2000); //Warte 2sec

    tft.fillRect(0, 35, 320, 240, backColor);
    tft.setCursor(0, 38);
    tft.setFont(Arial_12);
    tft.setTextColor(valueTextColor);

    //request DTCs
    sendEcuData(0x03, 0xA9, 0x81, 0x12, 0x00, 0x00, 0x00, 0x00);
    int requestTime = millis();
    answered = false;

    //recive with timeout
    CAN_message_t can_MsgRx;
    while ((int)millis() < requestTime + 1000) {

    }
    Serial.println("answered = " + (String)answered);
    if (fcs <= 0) { //wenn keine Fehlercodes, Info anzeigen
      if (actualView == FC_VIEW) {
        tft.println("Keine Fehlercodes");
      }
      info thisActualInfo("Keine Fehlercodes", ILI9341_WHITE, 3, 5000);
      actualInfo = thisActualInfo;
    } else {
      info thisActualInfo((String)fcs + " Fehlercodes", ILI9341_RED, 3, 5000);
      actualInfo = thisActualInfo;
    }
    if (actualView == FC_VIEW && answered == false) {
      tft.println("Keine Rueckmeldung von ECU");
    }
  }
}

void requestFCs() {
  // kill tech2- and heartbeat-thread
  threads.kill(tech2ThreadID);
  threads.kill(heartBeatThreadID);

  threads.delay(25);

  // Disconnect Datamonitor
  sendEcuData(0x02, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  threads.delay(50);

  //request DTCs
  fcs = 0;
  sendEcuData(0x03, 0xA9, 0x81, 0x12, 0x00, 0x00, 0x00, 0x00);
  answered = false;

  threads.delay(1000);

  if (fcs <= 0) { //wenn keine Fehlercodes, Info anzeigen
    if (actualView == FC_VIEW) {
      tft.println("Keine Fehlercodes");
    }
    info thisActualInfo("Keine Fehlercodes", ILI9341_WHITE, 3, 5000);
    actualInfo = thisActualInfo;
  } else {
    info thisActualInfo((String)fcs + " Fehlercodes", ILI9341_RED, 3, 5000);
    actualInfo = thisActualInfo;
  }
  if (actualView == FC_VIEW && answered == false) {
    tft.println("Keine Rueckmeldung von ECU");
  }
}

void getFCs(const byte buff[], const int stat) {
  if (actualView == FC_VIEW) {

    tft.print(intToChar(fcs)); //Fehlernummer (1. , 2. , 3.)
    tft.print(". "); //Fehlernummer (1. , 2. , 3.)

    byte first = (byte)buff[1];
    byte seconda = (byte)buff[2];

    switch (first >> 6) {
      case 0: tft.print('P'); break;
      case 1: tft.print('C'); break;
      case 2: tft.print('B'); break;
      case 3: tft.print('U'); break;
      default: tft.print('-'); break;
    }

    tft.print((first >> 4) & 0x03, DEC);
    tft.print(first & 0x0f, HEX);
    tft.print(seconda >> 4, HEX);
    tft.print(seconda & 0x0f, HEX);

    if (stat == 0) {
      tft.print(" - INAKTIV");
    } else if (stat == 1) {
      tft.print(" - AKTIV");
    }
    tft.println();
  }
}
