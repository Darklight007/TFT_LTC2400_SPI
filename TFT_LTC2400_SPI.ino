//  LTC2400 24bit ADC Module
//
//  6 digit voltmeter
//  24bit ADC IC: LTC2400
 
#pragma GCC optimize ("-O3")

#include <SPI.h>

#include <Stdio.h>
#include <stdlib.h>
#include "Adafruit_GFX_AS.h"      // Core graphics library
#include "Adafruit_ILI9341_AS.h"  // Hardware-specific library

#define TWI_FREQ 4000000L
#include <Wire.h>

#define ILI9341_TRANSPARENT  0x80000000


#ifndef cbi
#define cbi(sfr, bit)     (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit)     (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/********************************************************************/
//ADC
//#define LTC_CS1 0       //D8   // LTC2400 Chip Select Pin  on Portb 2
const int LTC_CS1=0;

/********************************************************************/
//LCD
#define sclk 13  // Don't change
#define mosi 11  // Don't change
#define cs   10
#define dc   9
#define rst  7     // connected to vcc


//////////////////////////////////////////////////////////////
Adafruit_ILI9341_AS tft = Adafruit_ILI9341_AS(cs, dc, rst);       // Invoke custom library
///////////////////////////////////////////////////////////////////////////////

// Statistics
class runningStat {

  private:
    long m_n;
    double m_oldM, m_newM, m_oldS, m_newS;

  public:
    runningStat() : m_n(0) {}

    void Clear()    {
      m_n = 0;
    }

    void Push(double x)     {
      m_n++;
      // See Knuth TAOCP vol 2, 3rd edition, page 232
      if (m_n == 1)        {
        m_oldM = m_newM = x;
        m_oldS = 0.0;
      }
      else       {
        m_newM = m_oldM + (x - m_oldM) / m_n;
        m_newS = m_oldS + (x - m_oldM) * (x - m_newM);

        // set up for next iteration
        m_oldM = m_newM;
        m_oldS = m_newS;
      }
    }

    long NumDataValues() const     {
      return m_n;
    }

    double Mean() const     {
      return (m_n > 0) ? m_newM : 0.0;
    }

    double Variance() const     {
      return ( (m_n > 1) ? m_newS / (m_n - 1) : 0.0 );
    }

    double StandardDeviation() const     {
      return sqrt( Variance() );
    }
};

/********************************************************************/

//Global

//I2C
byte  bw[4];
int x = 0;

const int numBins = 120;
unsigned char hist[numBins];
double  bounds[numBins];
uint16_t bin = 0;
bool histFull = 0;
long int ltw2 = 0;         // ADC Data ling int


double volt,     volt_old = -1, volt_dec = 6;
double current,   current_old = -1, current_dec = 6;

double vMx = -9, vMx_old;
double vMn = 9, vMn_old;

double vAvg = 0, vAvg_old = -1;
long n = 0;

double sigmaX = 0;
double sigmaX2 = 0;
double sd = 0;

float v_ref = 5.000 ;     // Reference Voltage, 5.0 Volt for LT1021 or 3.0 for LP2950-3

long int ltw = 0;         // ADC Data ling int

int cnt = 1;              // counter
byte b0;                  //
byte sig;                 // sign bit flag

char tmp[12];
double timeKeeper = 0;

runningStat rs[2];

const int numReadings = 32;
double readings[numReadings];      // the readings from the analog input
int index = 0;                    // the index of the current reading
double total = 0;                  // the running total
double average = 0, average_old, averageFirst = 0, deltaAverage;           // the average
char chs = 0;
double s = 0; //standardDeviation =0;
char osc[200];



/********************************************************************/
void setup() {

  sbi (DDRB, LTC_CS1);      // LTC2400 CS HIGH
  drawFixedItems();

  //I2C
  // Start the I2C Bus as Slave on address 9
  Wire.begin(9);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  Serial.begin(250000);
}





void receiveEvent(int bytes) {
  ltw2 = 0;
  int sig = 0;
  for (int i = 0; i < 4; i++)
    bw[i] = Wire.read();  // read one character from the I2C

  if ((bw[3] & 0x20) == 0) sig = 1; // is input negative ?

  bw[3] &= 0x1F;                  // discard bit 25..31
  ltw2 |= bw[3]; ltw2  <<= 8;
  ltw2 |= bw[2]; ltw2  <<= 8;
  ltw2 |= bw[1]; ltw2  <<= 8;
  ltw2 |= bw[0];

  if (sig) ltw2 |= 0xf0000000;    // if input negative insert sign bit
  ltw2 = ltw2 / 16;                // scale result down , last 4 bits have no information
}
/********************************************************************/



/********************************************************************/
void loop() {
  //readAdc();

  if (chs > 1) chs = 0;

  if  (readAdc() || 1) {

    n++;


    // # sample
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
    tft.drawNumber( n, 2 + 8 * 2 + 3, 64, 1);

    //Raw value
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
    //  if (round(volt * pow(10, volt_dec)) != volt_old) {
    tft.drawFloat(volt, volt_dec, 26, 8, 7);

    tft.drawPixel(n % 200, osc[n % 200] + 140, ILI9341_BLACK);
    osc[n % 200] = volt * 200000;
    //tft.drawPixel(n % 200, osc[n % 200] + 140, ILI9341_GREEN);
    tft.drawFloat(osc[n % 200], 2, 26, 181, 2);

    if (volt > vMx) vMx = volt;
    if (volt < vMn) vMn = volt;

    // Averaging last 25 ***************************
    total = total - readings[index];
    readings[index] = volt;
    total = total + readings[index++];

    if (index >= numReadings)   index = 0;
    average = double(total / numReadings);
    // ********************************************
    //setting bins
    if  (n == numReadings) {
      if (1) {
        s = rs[0].StandardDeviation() / 20; //every 20 pixel as  sigma     //******************************************************************************
        averageFirst = rs[0].Mean();
        for (int i = 0; i < numBins; i++) {
          bounds[i] = averageFirst -  s * numBins / 2 + i * s  ;
          hist[i] = 0;
        }
      }
    }

    bin = histFind(volt);
    rs[0].Push(volt);
    //    rs[1].Push(average);



       if (n <= numReadings) return;

    // Soft reset
    //    if (n>100) asm volatile ("  jmp 0");
    
    //******************************************************************************************************
    if (!histFull) {
      if ((bin > 0) && (bin < numBins) && (hist[bin] < 120) ) {
        tft.drawPixel(bin + (320 - numBins) / 2 /*center*/, 180 - hist[bin]++, ILI9341_BLUE);
        tft.drawPixel(bin + (320 - numBins) / 2 /*center*/, 180 - hist[bin]  , ILI9341_WHITE);

      }
      else if (hist[bin] >= 120) {
        //        histFull=1;
        //        drawFixedItems();
        //        double s = rs[0].StandardDeviation() / 20; //every 20 pixel as  sigma     //******************************************************************************
        //        averageFirst = rs[0].Mean();
        //        for (int i = 0; i < numBins; i++) {
        //          bounds[i] = averageFirst -  s * numBins / 2 + i * s  ;
        //          hist[i] = 0;
        //        }
        //        rs[0].Clear();
        //        vMn = 2.0;
        //        vMx = -2.0;

        //        if  (n == numReadings) {
        //          double s = (vMx-vMn) / 32;
        //          for (int i = 0; i < numBins; i++) {
        //            bounds[i] = rs[0].Mean() -  s * numBins / 2 + i * s  ;
        //            hist[i] = 0;
        //          }
        //        }
        vMx_old = -8;
        vMn_old = 8;
        //              if (hist[bin] > 140) {
        for (int i = 0; i < numBins; i++) {
          hist[i] = hist[i] / 2;
          tft.drawLine(i  + (320 - numBins) / 2, 180, i + (320 - numBins) / 2, 180 -  hist[i]++, ILI9341_BLUE);
          tft.drawPixel(i + (320 - numBins) / 2, 180 -  hist[i], ILI9341_WHITE);
        }
        //              }
      }
    }
  }
  //**************************************





  //  Serial.println(volt);
  //    char tmp[10];
  dtostrf(volt, 6, 6, tmp);
  tmp[9] = 'V';
  tmp[10] = '\n';
  Serial.print(tmp);

  //    volt_old = round(volt * pow(10, volt_dec));
  //  }

  if (n < numReadings) return;

  //tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  //min max
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  if (vMn != vMn_old) {
    tft.drawFloat(vMn, 6,   3, 223, 2);
    vMn_old = vMn;
  }

  if (vMx != vMx_old) {
    tft.drawFloat(vMx, 6, 83 , 223, 2);
    vMx_old = vMx;
  }



  //White avereage
  if (average != average_old)
    tft.drawFloat(average, 6, 163, 223, 2); average_old = average;

  // Enob version 2 & its mean ************************************************
  //    tft.drawFloat(rs[0].Mean() , 6, 0, 70, 2);                          //

  if (enob(rs[0].StandardDeviation()) < 100)                                //
    tft.drawFloat(enob(rs[0].StandardDeviation()) , 4, 243, 223, 2);        //


  //***************************************************************************


  tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);

  int dt = (1000.0 / (millis() - timeKeeper)); timeKeeper = millis();
  if (dt < 100)
    tft.drawNumber(dt,  2 + 8 * 2 + 3, 64 + 8 * 2, 1);
  // FPS
  int fps = (1000.0 / dt);
  if (fps < 100)
    tft.drawNumber(fps, 2 + 8 * 2 + 3, 64 + 8, 1);

  //  tft.drawNumber((millis() - timeKeeper),  2 + 8 * 2 + 3, 64 + 8, 1);
  //    tft.drawString("dt", 2, 64 + 8*2, 1);


}

void drawFixedItems() {
  //TFT
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  long lineColor = ILI9341_BLUE;

  tft.drawLine(0, 0, 0, 239, lineColor);
  tft.drawLine(1, 239, 320, 239, lineColor);
  tft.drawLine(319, 239, 319, 1, lineColor);
  tft.drawLine(0, 0, 319, 0, lineColor);

  tft.drawLine(1, 210, 319, 210, lineColor);

  //lower grid
  for (int i = 1; i < 4; i++)
    tft.drawLine(80 * i, 210, 80 * i, 239, lineColor);


  lineColor = 0x000A;

  for (int i = 0; i < 7; i++)
    tft.drawLine(1, 60 + 20 * i, 319, 60 + 20 * i, lineColor);

  for (int i = 1; i < 16; i++)
    tft.drawLine(20 * i, 60, 20 * i, 180, lineColor);


  for (int i = 61; i < 200; i += 5)
    tft.drawLine(1,  i, 310,  i, ILI9341_BLACK);

  for (int i = 11; i < 311; i += 5)
    tft.drawLine(i,  66, i, 199, ILI9341_BLACK);


  tft.setTextSize(1);
  tft.fillRect(2, 60, 60, 30, ILI9341_BLUE);

  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLUE);
  tft.drawString("#", 2, 64, 1);
  tft.drawString("FPS", 2, 64 + 8, 1);
  tft.drawString("dt", 2, 64 + 8 * 2, 1);

  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.drawString("Vmin", 3, 215, 1);
  tft.drawString("Vmax", 83, 215, 1);

  tft.drawString("Avg (last   )", 162, 215, 1);
  tft.drawNumber( numReadings, 162 + 10 * 6, 215, 1);
  //numReadings

  tft.drawString("ENOB", 243, 215, 1);

  tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  tft.drawString("V", 290, 8, 2);

  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  tft.drawString("SIGMA", 160 - 12, 194, 1);
  for (int i = 1; i < 6; i += 1) {
    tft.drawNumber( i, 160 - 2  +  i * 20, 182, 1);
    tft.drawNumber(-i, 160 - 8 + -i * 20, 182, 1);

  }


}
/********************************************************************/
uint16_t histFind(double f) {
  uint16_t i = 0;
  while ((i < 256) && (f >= bounds[i])) {
    i++;
    //    tft.drawNumber(i,0, 120,  1);
  }
  return i;
}
/********************************************************************/
byte SPI_read()
{
  SPDR = 0;
  while (!(SPSR & (1 << SPIF))) ; /* Wait for SPI shift out done */
  return SPDR;
}
/********************************************************************/
double enob(double sd) {
  return double(log((v_ref * 20) / (sd * 3.4641016151377)) / log(2)) ;
}
/********************************************************************/
int  readAdc() {

  cbi(PORTB, LTC_CS1);            // LTC2400 CS Low
  delayMicroseconds(1);

  if (!(PINB & (1 << 4))) {    // ADC Converter ready ?
    //    cli();
    ltw = 0;
    sig = 0;

    b0 = SPI_read();             // read 4 bytes adc raw data with SPI
    if ((b0 & 0x20) == 0) sig = 1; // is input negative ?
    b0 &= 0x1F;                  // discard bit 25..31
    ltw |= b0;
    ltw <<= 8;
    b0 = SPI_read();
    ltw |= b0;
    ltw <<= 8;
    b0 = SPI_read();
    ltw |= b0;
    ltw <<= 8;
    b0 = SPI_read();
    ltw |= b0;
    delayMicroseconds(1);

    sbi(PORTB, LTC_CS1); // LTC2400 CS hi

    if (sig) ltw |= 0xf0000000;      // if input negative insert sign bit
    volt = (ltw) * v_ref / 4.0 / 3355443.0 -0*0.472- 1*(0.0292); // calibration c
   volt *= 1.0/((18.3478-2.0705)/(18.38-2.048))  ; //calibration m as in y=mx+c


    return 1;
  }
  sbi(PORTB, LTC_CS1); // LTC2400 CS hi
  return 0;
}

