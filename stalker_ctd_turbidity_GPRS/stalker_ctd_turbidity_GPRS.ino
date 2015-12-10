
//Data logger Demonstration using SeeeduinoStalker.
//Reads data from Decagon CTD using SDI-12
//and an OBS3+ turbidity sensor using the ADS1015 board.
//Data is transmitted via GPRSBee cell modem.
//Data also logged every 5 minutes to microSD card using an OpenLog module.

#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <DS3231.h>
//#include <SD.h>       //remove all SD functionality to save 10k flash
#include <Adafruit_ADS1015.h>
#include <SDI12.h>
#include <GPRSbee.h>
#include <Battery.h>

String targetURL; 
#define APN "internet"        //Change what's in quotation marks to the correct APN for the cell network

#define GPRSBEE_PWRPIN  9  //DTR
#define XBEECTS_PIN     4   //CTS

//int OpenLogPwr = 5;   //provides power to OpenLog board
#define DATAPIN 7         // change to the proper pin for sdi-12 data pin, pin 7 on shield 3.0
int sdi12enable = 6;    //pin 6 on shield 3.0 is sdi-12 excitation
SDI12 mySDI12(DATAPIN); 
int xbeeEnable = 5;

char CTDaddress = '1';      //for one sensor on channel '1' 
//char Oxygenaddress = '0';  

Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
Battery battery;

float batteryvoltage;
bool chargestate;
    
int testtimer = 1;    
int logginginterval = 120;    //number of seconds between logging measurements  (300 = 5 minutes)
long currentepochtime = 0;

float CTDtempC, CTDdepthmm, CTDcond, boardtemp;
//DOtempC, DOpercent, DOppm;

                  
//#define turbidityEnable = A1;   //digital pin connected to an optoisolator that triggers power to the OBS3 sensor                        

float lowturbidity, highturbidity;   //variables to hold the calculated NTU values





//The following code is taken from sleep.h as Arduino Software v22 (avrgcc) in w32 does not have the latest sleep.h file
#define sleep_bod_disable() \
{ \
  uint8_t tempreg; \
  __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
                       "ori %[tempreg], %[bods_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" "\n\t" \
                       "andi %[tempreg], %[not_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" \
                       : [tempreg] "=&d" (tempreg) \
                       : [mcucr] "I" _SFR_IO_ADDR(MCUCR), \
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE))); \
}


DS3231 RTC; //Create RTC object for DS3231 RTC come Temp Sensor 
static DateTime interruptTime;
//const int chipSelect = 10;


// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))



void setup () 
{
     /*Initialize INT0 pin for accepting interrupts */
     PORTD |= 0x04; 
     DDRD &=~ 0x04;

     Wire.begin();
     Serial.begin(19200);
     mySDI12.begin();
     RTC.begin();
     gprsbee.init(Serial, XBEECTS_PIN, GPRSBEE_PWRPIN);

     pinMode(A1, OUTPUT);        //configures OpenLog power trigger line as output
     digitalWrite(A1, HIGH);    //turns OpenLog on to begin with

     pinMode(A0, OUTPUT);        //configures extra opto pin as output
     digitalWrite(A0, LOW);    //turns off extra opto

     pinMode(A3, OUTPUT);        //configures  extra opto pin as output
     digitalWrite(A3, LOW);    //turns off extra opto
     
     pinMode(A2, OUTPUT);        //configures turbidity probe power trigger line as output
     digitalWrite(A2, LOW);    //turns turbidity probe off to begin with
     pinMode(sdi12enable, OUTPUT);   // make the excitation pin an output
     digitalWrite(sdi12enable, LOW);  //turn   off   the excitation pin
     //pinMode(xbeeEnable, OUTPUT);         //Xbee power enable pinmode declaration
     //digitalWrite(xbeeEnable, LOW);      //turn xbee on to start with 

//      gprsbee.off();
     
      ads.setGain(GAIN_ONE);          // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
      ads.begin();       //begin adafruit ADS1015
      
     attachInterrupt(0, INT0_ISR, LOW); //Only LOW level interrupt can wake up from PWR_DOWN
     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
     //Enable Interrupt 
     //RTC.enableInterrupts(EveryMinute); //interrupt at  EverySecond, EveryMinute, EveryHour
     // or this
     DateTime  start = RTC.now();
     interruptTime = DateTime(start.get() + logginginterval); //Add 5 mins in seconds to start time
     
     
   //  InitSDcard();   //go initialize the stalker SD card and write the header
     
     delay(3000);
     
     //writing header line to OpenLog
     Serial.println("CTD & turbidity logger");
     Serial.println("Date_Time_UTC,TZ-Offset,LoggerTime,CTD_Depth_mm,CTD_temp_DegC,CTD_cond_dS/m,Turb_low_NTU,Turb_high_NTU,BoardTempDegC,batteryvoltage,chargestate");
     
     delay(3000);
     digitalWrite(A1, LOW);    //turns OpenLog off
     
     
}

void loop () 
{
    ////////////////////// START : Application or data logging code//////////////////////////////////
    RTC.convertTemperature();          //convert current temperature into registers
    boardtemp = RTC.getTemperature(); //Read temperature sensor value
    battery.update();
    batteryvoltage = battery.getVoltage();
    chargestate = battery.isCharging();
    
    gprsbee.off();
    
    // freeRam();   
        
    DateTime now = RTC.now(); //get the current date-time    
    currentepochtime = (now.get());    //Unix time in seconds 
    

    String dateString = "";
    dateString += (now.year());
    dateString += "/";
    dateString += (now.month());
    dateString += "/";
    dateString += (now.date());
    dateString += " ";    
    dateString += (now.hour());
    dateString += ":";
    dateString += (now.minute());
    dateString += ":";
    dateString += (now.second());
    dateString += ",-0,";
    dateString += currentepochtime;
    
    
    
    
    //  now take some samples from the sensors  ///////////////////////
    
    //Serial.println("starting sampling");

    
    
    // --- CTD
    digitalWrite(sdi12enable, HIGH);   //wake up the sdi-12 CTD sensor 
    delay(1000);
    CTDMeasurement(CTDaddress);   //takes SDI-12 CTD measurement  
    delay(100);
    digitalWrite(sdi12enable, LOW);   //turn off the SDI12 sensor 
  

   
   //turn on OpenLog to let it settle while turbidity sensor is reading
   digitalWrite(A1, HIGH);
   
          //----  Turbidity 
    digitalWrite(A2, HIGH);       //trigger the opto that powers the OBS3+
    delay(2000);
    analogturbidity();   //take analog measurement for turbidity
    delay(100);
    digitalWrite(A2, LOW);   //turn off the turbidity probe


    delay(2000);

    sendtoSerial(dateString);   //sends all of the data to the serial port, gets captured by OpenLog module
    //delay(100);
    

    delay(3000);   //wait a second for OpenLog to finish 
    digitalWrite(A1, LOW);   //turns off OpenLog
    
    delay(100);
    
    assembleURL();
 
    delay(500);
    
    sendviaGPRS();

   // delay(3000);
    
    delay(1000);

        
    if (testtimer < 4){ 
       logginginterval = 120;    //Logger transmits every 2 minutes when first powered on, but only 4 times
       testtimer++;
    }
       else {
       logginginterval = 300;      // This is the number of seconds between measurements after the initial 10 minutes after powerup
    }
    

    delay(5000);
    
    gprsbee.off();

    delay(500);
    
    RTC.clearINTStatus(); //This function call is  a must to bring /INT pin HIGH after an interrupt.
    RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());    // set the interrupt at (h,m,s)
    attachInterrupt(0, INT0_ISR, LOW);  //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt
    
    
    ////////////////////////END : Application code //////////////////////////////// 
   
    
    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Down routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
            
    //Power Down routines
    cli(); 
    sleep_enable();      // Set sleep enable bit
    sleep_bod_disable(); // Disable brown out detection during sleep. Saves more power
    sei();
        
    //Serial.println("\nSleeping");
    delay(10); //This delay is required to allow print to complete
    //Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
    power_all_disable(); //This shuts down ADC, TWI, SPI, Timers and USART
    sleep_cpu();         // Sleep the CPU as per the mode set earlier(power down)  
    sleep_disable();     // Wakes up sleep and clears enable bit. Before this ISR would have executed
    power_all_enable();  //This shuts enables ADC, TWI, SPI, Timers and USART
    delay(10); //This delay is required to allow CPU to stabilize
    //Serial.println("Awake from sleep");    
    
    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Saver routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

} 

  
//Interrupt service routine for external interrupt on INT0 pin conntected to DS3231 /INT
void INT0_ISR()
{
  //Keep this as short as possible. Possibly avoid using function calls
    detachInterrupt(0); 
    interruptTime = DateTime(interruptTime.get() + logginginterval);  //decide the time for next interrupt, configure next interrupt  
}



void CTDMeasurement(char i){    //averages 6 readings in this one loop
  CTDdepthmm = 0.0;
  CTDtempC = 0.0;
  CTDcond = 0.0;
   
  for (int j = 0; j < 6; j++){
  
  String command = ""; 
  command += i;
  command += "M!"; // SDI-12 measurement command format  [address]['M'][!]
  mySDI12.sendCommand(command); 
  delay(500); // wait a while
  mySDI12.flush(); // we don't care about what it sends back

  command = "";
  command += i;
  command += "D0!"; // SDI-12 command to get data [address][D][dataOption][!]
  mySDI12.sendCommand(command);
  delay(500); 
     if(mySDI12.available() > 0){
        float junk = mySDI12.parseFloat();
        int x = mySDI12.parseInt();
        float y = mySDI12.parseFloat();
        float z = mySDI12.parseFloat();

      CTDdepthmm += x;
      CTDtempC += y;
      CTDcond += z;
     }
  
  mySDI12.flush(); 
     }     // end of averaging loop
     
      CTDdepthmm /= 6.0 ;
      CTDtempC /= 6.0;
      CTDcond /= 6.0;
     
}  //end of CTDMeasurement




void analogturbidity()     // function that takes reading from analog OBS3+ turbidity sensor
{ 
 int16_t adc0, adc1; //  adc2, adc3;      //tells which channels are to be read
 
 //digitalWrite(A1, HIGH);       //trigger the opto that powers the OBS3+
 //delay(2000);            //waits 2 seconds to allow readings to stabilize
 
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
 
 
//now convert bits into millivolts
 float lowvoltage = (adc0 * 2.0)/1000.0;   //with gain of ONE, adc0 is 2mv per bit
 float highvoltage = (adc1 * 2.0)/1000.0;
 



  //calibration information below if only for instrument SN# S9597
 lowturbidity =  (4.5285 * square (lowvoltage)) + (92.033 * lowvoltage) - 0.15659;
 highturbidity = (57.799 * square (highvoltage)) + (377.75 * highvoltage) - 1.023;



}


void assembleURL()
{
    targetURL = "";
    targetURL = "http://somewebsite.com/script.php?";
    targetURL += "LoggerID=xxxxx&Loggertime=";
    targetURL += currentepochtime;
    targetURL += "&CTDdepth=";
    addFloatToString(targetURL, CTDdepthmm, 3, 1);    //float   
    targetURL += "&CTDtemp=";
    addFloatToString(targetURL, CTDtempC, 3, 1);    //float   
    targetURL += "&CTDcond=";
    addFloatToString(targetURL, CTDcond, 3, 1);    //float  
    targetURL += "&TurbLow=";
    addFloatToString(targetURL, lowturbidity, 3, 1);    //float   
    targetURL += "&TurbHigh=";
    addFloatToString(targetURL, highturbidity, 3, 1);   //float
    targetURL += "&BoardTemp=";    
    addFloatToString(targetURL, boardtemp, 3, 1);     //float 
    targetURL += "&Battery=";    
    addFloatToString(targetURL, batteryvoltage, 4, 2);     //float 
    
    
//    Serial.print("TargetURL: ");
//    Serial.print(targetURL_1);
//    Serial.print(targetURL_2);
//    Serial.println();
//    delay(1000);
  
}

static void addFloatToString(String & str, float val, char width, unsigned char precision)
{
  char buffer[10];
  dtostrf(val, width, precision, buffer);
  str += buffer;
}

void sendviaXbee() {
   // Serial.print("sendviaXbee function:");

      Serial.println(targetURL); 
   
 //   Serial.print(targetURL_1.c_str()); 
    //  Serial.println(targetURL.c_str()); 
}


void sendviaGPRS()
{
  char buffer[10];
  if (gprsbee.doHTTPGET(APN, targetURL.c_str(), buffer, sizeof(buffer))) {
  }
}


void sendtoSerial(String & datestr)  {
   //Serial.println();
   //  Serial.print("sendtoSerial function: ");
      Serial.print(datestr);
      Serial.print(",");
      Serial.print(CTDdepthmm);
      Serial.print(",");
      Serial.print(CTDtempC);
      Serial.print(",");
      Serial.print(CTDcond);
      Serial.print(",");
      Serial.print(lowturbidity);
      Serial.print(",");
      Serial.print(highturbidity);
      Serial.print(",");
//      Serial.print(DOtempC);
//      Serial.print(",");
//      Serial.print(DOpercent);
//      Serial.print(",");
//      Serial.print(DOppm);      
//      Serial.print(",");
      Serial.print(boardtemp);
      Serial.print(",");
      Serial.print(batteryvoltage);
      Serial.print(",");
      Serial.print(chargestate);
      Serial.println();  
}
  
  
