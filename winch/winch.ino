


    // MAIN WINCH SKETCH BLOWN AWAY 14/9/19
    
    // COPY OF sketch_jul13a
    // this is mainsheet winch sketch
    // am going to try and alter the Serial2 section to receive actualBoomAngle Only




          //

          //                        THIS IS THE MAIN SAIL SHEETING SKETCH FOR BLOWN AWAY 28/2/19
          //                     *******************************************************************

          //                           OVERVIEW 
          //                       ********************
          //      THE FOLLOWING SECTIONS ARE MAIN SECTIONS AND MAY BE SUB DIVIDED , AND HAVE ADDITIONAL SECTIONS INCLUDED .
          //      ********************************************************************************************************
          
          // THE 1ST SECTION RECEIVES BOOM ANGLE AND BOOM SIDE FROM  BOOM ANGLE SKETCH ON SERIAL 2, PINS TX 16 , RX 17 .
          
          // THE 2ND SECTION AQUIRES WIND ANGLE FROM THE PROPRIETARY WIND INSTRUMENT AND CASTS IT TO AN int .
          
          // THE 3RD SECTION ASSIGNS A " REQUIRED BOOM ANGLE " USING " MAPPING " TO ( CURENTLY ) 35 CASES . 
          
          // THE 4TH SECTION AUTO WINCHING ASSESMENT SECTION INITIALLY SENDS REQUEST TO THE WINCH SECTION TO SET THE BOOM TO THE REQUIRED BOOM ANGLE , SUBSEQUENTLY IT TESTS TO SEE IF THERE...
          // ... HAS BEEN A " 15 DEGREE ( + OR - ) SHIFT IN WIND ANGLE " , IF TRUE IT SENDS A REQUEST TO THE WINCH SECTION TO SET BOOM TO REQUIRED ANGLE . THIS SECTION IS SUB DIVIDED INTO...
          // .. FOUR SUD SECTIONS , AN " IN " AND " OUT " SECTION FOR BOTH " PORT" AND " STARBOARD " .
          
          // THE 5TH SECTION IS A GYBE SECTION , WHICH ACTIVATES THE WINCHES IF THE WIND PASSES BEHIND THE BOAT BY ( CURRENTLY )" 15  DEGREES " INTO THE OPPOSITE SECTOR( IE.PORT OR STARBOARD ) .
          
          // THE 6TH SECTION IS A TACK SECTION , WHICH ACTIVATES THE WINCHES IF THE WIND PASSES INFRONT OF THE BOAT BY (CURRENTLY) "15 DEGREES " INTO THE OPPOSITE SECTOR . THIS IS SUB DIVIDED INTO 2 SECTIONS
          
          // THE 7TH SECTION IS THE WINCH SECTION  , THIS IS SUB DIVIDED INTO , 1st,THIS SUB SECTION CONTROLS THE WINCHES THAT HAVE BEEN INITIATED BY THE AUTO WINCHING ASSESMENT SECTION .
          // 2nd SUB SECTION CONTROLS THE WINCHES THAT HAVE BEEN INITIATED BY MANUAL INPUTS ( THIS SECTION IS ONLY ACTIVE IF THE AUTO SHEETING IS ON ) . 3rd SUB SECTION CONTROLS ....
          // ... THE WINCHES IF THERE HAS BEEN MANUAL INPUT , BUT THE AUTO SHEETING IS TURNED OFF .
          
          // THE FOLLOWING SECTIONS CONTROL LEDS WHICH INDICATE STATE OF OPERATION IF WINCHES OPERATED MANUALY . FINALLY SERIAL PRINT SECTION FOR TESTING .
          

   // AUTO SHEETING AND REQUIRED BOOM ANGLE SKETCH .
  // **********************************************


// 14/2/17
/*
    copy of sketch_jan25a_auto_sail
need to see if i can get LED feeds out to indicate
boom angle 
REQUIRED BOOM ANGLES (on 29/6/15) ARE  "  5 ,15 ,30 ,50 ,   "
*******************                      ***************
Currently (14/2/17) Wind Angles of between 0  and  41  degrees  Require a Boom Angle of  5 degrees
                    Wind Angles of between 319 and 359 degrees  Require a Boom Angle of  5 degrees

                    Wind Angles of between 42 and  71  degrees  Require a Boom Angle of  15 degrees
                    Wind Angles of between 289 and 318 degrees  Require a Boom Angle of  15 degrees
                    
                    Wind Angles of between 72 and  123 degrees  Require a Boom Angle of  30 degrees
                    Wind Angles of between 237 and  288 degrees Require a Boom Angle of  30 degrees
                    
                    Wind Angles of between 124 and  236 degrees Require a Boom Angle of  50 degrees

    Note Pins  30 to 53 are available for leds 15/2/17.               
                    


*/




   // sketch_apr22a
   //    Working_bridge_RANGE_sketch_feb06a
   // READY TO PUT IF NEGATIVE ON results 2_3  AND else if ;
   
  //  Copied From  " yes_custom_TinyGPS_for_airmar_200wxsketch_jan29a  "  ( SEE custom TinyGPS by Tom Igoe )
  //  SEE ALSO   "  differential_ADS1115_TEST_working sketch "
  /* TRYING  TO INCORPORATE THE TMP36  INTO THIS  WORKING BRIDGE SKETCH */
   //  SEE ALSO  Test_ADS1115
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

 Adafruit_ADS1115 ads1115(0x48);  /* Use this for the 16-bit version */
 Adafruit_ADS1115 adcTP36(0x49);  //    IMPORTANT :  NOTE THE CHANGE OF NAME  "  adcTP36  "  THIS ENABLES IDENTIFICATION OF THIS PARTICULAR ADS1115.
//Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */


  // MY TESTING SKETCH   NOT USING SS BUT  SERIAL1

/*
   This sample demonstrates TinyGPS++'s capacity for extracting custom
   fields from any NMEA sentence.  TinyGPS++ has built-in facilities for
   extracting latitude, longitude, altitude, etc., from the $GPGLL and 
   $GPRMC sentences.  But with the TinyGPSCustom type, you can extract
   other NMEA fields, even from non-standard NMEA sentences.

   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(RX) and 3(TX).
*/
//static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;

// The TinyGPS++ object
TinyGPSPlus gps;

const int travellerDriveRight = 48 ; // pin
const int travellerDriveLeft  = 49 ; // pin

const int travellerStopRight  = 50 ; // pin INPUT FROM HALL SWITCH MP101401  LOW == MADE
const int travellerStopCenter = 51 ; // pin INPUT FROM HALL SWITCH MP101401  LOW == MADE
const int travellerStopLeft   = 52 ; // pin INPUT FROM HALL SWITCH MP101401  LOW == MADE
const int mainSheet_IN_Stop   = 28;  // pin INPUT
const int mainSheet_OUT_Stop  = 29;  // pin INPUT

const int auto_winch_isolate  = 27; //( changed from 27 as this pin already allocated ); // pin  INPUT  ISOLATION INPUT SWITCH VARIABLE // IF THIS GOES " LOW " IT TURNS OFF ALL AUTO SHEETING , SUPPLIED FROM A MANUAL ISOLATOR SWITCH .
const int manualWinch_IN_input = 8;// pin 8 ( changed from 26 as this pin already allocated );26; // pin  INPUT VIA TOGGLE INTERFACE SKETCH FROM SIP /PUFF SKETCH                                        *****************************************
const int manualWinch_OUT_input = 7;// pin 7 ( changed from 25 as this pin already allocated );25;//  pin  INPUT VIA TOGGLE INTERFACE SKETCH FROM SIP /PUFF SKETCH

long manualWinch_input_debounceTime ; // debounce the manualWinch_inputs
int  manualWinch_debounce_toggle = 0 ; // 0 IS UNLOCKED 
long manualWinch_IN_Timer ;  // MANUAL : TIME THE WINCH IN
int  manualWinch_IN_Lock = 0 ; // 0 = UNLOCKED
long manualWinch_OUT_Timer ;  //  MANUAL : TIME THE WINCH OUT
int  manualWinch_OUT_Lock = 0; // 0 = UNLOCKED  

int  autoWinchOutStarted = 1 ; // LOCKED , NO " AUTO WINCHING OUT " STARTED  ( IF 0 " AUTO WINCHING OUT " HAS INITIATED ).
int  autoWinchInStarted  = 1; //  LOCKED , NO " AUTO WINCHING IN  " STARTED  ( IF 0 " AUTO WINCHING IN  " HAS INITIATED ).
int gybeInProgressLock = 0;  // MAY NEED TO USE THIS TO HALT   SOME  OF THE PROCESSES In  sketch_sep04b  WHILE GYBE IS IN PROGRESS ???????????????????????
                             //                                  ******
int tackInProgressLock = 0; // 0 zero = unlocked
int windAft_or_forward = 0;      // 1 = aft , 2 = forward determine where wind is in relation to boat when move through wind INITIATED
int winch_IN_Signal = 0; // 0 = IN Signal OFF ,  1 = IN Signal ON .
int winch_OUT_Signal = 0; // 0 = OUT Signal OFF , 1 = OUT Signal ON .

const int  ledBoomAngPort5 =  30;  // start of Actual boom angle leds . don't forget pinmodes 15/2/17 // wire black white stripe
const int  ledBoomAngPort15 = 46;  // changed from pin 31    22/8/17  // wire green no stripe
const int  ledBoomAngPort30 = 32;  // wire white no stripe
const int  ledBoomAngPort38 = 33;  // wire blue red stripe // altered from 50 to 38 for blown away on 23/8/19
const int  ledBoomAngStar5 =  34;  // wire green black stripe
const int  ledBoomAngStar15 = 35;  // wire orange red stripe
const int  ledBoomAngStar30 = 36;  // wire green white stripe
const int  ledBoomAngStar38 = 37;  // wire white red stripe

const int  ledBoomAng_Req_Port5  = 38;// Start of Required Boom angle leds 15/2/17  // wire  red no stripe
const int  ledBoomAng_Req_Port15 = 39; //  wire red black stripe
const int  ledBoomAng_Req_Port30 = 40; //  wire white  black & red stripe
const int  ledBoomAng_Req_Port38 = 41; //  wire orange green stripe
const int  ledBoomAng_Req_Star5  = 42; //  wire blue white stripe
const int  ledBoomAng_Req_Star15 = 43; //  wire orange black stripe
const int  ledBoomAng_Req_Star30 = 44; //  wire red green stripe
const int  ledBoomAng_Req_Star38 = 45; //  wire orange no stripe

int        ledBoomAng_Toggle = 0;  //  ??????   22/3/17 ???
long unsigned ledBoomAng_Timer =0; //  ??????   22/3/17 ???

long unsigned count = millis();
int actualBoomAngle = -20; // THIS AND requiredBoomAngle  ARE THE VALUES THAT DETERMINE THE OUTPUT .
int boomSide = -1 ; 
int requiredBoomAngle = 0; // THIS AND actualBoomAngle  ARE THE VALUES THAT DETERMINE THE OUTPUT .
const int port = 1 ; // boom side port  allocate as appropriate
const int starboard = 2 ; // boom side starboard  allocate as appropriate


int failWinchDrive = 0;//
String headingString ;
int headingInt = -1000;

long unsigned ledTimer;
long unsigned ledCount = 0;
long unsigned inLEDtimer;
long unsigned inLEDcount = 0;
long unsigned outLEDtimer;
long unsigned outLEDcount = 0;

long unsigned winchDelayTimer; // 16/10/15
long unsigned winchDelayCount; // 16/10/15

int winchDelayResetOne = 0 ; // 12/2/19 AUTO SHEETING WINCH IN STARBOARD TACK
int winchDelayResetTwo = 0 ; // 12/2/19 AUTO SHEETING WINCH OUT STARBOARD TACK
int winchDelayResetThree = 0; // 12/2/19AUTO SHEETING WINCH IN PORT TACK
int winchDelayResetFour = 0 ; // 12/2/19AUTO SHEETING WINCH OUT PORT TACK
const int winch_IN_Mainsheet_Pin = 10; //winch_IN_pin = 10;
const int winch_OUT_Mainsheet_pin = 11; //

const int ledOrangeIn = 22;
const int ledBlueOut =  23;
const int ledGreenOK =  24;
const int ledWhiteManualIn = 25;
const int ledWhiteManualOut = 26;

int       ledGreenSTATE = LOW;
int toggleActivateWinchIN = 0;
int toggleActivateWinchOUT = 0;
int manualWinchINtoggle = 0;
int manualWinchOUTtoggle = 0;


int winchInToggle = 0;
int winchOutToggle = 0;
int currentWindA_Toggle = 0;
int currentWindA = 0 ; // testing seems 
int inLEDtoggle = 0;
int outLEDtoggle = 0;

long unsigned ledBoomOperatedMANUALLY_Timer = 0;
long unsigned ledManuallyOperated_count = ledBoomOperatedMANUALLY_Timer;



long unsigned requiredBoomAngle_delayTimer =0;
int           requiredBoomAngle_delayReset =0;

int startTransmissionRequest = 0;
int countToggle = 0; 
long transmissionCount ;
long transmissionCount1 ;

//TinyGPSCustom MET(gps , "PAMTS" , 8 ); //   MY STUFF ?????? THIS WORKED ON old AIRMAR 150wx
TinyGPSCustom windA(gps , "WIMWV" ,1); //   THIS GIVES WIND ANGLE  IT WORKS
TinyGPSCustom RorT(gps , "WIMWV" ,2); //  this gives 2nd value R(apparent) or T(true)
TinyGPSCustom heading(gps , "HCHDG" ,1); //  THIS IS SURPOSED TO GIVE HEADING RELATIVE TO TRUE NORTH . NOTE: ( WILL ONLY BE AVAILABLE IF HEADING AND VARIATION ARE  AVAILABLE .  ) 

// these constants won't change. They are the
// lowest and highest readings you get from your sensor:

       int airMarMin = 0;      // sensor minimum, discovered through experiment    changed fom 0   works for my sensor
       int airMarMax = 360;    // sensor maximum, discovered through experiment      changed from 600 works for my sensor
       int windAngleInt = -10;
       float windAngleFloat ;

void setup() 
{

  

  pinMode ( travellerStopRight , INPUT  ); // pin 50  , don't forget to 10K ground the pin NEED TO GET INPUT FROM THESE SWITCHES FROM HALL SWITCH MP101401  LOW == MADE
  pinMode ( travellerStopCenter , INPUT ); // pin 51    don't forget to 10K ground the pin NEED TO GET INPUT FROM THESE SWITCHES FROM HALL SWITCH MP101401  LOW == MADE
  pinMode ( travellerStopLeft  , INPUT );  // pin 52    don't forget to 10K ground the pin NEED TO GET INPUT FROM THESE SWITCHES FROM HALL SWITCH MP101401  LOW == MADE
  
  pinMode ( mainSheet_IN_Stop , INPUT ); // pin 28      don't forget to 10K ground the pin NEED TO GET INPUT FROM THESE SWITCHES
  pinMode ( mainSheet_OUT_Stop , INPUT ); //pin 29      don't forget to 10K ground the pin NEED TO GET INPUT FROM THESE SWITCHES
  
  pinMode ( auto_winch_isolate , INPUT ); // pin 27     don't forget to 10K ground the pin NEED TO GET INPUT FROM THESE SWITCHES FROM MANUAL ISOLATION SWITCH
  
  pinMode ( manualWinch_IN_input , INPUT ); // pin 8   don't forget to 10K ground the pin NEED TO GET INPUT FROM THESE SWITCHES INPUT VIA TOGGLE INTERFACE SKETCH FROM SIP /PUFF SKETCH 
  pinMode ( manualWinch_OUT_input , INPUT ); // pin 7   don't forget to 10K ground the pin NEED TO GET INPUT FROM THESE SWITCHES INPUT VIA TOGGLE INTERFACE SKETCH FROM SIP /PUFF SKETCH 
  
 
  pinMode (travellerDriveRight , OUTPUT ); // pin 48; grey
  pinMode (travellerDriveLeft  , OUTPUT ); // pin 49 ;green
  pinMode(winch_IN_Mainsheet_Pin , OUTPUT); // pin 10 ; white
  pinMode(winch_OUT_Mainsheet_pin , OUTPUT);// pin 11 ; yellow
  
  pinMode(ledBoomAngPort5 ,OUTPUT); //Pin 30 , Start OF ACTUAL led boom angle pinModes 15/2/17
  pinMode(ledBoomAngPort15 ,OUTPUT);//Pin 31,
  pinMode(ledBoomAngPort30 ,OUTPUT);//Pin 32,
  pinMode(ledBoomAngPort38 ,OUTPUT);//Pin 33,
  pinMode(ledBoomAngStar5 ,OUTPUT); //Pin 34,
  pinMode(ledBoomAngStar15 ,OUTPUT);//Pin 35,
  pinMode(ledBoomAngStar30 ,OUTPUT);//Pin 36,
  pinMode(ledBoomAngStar38 ,OUTPUT);//Pin 37,
  pinMode(ledBoomAng_Req_Port5 ,OUTPUT);// Pin38,Start OF REQUIRED led boom angle pinModes 15/2/17
  pinMode(ledBoomAng_Req_Port15 ,OUTPUT);//Pin39,
  pinMode(ledBoomAng_Req_Port30 ,OUTPUT);//Pin40,
  pinMode(ledBoomAng_Req_Port38 ,OUTPUT);//Pin41,
  pinMode(ledBoomAng_Req_Star5 ,OUTPUT); //Pin42,
  pinMode(ledBoomAng_Req_Star15 ,OUTPUT);//Pin43,
  pinMode(ledBoomAng_Req_Star30 ,OUTPUT);//Pin44,
  pinMode(ledBoomAng_Req_Star38 ,OUTPUT);//Pin45  ,End of boomangle LEDS . 15/2/17
    
  pinMode(ledGreenOK , OUTPUT);  // pin 24
  pinMode(ledOrangeIn , OUTPUT); // pin 22
  pinMode(ledBlueOut , OUTPUT);  // pin 23
  pinMode(ledWhiteManualIn , OUTPUT); // pin 25
  pinMode(ledWhiteManualOut , OUTPUT); // pin 26
  
  digitalWrite(winch_IN_Mainsheet_Pin , LOW );
  digitalWrite(winch_OUT_Mainsheet_pin , LOW );
  digitalWrite(travellerDriveRight , LOW );
  digitalWrite(travellerDriveLeft , LOW );
 
 
  

  

  delay( 1000 ) ; // THIS HAS BEEN PLACED HERE BECAUSE THE SERIAL CONNECTION WAS NOT BEING MADE ON START UP WITH OUT IT , DON'T KNOW WHY , BUT THIS SEEMS TO WORK !!!!
    


  
  
  Serial.begin(4800);  //  Note the Baud RATE FOR SERIAL MONITOR IS  4800
                       //                         **************
  Serial1.begin(4800); //  NOTE THIS IS   Serial 1 , TX 18 , RX 19 , FROM WIND INSTRUMENT .
                      //                        **
  Serial2.begin(4800); // NOTE THIS IS Serial 2 16 TX 17 RX , FROM BOOM ANGLE SKETCH .
  Serial3.begin(4800); // NOTE: This is Serial 3 14 TX 15 RX , TRANSMITTING actualBoomAngle , AND windAngleInt TO THE DEDICATED GRAPHICS ARDUINO
  
  
}

void loop() 
{
  

           /* BOOM ANGLE RECEIVING  SECTION FROM BOOM ANGLE SKETCH ON SERIAL 2, PINS TX 16 , RX 17
             ****************************** */
   Serial.print("startTransmissionRequest = " );
   Serial.println(startTransmissionRequest );

   if(startTransmissionRequest == 0)
    {
     Serial2.write('x');    // changed from Serial1.print("x");
     startTransmissionRequest = 1;
     transmissionCount = millis();
    }

   if (Serial2.available()) 
    {        // NOTE THIS IS Serial 2 16 TX , 17 RX .
      Serial.print (" HERE " );
     char  inChar = Serial2.read();
      if(inChar == 'a') 
       {
         Serial.print (" HERE AGAIN " );
         actualBoomAngle = Serial2.parseInt();
         startTransmissionRequest = 0;
         Serial.print ( " Time Taken for Serial 2 Section = " );
         Serial.println( millis() - transmissionCount );
      //  boomSide = Serial2.parseInt();  // STOPPED USING BOOM SENSOR TO DETERMINE BOOM SIDE AND STARTED USING WIND ANGLE  16/8/19
     //   Serial2.write('y');               //******************************************************************************************
      
       }
         if( actualBoomAngle != -1000) // TEST PURPOSES ONLY
           {
            Serial.print (" BUGGER actualboomAngle is ");
            Serial.println(actualBoomAngle);
           }
     // if(inChar == 'b')
     //  {
     //   Serial.print (" HERE STILL " );
     //   actualBoomAngle = Serial2.parseInt();
     //   startTransmissionRequest = 0;
     //   Serial.print ( " Time Taken for Serial 2 Section = " );
     //   Serial.println( millis() - transmissionCount );
     //  }

    }
   
    

     if( millis() >= transmissionCount1 + 2000 )
       {
         countToggle = 0;
       }
  

     if(countToggle == 0 )
      {
       Serial.print ("boomSide is ");
       Serial.println (boomSide);
       Serial.print (" actualBoomAngle is ");
       Serial.println(actualBoomAngle);
    // Serial.print("intTemp is ");
    // Serial.println(intTemp);
       countToggle = 1;
       transmissionCount1 = millis();
      }
  
    // *********************************************************************************  
   

      // BOOM ANGLE AND WIND ANGLE TRANSMISSION TO GRAPHICS ARDUINO VIA SERIAL 3 (14 TX 15 RX) Section
      // ***********************************************************************              ******
      if ( Serial3.available() ) // TX Pin 14  RX Pin 15
         {
          char inChar3  = Serial3.read();
          
          if(inChar3 == 'v')
            {
              Serial3.write ( 'c' );
              Serial3.println(windAngleInt);
            }

          if(inChar3 == 'w')
            {
              Serial3.write ( 'd' ) ;
              Serial3.println( actualBoomAngle );
              
            }
         }

      
      /*    REQUIRED BOOM ANGLE SECTION .   AIRMAR INPUT .
          ***************************************  */
          
        
                          /* AQUIRE WIND ANGLE 
                            *******************  */
            
         while (Serial1.available() > 0 )              // 
   {
    gps.encode (Serial1.read());              //(Serial1.read()); PINS 18 , 19 .          ****************************       MOVED THIS SECTION UP FROM BELOW  , SEEMS TO BE A BETTER INITIAL POSITION  ?????
  // Every time anything is updated, print everything.
  if (
    windA.isUpdated()||
    RorT.isUpdated() ||
    heading .isUpdated() 
    )
  {   // IMPORTANT NOTE: the Close of this Bracket Terminates at the " End of Switch Case Mapping to map wind angle to required boom angle " .
   
   String windAngleString  = windA.value();  //  windAngleString ONLY APPEARS HERE   LOCAL  CONVERTS  char*  TO  String
   char carray [windAngleString.length() +1];   // IMPORTANT:  THIS IS THE CONVERSION  SECTION  CREATES AN ARRAY OF CORRECT   SIZE   VERY IMPORTANT !!!!  THIS  WORKS ....http://forum.arduino.cc/index.php?topic=45357.0  SEE  ZOOMKAT !!!!
   windAngleString.toCharArray(carray, sizeof(carray)); //      CONVERTS   String   TO   array  .
   windAngleFloat = atof(carray);                       //      CONVERTS   array    TO   float
      // windAngleInt = atoi(carray);  //  NOT SURE ABOUT THIS  AS THE   windA  IS A DECIMAL  DON'T KNOW WHAT'S GOING TO HAPPEN WITH THE DECIMAL  THINK I MAY LEAVE AS FLOAT ABOVE AND CONVERT TO INT  FURTHER DOWN !
   windAngleInt = (int) windAngleFloat; // TRYING TO CONVERT THE FLOAT  TO AN  INT . SEEMS TO WORK  IT WILL  REDUCE THE DECIMAL TO INT  EG  41.9  =   41  OK FOR OUR PURPOSE
   
   // ***************************************************************************
   
   headingString =  heading . value();
   //char carray [headingString.length() +1];   // IMPORTANT:  THIS IS THE CONVERSION  SECTION  CREATES AN ARRAY OF CORRECT   SIZE   VERY IMPORTANT !!!!  THIS  WORKS ....http://forum.arduino.cc/index.php?topic=45357.0  SEE  ZOOMKAT !!!!
   headingString.toCharArray(carray, sizeof(carray)); //      CONVERTS   String   TO   array 
   headingInt = atoi(carray);//

  // *****************************************************************************    */  

    // Section using windAngleInt to Determine Boom Side 16/8/19
    //**********************************************************
      if (windAngleInt > 0 && windAngleInt < 180 )
         {
          boomSide = 1; // PORT :   started using windAngleInt to Determine Boom Side 16/8/19
         }
       if(windAngleInt > 180 && windAngleInt < 360 )
         {
          boomSide = 2; // STARBOARD
         }
     // ***********************************
     
   /*  " REQUIRED BOOM ANGLE " SECTION
        ********************   *******   
         */
 
 /*   if (requiredBoomAngle_delayReset == 1)  // NOTE : THIS DELAY IS INTENDED TO ALLOW TIME FOR THE " WINCH " TO ACHIEVE THE REQUIRED BOOM ANGLE , BEFORE..
    {                                       //  ..     ALLOWING A SUBSEQUENT REQUIRED BOOM ANGLE TO BE CALCULTED !( hopefully avoiding the winch trying to chase a 2nd required boom angle before attaining the original angle )
      if( millis()>= requiredBoomAngle_delayTimer + 2000)  // ALTER DELAY TIME HERE AS REQUIRED
      {
        requiredBoomAngle_delayReset = 0;
      }
    } */ // CUT 12/2/19
    

    if( requiredBoomAngle_delayReset == 0 ) // IF WINCH SECTIONS BELOW ARE SATISFIED
             {
              // MAPPING WIND ANGLE TO REQUIRED BOOM ANGLE
            //  ********************************************    
               
         
   // map the sensor range to a range of four options:
   int wind_range = map  (windAngleInt, airMarMin, airMarMax, 0, 35); //  THIS IS THE CLEVER BIT USING MAP TO SWITCH CASE WITH  " wind_range  "

  // do something different depending on the 
  // range value:
  switch (wind_range) {
  case 0:    // 
   // Serial.println("should be between 0 and 10"); // this seems to be accurate , despite there being 11 units between 0 and 10
   // Serial.print (" ...." );
   // Serial.println(windAngleInt);
    requiredBoomAngle = 5; // altered to 10 from 5 as sytem could not achieve a target value of 5 . ( 4/9/17 )
    if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star5 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port5 , HIGH );
      
    }
   // Serial.println ("....requiredBoomAngle is .. ");
   // Serial.print (requiredBoomAngle);
   // Serial.print(".......");
    break;
  case 1:    // 
    //Serial.println("should be between 11 and 20");  //  this seems to be correct , despite there only being  10 units between 11 and 20
   // Serial.println ("...wind Angle...");
   // Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 5; // altered to 10 ( 4/9/17 )
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star5 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port5 , HIGH );
      
    }
   // Serial.println ("....requiredBoomAngle is .. ");
   // Serial.print (requiredBoomAngle);
   // Serial.print(".......");
    break;
  case 2:    // 21 to 30 deg
    //Serial.println("should be between 21 and 30");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
   // Serial.println ( "......");
    requiredBoomAngle = 5; // altered to 10 ( 4/9/17 )
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star5 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port5 , HIGH );
     
    }
   // Serial.println ("....requiredBoomAngle is .. ");
   // Serial.print (requiredBoomAngle);
   // Serial.print(".......");
    break;
  case 3:    // 31 to 40 deg
   // Serial.println("should be between 31 and 41");
   // Serial.println ("...wind Angle...");
   // Serial.print ( windAngleInt);
   // Serial.println ( "......");
    requiredBoomAngle = 5; // altered to 10 ( 4/9/17 )
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star5 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port5 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 4:
    //Serial.println("should be between 42 and 51");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 5;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star15 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port15 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;     //                                                                                                
  case 5:    // 
    //Serial.println("should be between 52 and 61");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 5;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star15 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port15 , HIGH );
     
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 6:    // 
    //Serial.println("should be between 62 and 71");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 15;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star15 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port15 , HIGH );
     
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 7:    // 
    //Serial.println("should be between 72 and 82");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 15;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
     
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 8:    // 
    //Serial.println("should be between 83 and 92");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 30;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
     
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 9:    // 
    //Serial.println("should be between 93 and 102");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 30;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 10:    // 
    //Serial.println("should be between 103 and 113");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 30;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 11:    // 
    //Serial.println("should be between 114 and 123");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 30;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 12:    // 
    //Serial.println("should be between 124 and 133");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 from 50 ( 4/9/17 ) // altered from 45 to 38 for blown away
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
     
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 13:    // 
    //Serial.println("should be between 134 and 143");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 14:    // 
    //Serial.println("should be between 144 and 152"); //  bit worrying  dropped back again   hope it's not randomly placing these figures !!!
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 15:    // 
    //Serial.println("should be between 153 and 164");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 16:    // 
    //Serial.println("should be between 165 and 174");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
     
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 17:    // 
    //Serial.println("should be between 175 and 185"); //  WIND  BEHIND    BOAT ON RUN  OR EVEN SAILING BY LEE
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 18:    // 
    //Serial.println("should be between 186 and 195");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
     
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 19:    // 
   // Serial.println("should be between 196 and 205");
   // Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 20:    // 
    //Serial.println("should be between 206 and 215");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 21:    // 
    //Serial.println("should be between 216 and 226");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 22:    // 
    //Serial.println("should be between 227 and 236");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 38; // altered to 45 ( 4/9/17 ) // altered to 38 on 23/8/19
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star38 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port38 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 23:    // 
    //Serial.println("should be between 237 and 246");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 30;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 24:    // 
    //Serial.println("should be between 247 and 257");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 30;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 25:    // 
   // Serial.println("should be between 258 and 268");
   // Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
   // Serial.println ( "......");
    requiredBoomAngle = 30;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
     
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 26:    // 
    //Serial.println("should be between 269 and 278");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 30;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 27:    // 
    //Serial.println("should be between 279 and 288");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 15;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star30 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port30 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
   // Serial.print (requiredBoomAngle);
   // Serial.print(".......");
    break;
  case 28:    // 
    //Serial.println("should be between 289 and 298");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 15;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star15 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port15 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 29:    // 
    //Serial.println("should be between 299 and 308");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 5;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star15 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port15 , HIGH );
      
    }
   // Serial.println ("....requiredBoomAngle is .. ");
   // Serial.print (requiredBoomAngle);
   // Serial.print(".......");
    break;
  case 30:    // 
    //Serial.println("should be between 309 and 318");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 5;
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star15 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port15 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 31:    // 
    //Serial.println("should be between 319 and 329");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 5; // altered to 10 ( 4/9/17 )
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star5 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port5 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 32:    // 
   // Serial.println("should be between 330 and 339 ");
   // Serial.println ("...wind Angle...");
   // Serial.print ( windAngleInt);
   // Serial.println ( "......");
    requiredBoomAngle = 5; // altered to 10 ( 4/9/17 )
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star5 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port5 , HIGH );
     
    }
   // Serial.println ("....requiredBoomAngle is .. ");
   // Serial.print (requiredBoomAngle);
   // Serial.print(".......");
    break;
  case 33:    // 
    //Serial.println("should be between 340 and 349");
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 5; // altered to 10 ( 4/9/17 )
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star5 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port5 , HIGH );
     
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    break;
  case 34:    // 
    //Serial.println("should be between 350 and 359"); // cant get 360 from airmar OF COURSE !
    //Serial.println ("...wind Angle...");
    //Serial.print ( windAngleInt);
    //Serial.println ( "......");
    requiredBoomAngle = 5;// altered to 10 ( 4/9/17 )
     if( boomSide == starboard)
    {
    digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
    digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
    digitalWrite( ledBoomAng_Req_Star5 , HIGH );
    
    }
    else if(boomSide == port)
    {
     digitalWrite( ledBoomAng_Req_Port5,LOW);digitalWrite( ledBoomAng_Req_Port15,LOW);digitalWrite( ledBoomAng_Req_Port30,LOW);digitalWrite( ledBoomAng_Req_Port38,LOW);
     digitalWrite( ledBoomAng_Req_Star5 ,LOW); digitalWrite( ledBoomAng_Req_Star15 ,LOW); digitalWrite( ledBoomAng_Req_Star30 ,LOW); digitalWrite( ledBoomAng_Req_Star38 ,LOW);
     digitalWrite( ledBoomAng_Req_Port5 , HIGH );
      
    }
    //Serial.println ("....requiredBoomAngle is .. ");
    //Serial.print (requiredBoomAngle);
    //Serial.print(".......");
    
    break;
    
 // case 35:    // 
 // Serial.println(" it appears that this one doesn't exist , it may be a break or something ");
 // break; 
    
  } 

             } // 22/8/17     closes " if( requiredBoomAngle_delayReset ==0 )
  
    
         } 
     } 
     
   // **********************************************END OF REQUIRED BOOM ANGLE SECTION   

   
     
     
     /*    OUTPUT  SECTIONS  FOLLOW
           *************************    */
     
      
        // ******************************************WINCH SECTIONS FROM ORIGINAL MISS ISLE TOO  sketch_sep04b  ALTERED RADICALLY 12/2/19
        //                                        **************************************************************

         
          /* TOGGLE ACTIVATE SECTION
             ***********************    */ 
           // THIS SECTION CAN ONLY RUN ONCE AT THE BEGINNING DUE TO " currentWindA_Toggle "
           //             ********************               ***************************** 
           //THESE SECTIONS ARE IN PLACE TO AUTOMATICALLY GET A GREEN LIGHT ON START UP
           //REMOVED 25/9/19
       /*    if(currentWindA_Toggle == 0)  // THIS TOGGLE MAY NOW BE REDUNDANT ? 26/8/15 . reinstated 29/8/15 to run only once at start to establish currentWindA .
             {                                                                          //**************************************************************************
               currentWindA =  windAngleInt; // THIS INITIATES currentWindA TO A VALUE , WHICH CAN BE COMPARED IN THE FOLLOWING SECTIONS .
               
              if (requiredBoomAngle > 0 )  //TRYING TO ENSURE THERE IS A CALCULATED requiredBoomAngle NOT THE 0 ASCRIBED TO THE int AT THE START .
                 { 
            //***************************************************************
                 if( actualBoomAngle > requiredBoomAngle) // THESE SECTIONS ARE IN PLACE TO AUTOMATICALLY GET A GREEN LIGHT ON START UP
                   {
                    digitalWrite(ledGreenOK , LOW );
                    digitalWrite(ledBlueOut , LOW );
                    digitalWrite(ledWhiteManualIn , LOW);
                    digitalWrite(ledWhiteManualOut , LOW);
                    digitalWrite(ledOrangeIn , HIGH );
                    winch_IN_Signal = 1; // change from winch_IN_Pin 6/2/19 // digitalWrite (winch_IN_pin,HIGH); WINCH " IN " SIGNAL " ON "
                    winchInToggle = 1;
                    manualWinchINtoggle = 0;
                    manualWinchOUTtoggle = 0;
                   }
                 if(winchInToggle == 1) // ACTIVATED FROM ABOVE
                   {
                   if(actualBoomAngle<= requiredBoomAngle)
                     {
                      winch_IN_Signal = 0; // change from winch_IN_Pin 6/2/19//digitalWrite(winch_IN_pin , LOW );//  WINCH " IN " SIGNAL " OFF "
                      digitalWrite(ledOrangeIn ,  LOW );
                      winchInToggle = 0;
                      toggleActivateWinchIN = 0;
                      currentWindA_Toggle = 1;  // THIS PREVENTS THIS SECTION FROM RUNNING AGAIN , AND ALLOWS THE WINCH SECTIONS THAT FOLLOW TO RUN
                     }
                   }
                 
             //****************************************************    
                if( actualBoomAngle < requiredBoomAngle ) // 
                  {
                   digitalWrite(ledGreenOK , LOW );
                   digitalWrite(ledOrangeIn , LOW );
                   digitalWrite(ledWhiteManualIn , LOW);
                   digitalWrite(ledWhiteManualOut , LOW);
                   digitalWrite(ledBlueOut , HIGH);
                   winch_OUT_Signal = 1; // 0 = OUT Signal OFF , 1 = OUT Signal ON .//digitalWrite(winch_OUT_pin,HIGH); WINCH " OUT " SIGNAL " ON "
                   winchOutToggle = 1;
                   manualWinchINtoggle = 0;
                   manualWinchOUTtoggle = 0;
                  }   
                  if(winchOutToggle == 1) // ACTIVATED FROM ABOVE
                    {
                    if(actualBoomAngle>= requiredBoomAngle)
                      {
                       winch_OUT_Signal = 0; // 0 = OUT Signal OFF , 1 = OUT Signal ON .// digitalWrite(winch_OUT_pin,LOW); WINCH " OUT " SIGNAL  " OFF "
                       digitalWrite(ledBlueOut , LOW);
                       winchOutToggle = 0;
                       toggleActivateWinchOUT = 0;
                       currentWindA_Toggle = 1;  // // THIS PREVENTS THIS SECTION FROM RUNNING AGAIN , AND ALLOWS THE WINCH SECTIONS THAT FOLLOW TO RUN
                      }
                    }
              //***************************************************
               
             }   
            }  // END OF currentWindA_Toggle == 0
         // ************************************************END OF START UP GREEN LIGHT SECTION 

         */

             //   TOGGLE ACTIVATION HAPPENS  HERE  THINK THIS SECTION IS NOW REDUNDANT 13/2/19   CHECK AND REMOVE
           //                                 \/   ****************************************************************
             //   ****************          ******   
                  
            if(actualBoomAngle >= requiredBoomAngle + 5 ) //|| (actualBoomAngle <= requiredBoomAngle - 15 )) CHANGED TO +5
  
            {
              toggleActivateWinchIN = 1; // " IN " ACTIVATION ON
              toggleActivateWinchOUT = 0;
             }
  
           if (actualBoomAngle <= requiredBoomAngle - 5 ) //   CHANGED FROM -15   TO  -5   AS THIS IS BOOM ANGLES  NOT  WIND ANGLES ( AT PRESENT WIND ANGLES AND BOOM ANGLES DO NOT  " MIRROR  " EACH OTHER !!!
  
           {
             toggleActivateWinchOUT = 1;// " OUT " ACTIVATION ON
             toggleActivateWinchIN = 0;
    
            }
            
           // ************************ END SECTION 
            
         
           
            currentWindA_Toggle = 1; // PLACED HERE AS START UP SECTION REMOVED , MAY BE ABLE TO REMOVE currentWindA_Toggle ALTOGETHER 25/9/19
                
  
          /* AUTOMATED  WINCH  SECTIONS ( EXCLUDING GYBE , TACK , AND MANUAL ) NOTE : NO DIFFERENTIATION IS MADE BETWEEN TRAVELLER AND MAINSHEET IN THESE SECTIONS , THIS IS CARRIED OUT IN THE OUTPUT SECTION FOLLOWING
             *****************       */
          // NOTE : THERE IS A " IN " & " OUT "  WINCH SECTION FOR BOTH " STARBOARD TACK " AND " PORT TACK " .
             
             
       
         if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  " THIS TERMINATES AT END OF THESE SECTIONS         
             {     
          
           
             if(currentWindA_Toggle == 1 )  //  THIS SHOULD ENCOMPASS THE ENTIRE WINCH SECTIONS . ( CAN ONLY RUN IF START UP GREEN LIGHT SECTION HAS RUN )
               {


            
               if(windAngleInt < 180 )// WIND STARBOARD SIDE , TO ALLOW currentWindA TO DETERMINE IF WIND HAS MOVED FORWARD OR AFT 
                 {
              
                  /* BELOW  180 DEGREES WIND STARBOARD SIDE .
                     ******************      */
  
     /*      WINCH  IN  SECTION   STARBOARD TACK
            *****************   */
           // ****************************************************************************************************************************

 if (winchDelayResetTwo == 0 || winchDelayResetTwo == 3 && winchDelayResetThree == 0 || winchDelayResetThree == 3 && winchDelayResetFour == 0 ||  winchDelayResetFour == 3 ) // ENSURING SUBSEQUENT WINCH SECTIONS ARE  UNLOCKED / NOT ACTIVE
           { // ENCOMPASSES THIS ENTIRE IN SECTION , ENSURING OTHER WINCH SECTIONS ARE UNLOCKED / NOT ACTIVE AT THIS TIME
            
                //******************************************
                 if(winchDelayResetOne == 0 )// 12/2/19 THIS SECTION WILL ONLY RUN UNTILL "SHUT OFF SECTION " HAS RUN ONCE , AND GIVEN winchDelayReset THE VALUE OF 3
                    {                        //                               *********************************************
                      if(windAngleInt < currentWindA -15 ) // IF WIND MOVED FORWARD , currentWindA WAS GIVEN A VALUE IN THE ABOVE RUN ONCE GREEN LIGHT SECTION
                        {
                          winchDelayResetOne = 1; // IMPORTANT NOTE THE ONE IN winchDelayResetOne
                        }                       // *********            ***                   ***
                    }
                // ***********************************
                
                
           // WINCH  IN  DELAY SECTION  STARBOARD   // added 16/10/15  to try and stop TRANSIENT WIND FLUCTUATIONS from OPERATING the WINCHES UNNECESSARILY
           // **************************************************************************************************************************** 
                 if( winchDelayResetOne == 3)// RESET BY SHUT OFF SECTION
                   {
                     if(millis() >= winchDelayCount + 500)  // DELAY TIME CAN BE ADJUSTED HERE IF REQUIRED INSERTED HERE FROM BELOW 12/2/19 DELAY IS INITIATED FROM SHUT OFF SECTION
                       {
                       if(windAngleInt < currentWindA -15)// IF WIND MOVED  FORWARD   NOTE: " windAngleInt IS CONSTANTLY UPDATED BY THE " AQUIRE WIND ANGLE " SECTION 
                         {
                          winchDelayResetOne = 1; //  1 IS SPECIFIC TO THIS DELAY SECTION
                         } 
                       }              
                    }
                    
                 // ********************************

                 
              // RESETS  winchDelayResetOne IF  winchDelayResetOne IS STUCK IN FOLLOWING " WINCH IN SECTION " OF " SHUT OFF SECTION " IE ISN'T RELEASED BY THE SHUT OFF SECTION BELOW EG requiredBoomAngle IS NOT ACHIEVED
              //      ******************    *** ******************   *******   
                   if( winchDelayResetOne == 2 )// IF winchDelayResetOne IS WITHIN THE FOLLOWING SECTION
                     {
                      if(millis() >= winchDelayCount + 8000 ) // IF  winchDelayResetOne HAS REMAINED IN THE FOLLOWING WINCH IN SECTION FOR " 8 SECONDS " OR MORE " IE STUCK FOR SOME REASON "
                        { //  THIS  SHUTS OFF , AND ALLOWS SECTION TO RUN AGAIN
                         winch_IN_Signal = 0; // change from winch_IN_Pin 6/2/19 // digitalWrite(winch_IN_pin , LOW ); TURN OFF
                         digitalWrite(ledOrangeIn ,  LOW );
                         winchInToggle = 0; // UNLOCKED 
                         winchDelayCount = millis(); // ADDED HERE FROM ABOVE SECTION 12/2/18
                         winchDelayResetOne = 3 ; // 12/2/19 ALLOWS ABOVE DELAY SECTION TO RUN AGAIN
                         currentWindA =  windAngleInt; // 12/2/19  RESETS currentWindA TO BE TESTED AGAIN BY WINCH DELAY SECTION ABOVE
                         requiredBoomAngle_delayReset = 0; 
                        }
                     }
             //*************************************************
             
            // WINCH IN STARBOARD SIDE SECTION
            // **************************************  
                 if(winchDelayResetOne == 1)     // 16/10/15 FROM ABOVE THE  winchDelayReset = 1; IS SPECIFIC TO THE ABOVE WINCH DELAY SECTION OTHER WINCH SECTIONS CAN NOT RESET THIS SPECIFIC WINCH DELAY SECTION
                   {
                     winchDelayResetOne = 2;//  ALLOW SHUT OFF SECTION TO RUN
                
                    if( actualBoomAngle > requiredBoomAngle)
                       {
                        digitalWrite(ledGreenOK , LOW );
                        digitalWrite(ledBlueOut , LOW );
                        digitalWrite(ledWhiteManualIn , LOW);
                        digitalWrite(ledWhiteManualOut , LOW);
                        digitalWrite(ledOrangeIn , HIGH );
                        winch_IN_Signal = 1; // change from winch_IN_Pin 6/2/19 // digitalWrite (winch_IN_pin,HIGH);
                        winchInToggle = 1;  // USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  "  BELOW IN LED SECTIONS
                        manualWinchINtoggle = 0;// USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                        manualWinchOUTtoggle = 0;// USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                        requiredBoomAngle_delayReset = 1;        // 22/8/17   See Above  REQUIRED BOOM ANGLE SECTION for Note on this 
                       }
                     }
              // *************************************     
                  
              
             // SHUT OFF WINCH   IN   SECTION   STARBOARD ( winchDelayResetOne )
             // ********
           if( winchDelayResetOne == 2 )//  12/2/19 ACTIVATED BY ABOVE WINCH IN SECTION
             {
              if(actualBoomAngle<= requiredBoomAngle)
                {
                 winch_IN_Signal = 0; // change from winch_IN_Pin 6/2/19 // digitalWrite(winch_IN_pin , LOW ); TURN OFF
                 digitalWrite(ledOrangeIn ,  LOW );
                 winchInToggle = 0; // UNLOCKED  USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                 winchDelayCount = millis(); //RESTARTS . ADDED HERE FROM ABOVE SECTION 12/2/18
                 winchDelayResetOne = 3 ; // 12/2/19 ALLOWS ABOVE DELAY SETION TO RUN AGAIN
                 currentWindA =  windAngleInt; // 12/2/19  RESETS currentWindA TO BE TESTED AGAIN BY WINCH DELAY SECTION ABOVE
                 requiredBoomAngle_delayReset = 0; 
                 }
          
             }

         } // END OF "  ENCOMPASSES THIS ENTIRE  IN  STARBOARD SECTION , ENSURING OTHER WINCH SECTIONS ARE UNLOCKED / NOT ACTIVE AT THIS TIME "
         
         //***********************************************************************************************************************************

         
  
  
      
         /*      WINCH   OUT    SECTION ( WIND LESS THAN 180  WIND )   STARBOARD TACK ( winchDelayResetTwo )
                        ******     */ //                               ***************
                 
        

       if (winchDelayResetOne == 0 || winchDelayResetOne == 3 && winchDelayResetThree == 0 || winchDelayResetThree == 3 && winchDelayResetFour == 0 ||  winchDelayResetFour == 3 ) // ENSURING SUBSEQUENT WINCH SECTIONS ARE  UNLOCKED / NOT ACTIVE
           { // ENCOMPASSES THIS ENTIRE IN SECTION , ENSURING OTHER WINCH SECTIONS ARE UNLOCKED / NOT ACTIVE AT THIS TIME
             //                                      *********************************           *************************
           

             if(winchDelayResetTwo == 0 )// 12/2/19 THIS SECTION WILL ONLY RUN UNTILL "SHUT OFF SECTION " HAS RUN ONCE , AND GIVEN winchDelayResetTwo THE VALUE OF 3
                    {                        //                               *********************************************
                      if(windAngleInt > currentWindA +15 ) // IF WIND MOVED AFT , currentWindA WAS GIVEN A VALUE IN THE ABOVE RUN ONCE GREEN LIGHT SECTION
                        {
                          winchDelayResetTwo = 1; // NOTE : winchDelayResetTwo
                        }                       // *********               ***                 
                    }
                // ***********************************

                
                // WINCH DELAY   OUT   SECTION    STARBOARD ( winchDelayResetTwo )    // added 16/10/15  to try and stop TRANSIENT WIND FLUCTUATIONS from OPERATING the WINCHES UNNECESSARILY
           // **************************************************************************************************************************** 
                 if( winchDelayResetTwo == 3)// RESET BY SHUT OFF SECTION
                   {
                     if(millis() >= winchDelayCount + 500)  // INSERTED HERE FROM BELOW 12/2/19 DELAY IS INITIATED FROM SHUT OFF SECTION
                       {
                       if(windAngleInt > currentWindA +15)// IF WIND MOVED AFT  NOTE: " windAngleInt IS CONSTANTLY UPDATED BY THE " AQUIRE WIND ANGLE " SECTION 
                         {
                          winchDelayResetTwo = 1; //  
                         } 
                       }              
                    }
                 // ********************************

                // RESETS  winchDelayResetTwo IF  winchDelayResetTwo IS STUCK IN FOLLOWING " WINCH IN STARBOARD SECTION " IE ISN'T RELEASED BY THE "SHUT OFF SECTION " BELOW EG requiredBoomAngle IS NOT ACHIEVED
              //      ******************    *** ******************  *******   
                   if( winchDelayResetTwo == 2 )// IF winchDelayResetOne IS WITHIN THE FOLLOWING SECTION
                     {
                      if(millis() >= winchDelayCount + 8000 ) // IF  winchDelayResetOne HAS REMAINED IN THE FOLLOWING WINCH IN SECTION FOR " 8 SECONDS " OR MORE " IE STUCK FOR SOME REASON "
                        { //  THIS  SHUTS OFF , AND ALLOWS DELAY SECTION ABOVE TO RUN AGAIN
                         winch_OUT_Signal = 0; // change from winch_IN_Pin 6/2/19 // digitalWrite(winch_IN_pin , LOW ); TURN OFF
                         digitalWrite(ledBlueOut ,  LOW );
                         winchInToggle = 0; // USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  "  BELOW IN LED SECTIONS
                         winchDelayCount = millis(); // RESTARTS  ADDED HERE FROM ABOVE SECTION 12/2/18
                         winchDelayResetTwo = 3 ; // 12/2/19 ALLOWS ABOVE DELAY SETION TO RUN AGAIN
                         currentWindA =  windAngleInt; // 12/2/19  RESETS currentWindA TO BE TESTED AGAIN BY WINCH DELAY SECTION ABOVE
                         requiredBoomAngle_delayReset = 0; 
                        }
                     }
             //*************************************************


           // WINCH   OUT STARBOARD   SIDE SECTION ( winchDelayResetTwo )
            // **************************************  
                 if(winchDelayResetTwo == 1)     // 16/10/15 FROM ABOVE THE  winchDelayReset = 1; IS SPECIFIC TO THE ABOVE WINCH DELAY SECTION OTHER WINCH SECTIONS CAN NOT RESET THIS SPECIFIC WINCH DELAY SECTION
                   {
                     winchDelayResetTwo = 2;// = 0;CHANGED TO =2 12/2/19 // ALLOW SHUT OFF SECTION TO RUN
                
                    if( actualBoomAngle < requiredBoomAngle) // IF LESS THAN REQUIRED
                       {
                        digitalWrite(ledGreenOK , LOW );
                        digitalWrite(ledOrangeIn , LOW ); 
                        digitalWrite(ledWhiteManualIn , LOW);
                        digitalWrite(ledWhiteManualOut , LOW);
                        digitalWrite(ledBlueOut, HIGH );
                        winch_OUT_Signal = 1; // change from winch_IN_Pin 6/2/19 // digitalWrite (winch_IN_pin,HIGH);
                        winchOutToggle = 1;  // USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  "  BELOW IN LED SECTIONS
                        manualWinchINtoggle = 0;// USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                        manualWinchOUTtoggle = 0;// USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                        requiredBoomAngle_delayReset = 1;        // 22/8/17  LOCKS REQUIRED BOOM ANGLE SECTION
                       }
                     }
              // *************************************     
              
              
  
      
             // SHUT OFF WINCH   OUT   SECTION   STARBOARD ( winchDelayResetTwo )
             // ********
               if( winchDelayResetTwo == 2 )//  12/2/19 ACTIVATED BY ABOVE WINCH IN SECTION
                 {
                 if(actualBoomAngle >= requiredBoomAngle)//IF BOOM EQUAL TO OR GREATER THAN REQUIRED , AS BOOM IS LET OUT BOOM ANGLE INCREASES , AND VICE VERSA .
                   {
                    winch_OUT_Signal = 0; // change from winch_IN_Pin 6/2/19 // digitalWrite(winch_IN_pin , LOW ); TURN OFF
                    digitalWrite(ledBlueOut ,  LOW );
                    winchOutToggle = 0; // UNLOCKED  USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                    winchDelayCount = millis(); //RESTARTS . ADDED HERE FROM ABOVE SECTION 12/2/19
                    winchDelayResetTwo = 3 ; // 12/2/19 ALLOWS ABOVE DELAY SETION TO RUN AGAIN
                    currentWindA =  windAngleInt; // 12/2/19  RESETS currentWindA TO BE TESTED AGAIN BY WINCH DELAY SECTION ABOVE
                    requiredBoomAngle_delayReset = 0; 
                   }
          
                }  
              
       
       
         
         }// END OF " ENCOMPASSES THIS ENTIRE OUT SECTION , ENSURING OTHER WINCH SECTIONS ARE UNLOCKED / NOT ACTIVE AT THIS TIME " 
       }  // THIS TERMINATES THE  LESS  THAN 180 DEGREES WIND( STARBOARD ) WINCH SECTION .
           
       //*****************************************************************************************************************     
         
            
            
          
            /* WIND ABOVE  180  DEGREES PORT TACK
               ********************       */
               
                 
    if (windAngleInt > 180 )
       { // ENCOMPASSES ENTIRE PORT SECTIONS  WIND > 180
               
     /* WINCH  IN   SIGNAL SECTION    PORT TACK  ( winchDelayResetThree )   ( WIND ANGLE GREATER THAN 180 )(NOTE WINCH OUTPUT IS DELIVERED BY THE WINCH OUTPUT SECTIONS THAT FOLLOW THE TACK SECTION )
              ****                   ******    */
                 
     if (winchDelayResetTwo == 0 || winchDelayResetTwo == 3 && winchDelayResetOne == 0 || winchDelayResetOne == 3 && winchDelayResetFour == 0 ||  winchDelayResetFour == 3 ) // ENSURING SUBSEQUENT WINCH SECTIONS ARE  UNLOCKED / NOT ACTIVE
        { // ENCOMPASSES THIS ENTIRE IN SECTION , ENSURING OTHER WINCH SECTIONS ARE UNLOCKED / NOT ACTIVE AT THIS TIME


          //**********************************************************
           if(winchDelayResetThree == 0 )// 12/2/19 THIS SECTION WILL ONLY RUN UNTILL "SHUT OFF SECTION " HAS RUN ONCE , AND GIVEN winchDelayResetTwo THE VALUE OF 3
                    {                        //                               *********************************************                               **********
                      if(windAngleInt > currentWindA +15 ) // WIND MOVED FORWARD  currentWindA WAS GIVEN A VALUE IN THE ABOVE RUN ONCE GREEN LIGHT SECTION
                        {
                          winchDelayResetThree = 1; // IMPORTANT NOTE THE 1 IN winchDelayReset1
                        }                       // *********         ***                 ***
                    }
            
        //**************************************************************
                 
      


         // WINCH   IN   DELAY SECTION   PORT    // added 16/10/15  to try and stop TRANSIENT WIND FLUCTUATIONS from OPERATING the WINCHES UNNECESSARILY
           // **************************************************************************************************************************** 
                 if( winchDelayResetThree == 3)// RESET BY SHUT OFF SECTION
                   {
                     if(millis() >= winchDelayCount + 500)  // INSERTED HERE FROM BELOW 12/2/19 DELAY IS INITIATED FROM SHUT OFF SECTION
                       {
                       if(windAngleInt > currentWindA +15)// IF WIND MOVED FORWARD  NOTE: " windAngleInt IS CONSTANTLY UPDATED BY THE " AQUIRE WIND ANGLE " SECTION 
                         {
                          winchDelayResetThree = 1; //  1 IS SPECIFIC TO THIS DELAY SECTION
                         } 
                       }              
                    }
                    
                 // ********************************



              // RESETS  winchDelayResetThree IF  winchDelayResetThree IS STUCK IN FOLLOWING " WINCH IN SECTION " IE ISN'T RELEASED BY THE "SHUT OFF SECTION " BELOW EG requiredBoomAngle IS NOT ACHIEVED
              //      ******************    *** ******************  *******   
                   if( winchDelayResetThree == 2 )// IF winchDelayResetOne IS WITHIN THE FOLLOWING SECTION
                     {
                      if(millis() >= winchDelayCount + 8000 ) // IF  winchDelayResetOne HAS REMAINED IN THE FOLLOWING WINCH IN SECTION FOR " 8 SECONDS " OR MORE " IE STUCK FOR SOME REASON "
                        { //  THIS  SHUTS OFF , AND ALLOWS SECTION TO RUN AGAIN
                         winch_IN_Signal = 0; // change from winch_IN_Pin 6/2/19 // digitalWrite(winch_IN_pin , LOW ); TURN OFF
                         digitalWrite(ledOrangeIn ,  LOW );
                         winchInToggle = 0; // UNLOCKED 
                         winchDelayCount = millis(); // STARTS ,ADDED HERE FROM ABOVE SECTION 12/2/18
                         winchDelayResetThree = 3 ; // 12/2/19 ALLOWS ABOVE DELAY SETION TO RUN AGAIN
                         currentWindA =  windAngleInt; // 12/2/19  RESETS currentWindA TO BE TESTED AGAIN BY WINCH DELAY SECTION ABOVE
                         requiredBoomAngle_delayReset = 0; // UNLOCKS REQUIRED BOOM ANGLE SECTION
                        }
                     }
             //*************************************************


          // WINCH IN PORT SIDE SECTION
            // **************************************  
                 if(winchDelayResetThree == 1)     // 16/10/15 FROM ABOVE THE  winchDelayReset = 1; IS SPECIFIC TO THE ABOVE WINCH DELAY SECTION OTHER WINCH SECTIONS CAN NOT RESET THIS SPECIFIC WINCH DELAY SECTION
                   {
                     winchDelayResetThree = 2;// = 0;CHANGED TO =2 12/2/19 //UNLOCK  SHOULD THIS , AND THE ABOVE LINE BE PLACED IN SHUT DOWN SECTION BELOW ???????????????????????????????????????????????????????????????????
                
                    if( actualBoomAngle > requiredBoomAngle)
                       {
                        digitalWrite(ledGreenOK , LOW );
                        digitalWrite(ledBlueOut , LOW );
                        digitalWrite(ledWhiteManualIn , LOW);
                        digitalWrite(ledWhiteManualOut , LOW);
                        digitalWrite(ledOrangeIn , HIGH );
                        winch_IN_Signal = 1; // change from winch_IN_Pin 6/2/19 // digitalWrite (winch_IN_pin,HIGH);
                        winchInToggle = 1;  //LOCKED  USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  "  BELOW IN LED SECTIONS
                        manualWinchINtoggle = 0;// USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                        manualWinchOUTtoggle = 0;// USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                        requiredBoomAngle_delayReset = 1;        // LOCKS 22/8/17   See Above  REQUIRED BOOM ANGLE SECTION for Note on this 
                       }
                     }
              // *************************************  


             // SHUT OFF WINCH  IN  SECTION  PORT ( winchDelayResetThree )
             // ********
           if( winchDelayResetThree == 2 )//  12/2/19 ACTIVATED BY ABOVE WINCH IN SECTION
             {
              if(actualBoomAngle<= requiredBoomAngle)
                {
                 winch_IN_Signal = 0; // change from winch_IN_Pin 6/2/19 // digitalWrite(winch_IN_pin , LOW ); TURN OFF
                 digitalWrite(ledOrangeIn ,  LOW );
                 winchInToggle = 0; // UNLOCKED  USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                 winchDelayCount = millis(); //RESTARTS . ADDED HERE FROM ABOVE SECTION 12/2/18
                 winchDelayResetThree = 3 ; // 12/2/19 ALLOWS ABOVE DELAY SETION TO RUN AGAIN
                 currentWindA =  windAngleInt; // 12/2/19  RESETS currentWindA TO BE TESTED AGAIN BY WINCH DELAY SECTION ABOVE
                 requiredBoomAngle_delayReset = 0; // UNLOCKS " REQUIRED BOOM ANGLE SECTION "
                 }
          
             }

         
        }// END OF " ENCOMPASSES THIS ENTIRE  IN PORT SECTION , ENSURING OTHER WINCH SECTIONS ARE UNLOCKED / NOT ACTIVE AT THIS TIME " 
        // **********************************************************************************************************************************************
        
        
  
  
    /* AUTO WINCH  OUT  SIGNAL SECTION   PORT TACK ( winchDelayResetFour )  ( NOTE WINCH OUTPUTS ARE CARRIED OUT BY THE WINCH OUPUT SECTION THAT FOLLOWS TACK SECTION )
                  *****                 ******     */

      if (winchDelayResetTwo == 0 || winchDelayResetTwo == 3 && winchDelayResetOne == 0 || winchDelayResetOne == 3 && winchDelayResetThree == 0 ||  winchDelayResetThree == 3 ) // ENSURING SUBSEQUENT WINCH SECTIONS ARE  UNLOCKED / NOT ACTIVE
         { // ENCOMPASSES THIS ENTIRE   OUT  PORT  SECTION , ENSURING OTHER WINCH SECTIONS ARE UNLOCKED / NOT ACTIVE AT THIS TIME

       
           //**********************************************************
                 if(winchDelayResetFour == 0 )// 12/2/19 THIS SECTION WILL ONLY RUN UNTILL "SHUT OFF SECTION " HAS RUN ONCE , AND GIVEN winchDelayResetTwo THE VALUE OF 3
                    {                        //                               *********************************************                               **********
                      if(windAngleInt < currentWindA -15 ) // IF WIND GONE AFT  currentWindA WAS GIVEN A VALUE IN THE ABOVE RUN ONCE GREEN LIGHT SECTION
                        {
                          winchDelayResetFour = 1; // IMPORTANT NOTE THE 1 IN winchDelayReset1
                        }                       // *********         ***                 ***
                     }
            
        //**************************************************************

            
           // WINCH   OUT   DELAY SECTION   PORT ( winchDelayResetFour )   // added 16/10/15  to try and stop TRANSIENT WIND FLUCTUATIONS from OPERATING the WINCHES UNNECESSARILY
           // **************************************************************************************************************************** 
                  if( winchDelayResetFour == 3)// RESET BY SHUT OFF SECTION
                    {
                    if(millis() >= winchDelayCount + 500)  // INSERTED HERE FROM BELOW 12/2/19 DELAY IS INITIATED FROM SHUT OFF SECTION
                       {
                       if(windAngleInt < currentWindA -15)// IF WIND MOVED FORWARD , NOTE: " windAngleInt IS CONSTANTLY UPDATED BY THE " AQUIRE WIND ANGLE " SECTION 
                         {
                          winchDelayResetFour = 1; //  
                         } 
                       }              
                    }
                    
                 // ********************************

            // RESETS  winchDelayResetFour IF  winchDelayResetFour IS STUCK IN FOLLOWING " WINCH IN SECTION "OR " SHUT OFF SECTION " IE ISN'T RELEASED BY THE "SHUT OFF SECTION " BELOW EG requiredBoomAngle IS NOT ACHIEVED
              //      ******************    *** ******************  *******   
                   if( winchDelayResetFour == 2 )// IF winchDelayResetOne IS WITHIN THE FOLLOWING SECTION
                     {
                      if(millis() >= winchDelayCount + 8000 ) // IF  winchDelayResetOne HAS REMAINED IN THE FOLLOWING WINCH IN SECTION FOR " 8 SECONDS " OR MORE " IE STUCK FOR SOME REASON "
                        { //  THIS  SHUTS OFF , AND ALLOWS SECTION TO RUN AGAIN
                         winch_OUT_Signal = 0; // change from winch_IN_Pin 6/2/19 // digitalWrite(winch_IN_pin , LOW ); TURN OFF
                         digitalWrite(ledBlueOut ,  LOW );
                         winchOutToggle = 0; // UNLOCKED  USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  "  BELOW IN LED SECTIONS
                         winchDelayCount = millis(); // STARTS 
                         winchDelayResetFour = 3 ; // 12/2/19 ALLOWS ABOVE DELAY SETION TO RUN AGAIN
                         currentWindA =  windAngleInt; // 12/2/19  RESETS currentWindA TO BE TESTED AGAIN BY WINCH DELAY SECTION ABOVE
                         requiredBoomAngle_delayReset = 0; // UNLOCKS REQUIRED BOOM ANGLE SECTION
                        }
                     }
             //*************************************************


             // WINCH  OUT  PORT TACK (winchDelayResetFour) SECTION
            // **************************************  
                 if(winchDelayResetFour == 1)     // 16/10/15 FROM ABOVE THE  winchDelayReset = 1; IS SPECIFIC TO THE ABOVE WINCH DELAY SECTION OTHER WINCH SECTIONS CAN NOT RESET THIS SPECIFIC WINCH DELAY SECTION
                   {
                     winchDelayResetFour = 2;// = 0;CHANGED TO =2 12/2/19 //UNLOCK  SHOULD THIS , AND THE ABOVE LINE BE PLACED IN SHUT DOWN SECTION BELOW ???????????????????????????????????????????????????????????????????
                
                    if( actualBoomAngle < requiredBoomAngle)
                       {
                        digitalWrite(ledGreenOK , LOW );
                        digitalWrite(ledOrangeIn , LOW ); 
                        digitalWrite(ledWhiteManualIn , LOW);
                        digitalWrite(ledWhiteManualOut , LOW);
                        digitalWrite(ledBlueOut, HIGH );
                        winch_OUT_Signal = 1; // change from winch_IN_Pin 6/2/19 // digitalWrite (winch_IN_pin,HIGH);
                        winchOutToggle = 1;  //LOCKED  USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  "  BELOW IN LED SECTIONS
                        manualWinchINtoggle = 0;// USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                        manualWinchOUTtoggle = 0;// USED IN "  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                        requiredBoomAngle_delayReset = 1;        // 22/8/17  LOCKS REQUIRED BOOM ANGLE SECTION
                       }
                     }
              // *************************************  


               // SHUT OFF WINCH  OUT  SECTION  PORT TACK (winchDelayResetFour )
             // ********
               if( winchDelayResetFour == 2 )//  12/2/19 ACTIVATED BY ABOVE WINCH IN SECTION
                 {
                 if(actualBoomAngle >= requiredBoomAngle)// AS BOOM IS LET OUT BOOM ANGLE INCREASES , AND VICE VERSA .
                   {
                    winch_OUT_Signal = 0; // change from winch_IN_Pin 6/2/19 // digitalWrite(winch_IN_pin , LOW ); TURN OFF
                    digitalWrite(ledBlueOut ,  LOW );
                    winchOutToggle = 0; // UNLOCKED  USED IN " MANUAL WINCH OUTPUT SECTION SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY  " BELOW IN LED SECTIONS
                    winchDelayCount = millis(); //RESTARTS . ADDED HERE FROM ABOVE SECTION 12/2/19
                    winchDelayResetFour = 3 ; // 12/2/19 ALLOWS ABOVE DELAY SETION TO RUN AGAIN
                    currentWindA =  windAngleInt; // 12/2/19  RESETS currentWindA TO BE TESTED AGAIN BY WINCH DELAY SECTION ABOVE
                    requiredBoomAngle_delayReset = 0; 
                   }
          
                 }  

             }// END OF  " ENCOMPASSES THIS ENTIRE   OUT  PORT TACK SECTION , ENSURING OTHER WINCH SECTIONS ARE UNLOCKED / NOT ACTIVE AT THIS TIME "
            } // THIS TERMINATES THE ABOVE 180 DEGREE WINCH SECTION
             
          } // END currentWindA_Toggle == 1  THIS TERMINATES ALL WINCH SECTIONS .
         } // THIS TERMINATES auto_winch_isolate  AT START OF WINCH SECTIONS . 9/2/19
          
        // *****************************************END OF ORIGINAL AUTO SHEETING FROM MISS ISLE TOO " sketch_sep04b " ( WITH ONLY , winch_IN_pin AND winch_OUT_pin BEING CHANGED FOR winch_IN_Signal AND winch_OUT_Signal .
         
            
            
            
            
            /*  GYBE  and  TACK  SECTIONS FOLLOW   THESE SECTIONS WORK INDEPENDENTLY OF THE WINCH OUTPUT SECTIONS WHICH FOLLOW .  4/2/19
          *************************************      ********   
          */
          
    /* Section To Determine If GYBE Situation
       *************************************  
       *************************************  
     */
   
   /*  if ( gybeInProgressLock == 0) //  0 IS UNLOCKED  , locks when gybe in progress
        {
         if( windAngleInt > 90 && windAngleInt < 270 ) // ENSURING WIND IS AFT HALF . 
           {                                          // **************************
            windAft_or_forward = 1; //  1 = aft ,   ( 2 = forward) .   
           }
        }


       if ( windAft_or_forward == 1) // 1 = AFT if wind is Aft
       {

         // THIS SECTION INITIATES THE GYBE WHEN WIND HAS CROSSED TO THE PORT SIDE .
      //    ************************************************************************
          if( boomSide == 1) // IF BOOM SIDE IS PORT , 1 = PORT  
          {
            if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  "
               {
            // IMPORTANT PLACE ANOTHER windAngleInt CHECK SECTION HERE , THIS CAN ONLY WORK IF THE windAngleInt IS RECHECKED HERE ( AS OTHERWISE IT CLEARLY WOULD STILL BE WITHIN THE windSide =1 area as it has just designated it as such ) NO BECAUSE IT'S LOCKED BY THE WINDSIDE LOCK SECTION ABOVE
            //                                                                                                                                                                                                                               ************************************************************
              if( windAngleInt < 360 && windAngleInt >= 195 ) // CHECKING TO SEE IF THE WIND HAS GONE INTO THE OTHER SECTOR ( ie port ) The  195 is 15 degrees into the windSide = 2   THE 15 DEGREE FIGURE CAN BE ALTERED TO THE ACCEPTED FIGURE MORE OR LESS !! 
                {                                               //     ****    ********************************                                 ********                   *************
                if (gybeInProgressLock == 0  )
                   {
                    gybeInProgressLock = 1; // GYBE LOCK  ON , Gybing From PORT TACK ( Boom on Starboard Side)  to STARBOARD TACK (Boom on Port Side )
                    // Here the WIND was on PORT the BOOM would be on STARBOARD as the boat has Gone Through the Wind I need to take the boom to PORT , But First I must ensure The Main Sheet is
                    // .. Hauled IN FULLY  
                    if (digitalRead(  auto_winch_isolate  )==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  "
                       { 
                        digitalWrite(winch_IN_Mainsheet_Pin , HIGH); // digitalWrite ( winch_IN_pin , HIGH ); // HAUL IN MAINSHEET ( using mainsheet to start as boom will be out as boat will be running deep on the wind )
                       }
                   }
                 }
               }
            }
            // ********************************************************************

          // THIS CARRYS OUT THE GYBE INTIATED BY THE ABOVE SECTION NOTE: DETERMINE TYPE OF SWITCH TO DECIDE ON " HIGH " OR " LOW "
         //  **********************************************************
          
          if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  "
              {
            if( gybeInProgressLock == 1)// IF GYBE IN PROGRESS LOCKED FROM THE ABOVE SECTION
               {
                  if (digitalRead( mainSheet_IN_Stop == LOW ))//IF MAINSHEET IN.  IMPORTANT: I'VE USED " LOW " DEPENDING ON SWITCH IT MAY BE " HIGH "  (PERHAPS MAINSHEET and TRAVELLER NEED TO BE HAULED SIMULTANEOUSLY ??????? )
                     {                            //                             ********************************************************
                      digitalWrite (travellerDriveRight , HIGH ); // Take BOOM to the RIGHT .
                      digitalWrite(winch_IN_Mainsheet_Pin, LOW); // digitalWrite ( winch_IN_pin , LOW ); // winch will be stopped by the switch , but this will turn off the call from the arduino .
                     }
                      if (digitalRead(travellerStopRight == LOW ) ) // IF Traveller  IS FULLY OUT TO THE RIGHT . LOW = MADE   SEE HALL SWITCH MP101401
                         {
                          digitalWrite( travellerDriveRight , LOW ); // traveller will be stopped by the switch , but this turns off call from arduino .
                          digitalWrite( winch_OUT_Mainsheet_pin , HIGH);//winch_OUT_Signal = 1; // 0 = OUT Signal OFF , 1 = OUT Signal ON .// digitalWrite ( winch_OUT_pin , HIGH ); // let out mainsheet on PORT side .
                          
                          if (digitalRead (mainSheet_OUT_Stop == LOW )) // IF Mainsheet Fully Out .
                             {
                              digitalWrite( winch_OUT_Mainsheet_pin , LOW);////digitalWrite (  winch_OUT_pin , LOW ); // winch will be stopped by the switch , but this will turn off call from arduino .
                              gybeInProgressLock = 0; // RELEASE THE  GYBE_IN_PROGRESS  LOCK
                             }
                         }
                      
                }
              }
               // ****************************************************************
            
            
            
       // THIS SECTION INITIATES A GYBE WHEN THE WIND HAS CROSSED TO THE STARBOARD SIDE
       // *******************************************************************************
        if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  "
           {
          if(boomSide == 2) // IF BOOM SIDE IS STARBOARD ? ,Starboard = 2 ;
          {
            if ( windAngleInt > 0 && windAngleInt <= 165 ) //  CHECKING TO SEE IF THE WIND HAS GONE INTO THE OTHER SECTORS ( ie STARBOARD ) The 165 is 15 degrees into the windside = 1 THE 15 DEGREE FIGURE CAN BE ALTERED TO THE ACCEPTED FIGURE BE THAT MORE OR LESS !!
               {

                if (gybeInProgressLock == 0 || gybeInProgressLock == 2 )
                   {
                    gybeInProgressLock = 2; // jybing from Port to Starboard
            // Here the wind was on Port the Boom would be on Starboard as the boat has Gone Through the Wind I need to take the boom to Port , But First I must ensure The Main Sheet is
            // .. Hauled IN FULLY 
                    
                      
                    digitalWrite(winch_IN_Mainsheet_Pin , HIGH);// // digitalWrite ( winch_IN_pin , HIGH ); // haul in mainsheet 
                    
                   }
                }
             }
           }
        // ****************************************************************************

      // THIS SECTION CARRYS OUT THE GYBE INITIATED BY THE ABOVE SECTION
      // **************************************************************** 
              if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  "
                 {     
                 if(gybeInProgressLock == 2)
                   {    
                    if (digitalRead( mainSheet_IN_Stop == LOW) ) // if mainsheet in.    (PERHAPS MAINSHEET and TRAVELLER NEED TO BE HAULED SIMULTANEOUSLY ??????? )
                       {
                        digitalWrite (travellerDriveLeft , HIGH ); // take boom to the right 
                        digitalWrite(winch_IN_Mainsheet_Pin, LOW);   // digitalWrite ( winch_IN_pin , LOW ); // winch will be stopped by the switch , but this will turn off the call from the arduino .
                       }
                        if (digitalRead(travellerStopLeft == LOW ) ) // IF Traveller  IS FULLY OUT TO THE LEFT . LOW = MADE   SEE HALL SWITCH MP101401
                           {
                            digitalWrite( travellerDriveLeft , LOW );  // traveller will be stopped by the switch , but this turns off call from arduino .
                            digitalWrite  (winch_OUT_Mainsheet_pin ,HIGH);  // digitalWrite ( winch_OUT_pin , HIGH );     // let out mainsheet on Starboard side .

                            if (digitalRead(mainSheet_OUT_Stop == LOW) )          // IF Mainsheet Fully Out .
                               {
                                digitalWrite  (winch_OUT_Mainsheet_pin ,LOW);   //digitalWrite (  winch_OUT_pin , LOW ); // winch will be stopped by the switch , but this will turn off call from arduino .
                                gybeInProgressLock = 0;                //  RELEASE THE GYBE_IN_PROGRESS LOCK
                               }  
                           }
                       
                     }
                 }
           // *************************************************************         
                     
             
           } */ // END OF AFT = 1 ( Wind is Aft )  REDACTED GYBE SECTION FOR TEST PURPOSES ON 17/8/19
             //  *************************************************************END OF GYBE SECTION 
    
       
   
       //      FORWARD TACKING SECTIONS
       //     ***************************
       
        //       Section to determine if a TACK Situation
        //      ****************************************
               
        /*       if ( windAngleInt < 90 || windAngleInt > 270)  // making sure wind is forward of beam 
                  {
                    windAft_or_forward = 2; // Wind is Forward
                  }
                   
                    if(windAft_or_forward == 2)// IS WIND FORWARD ?
                       {

                        
                    // WIND CROSSED TO PORT SIDE ; SECTION
                   // *********************************************************************
                      if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  "
                         {
                       if ( boomSide == 1) // IS BOOM SIDE PORT ?
                          {
                          if (gybeInProgressLock == 0 ) //GYBE NOT IN PROGRESS ? IF WE FEEL WE NEED TO DO SOMETHING IN THE TACK SECTION WE MAY NEED TO ADD ADDITION NUMBERS TO THIS VARIABLE EG 3 AND 4
                             {
                              if(tackInProgressLock == 0 ) // NOT LOCKED  .
                                {
                                if ( windAngleInt < 345 && windAngleInt > 270 ) // HAS WIND MOVED INTO PORT SECTOR ( May Need to Alter 345 to be Closer to 360 ??? )  NOTE 345 is 15 Degrees INTO the PORT SECTOR , Assuring this is a tack , NUMBER CAN BE ALTERED IF FELT TOO GREAT OR SMALL , also the 270 is not essential as it is covered in the section above ensuring wind is forward of beam , so just belt and braces .
                                   {
                                   // IF TACKING THE BOAT , THE  TRAVELLER ' SHOULD BE CENTERED '  , THEREFORE THERE SHOULD BE NO NEED TO TAKE ANY ACTION ( MAYBE )
                                   //Serial.print ( " Tack From windSide 1 to WindSide 2 .... "); // TESTING ONLY !!!!
                                   if(boomSide == 1)//CHECKING AGAIN . IF BOOM IS PORT (boomSide = 1 = Port) BOOM SHOULD HAVE MOVED TO STARBOARD AS BOAT HAS PASSED THROUGH WIND ( ie boomSide Should = 2 , eg Starboard )
                                     {               //                                        ***********************************
                                      if(digitalRead( mainSheet_IN_Stop == LOW )) // mainsheet is hauled IN .
                                         {
                                          digitalWrite (travellerDriveRight , HIGH ); // DRIVES TRAVELLER TO RIGHT IF BOOM HASN'T CROSSED DURING TACK !
                                          tackInProgressLock = 1; // LOCKS BOTH TACK SECTIONS , BUT  NOT THE GYBE SECTIONS , HOPE THIS IS CORRECT !! ?? 
                                         }
                                     }
                                   }
                                 }
                              }
                            }
                          }
                         //**********************************************************************
                         
                     // THIS SHUTS DOWN THE TRAVELLER WHEN BOOM HAS REACHED THE STARBOARD SIDE , AS INITIATED BY THE ABOVE SECTION
                     // ***********************************************************************************************************
                      if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  "
                          {
                            if(tackInProgressLock == 1 ) // IS TACK FROM STARBOARD TO PORT IN PROGRESS ? INITIATED FROM ABOVE SECTION .
                                {
                                  if(boomSide == 2 ) // IF BOOM HAS TACKED  boomSide 2 IS BOOM ON STARBOARD SIDE , THEREFORE ON PORT TACK .
                                  {
                                    digitalWrite(travellerDriveRight , LOW ); // SHUTS DOWN TRAVELLER FROM ABOVE SECTION INTIALISATION 
                                    tackInProgressLock = 0;
                                  }
                                }
                          }
                    //*************************************************************************************************************
                          

              //  WIND CROSSED TO STARBOARD SIDE SECTION
              // ******************************************
                     if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  "
                        {  
                       if ( boomSide == 2 ) // IS BOOM SIDE STARBOARD ?
                          {
                          if (gybeInProgressLock == 0 ) // IF GYBE NOT IN PROGRESS ? IF WE FEEL WE NEED TO DO SOMETHING IN THE TACK SECTION WE MAY NEED TO ADD ADDITION NUMBERS TO THIS VARIABLE EG 3 AND 4
                             {
                              if(tackInProgressLock == 0 ) //CHECK NOT LOCKED ?
                                {
                                if ( windAngleInt > 15 && windAngleInt < 90 ) // HAS WIND MOVED INTO STARBOARD SECTOR ?(May Need To Alter 15 to be Closer To 360(0) )  15 is 15 Degrees INTO The STARBOARD SECTOR , ENSURING THIS IS A TACK .
                                   {
                                 // IF TACKING THE BOAT , THE  TRAVELLER ' SHOULD BE CENTERED '  , THEREFORE THERE 'SHOULD ' BE NO NEED TO TAKE ANY ACTION ( MAYBE )
                                 // Serial.print ( " Tack From windSide 2 to WindSide 1 .... "); // TESTING ONLY !!!!
                                     if(boomSide == 2)//CHECKING AGAIN IF BOOM IS STARBOARD (boomSide = 2 = Starboard) BOOM 'SHOULD ' HAVE MOVED TO PORT AS BOAT HAS PASSED THROUGH WIND ( ie boomSide Should = 1 , eg Port )
                                     {               //                                        ***********************************
                                      if(digitalRead(mainSheet_IN_Stop == LOW)) // mainsheet Hauled IN .
                                        {
                                         digitalWrite (travellerDriveLeft , HIGH ); // DRIVES TRAVELLER TO LEFT IF BOOM HASN'T CROSSED DURING TACK !
                                         tackInProgressLock = 2; // LOCKS BOTH TACK SECTIONS , BUT  NOT THE GYBE SECTIONS , HOPE THIS IS CORRECT !! ?? 
                                        }
                                     }
                                   }
                                }
                              }
                            }
                          }
                 // ************************************************************
                 
                          // THIS SHUTS DOWN TRAVELLER IF BOOM HAS REACHED THE PORT SIDE AS INITIATED BY THE ABOVE SECTION
                          // **********************************************************************************************
                         if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  "
                            {
                           if(tackInProgressLock == 2 ) // IS TACK FROM STARBOARD TO PORT IN PROGRESS ? INITIATED FROM ABOVE SECTION .
                                {
                                  if(boomSide == 1 ) // IF BOOM HAS TACKED  boomSide 1 IS BOOM ON PORT SIDE , THEREFORE ON STARBOARD TACK .
                                  {
                                    digitalWrite(travellerDriveLeft , LOW ); // SHUTS DOWN TRAVELLER FROM ABOVE SECTION INTIALISATION 
                                    tackInProgressLock = 0;
                                  }
                                }
                            }
                          //*************************************************************************************************
                          
              }// END OF WIND AFT OR FORWARD ( forward ) SECTION

           */   // ************************************************ END TACK SECTION // REDACTED TACK SECTION FOR TEST PURPOSES ON 17/8/19
        //*****************************************************************************************************************************************************

        
        


        // ****************************************************************************************************************************************************
        // THESE SECTIONS CONTROL  OUTPUTS TO MAINSHEET WINCH AND TRAVELLER , THAT ARE AUTOMATED, BUT NOT CONTROLLED BY THE GYBE OR TACK SECTIONS
       // ******************************************************************       *************
       
      
       // ANYTHING MISSED BY THE FOLLOWING SECTIONS WILL BE STOPPED HERE
       // ***************************************************************
        if(digitalRead( travellerStopRight) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP RIGHT IS MADE
                        { 
                          digitalWrite ( travellerDriveRight , LOW );
                        }
        if(digitalRead( travellerStopLeft) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP RIGHT IS MADE
                        { 
                          digitalWrite ( travellerDriveLeft , LOW );
                        }
         if(digitalRead( mainSheet_IN_Stop) == LOW) // LOW = MADE  RS No. 4994678 Push Switch aft on coach roof
                        { 
                          digitalWrite ( winch_IN_Mainsheet_Pin , LOW  );
                        }
          if(digitalRead( mainSheet_OUT_Stop) == LOW) // LOW = MADE  RS No. 4994678 Push Switch forward on coach roof
                        { 
                          digitalWrite ( winch_OUT_Mainsheet_pin , LOW  );
                        }
        //***************************************************************
        
          
             // AUTO WINCH "OUT" OUTPUT Section
             // *****************************
       if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  " 
          { 
          if(tackInProgressLock == 0 && gybeInProgressLock == 0) // IF NO GYBE AND NO TACK IN PROGRESS 
          {
            if( manualWinch_IN_Lock == 0 &&  manualWinch_OUT_Lock == 0   )// IF MANUAL WINCH SECTIONS NOT ACTIVE
               {                                                          // ***********************************
               if( winch_OUT_Signal == 1 ) // TURN  ON  " OUT "  WINCHING   THIS IS THE " AUTOMATED " SIGNAL .
                 {
                  //TRAVELLER OUT
                  if(boomSide == 2 ) // STARBOARD
                    {
                      if(digitalRead (travellerStopRight) == HIGH ) // HIGH = NOT MADE see Hall Sensor MP101401
                        {
                         digitalWrite(travellerDriveLeft , LOW );// LEFT OFF
                         digitalWrite( winch_OUT_Mainsheet_pin , LOW );// MAINSHEET OUT OFF
                         digitalWrite( winch_IN_Mainsheet_Pin , LOW );// MAINSHEET IN OFF
                         digitalWrite(travellerDriveRight , HIGH );//   winch_Traveller_right .
                         autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                        }
                    }
                  else if (boomSide == 1 ) //PORT
                   {
                    if(digitalRead (travellerStopLeft) == HIGH ) // HIGH = NOT MADE see Hall Sensor MP101401
                      {
                       digitalWrite(travellerDriveRight , LOW );// OFF
                       digitalWrite( winch_OUT_Mainsheet_pin , LOW );//OFF
                       digitalWrite( winch_IN_Mainsheet_Pin , LOW );//OFF
                       digitalWrite(travellerDriveLeft , HIGH );// winch_Traveller_left .
                       autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                      }
                   }
                 
                  // MAIN OUT
                  if(boomSide == 1)//IF BOOM ON PORT
                    {
                    if( digitalRead ( travellerStopLeft ) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP LEFT IS MADE
                      {
                        if(digitalRead( mainSheet_OUT_Stop) == LOW ) // Ensuring Main Sheet is NOT at the OUT Stop
                           {
                            digitalWrite(travellerDriveLeft , LOW );// off
                            digitalWrite(travellerDriveRight , LOW );// off 
                            digitalWrite (winch_IN_Mainsheet_Pin , LOW);//off
                            digitalWrite( winch_OUT_Mainsheet_pin , HIGH );//  winch_OUT_Mainsheet .
                            autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                           }
                      }
                    }
                    else if (boomSide == 2 )//IF BOOM ON STARBOARD
                      {
                      if(digitalRead( travellerStopRight) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP RIGHT IS MADE
                        {
                          if(digitalRead( mainSheet_OUT_Stop) == LOW ) // Ensuring Main Sheet is NOT at the OUT Stop
                            {
                             digitalWrite(travellerDriveLeft , LOW );// off
                             digitalWrite(travellerDriveRight , LOW );// off 
                             digitalWrite (winch_IN_Mainsheet_Pin , LOW);//off
                             digitalWrite( winch_OUT_Mainsheet_pin , HIGH );//  winch_OUT_Mainsheet .
                             autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                            }
                        }
                      }
                } // END OF  if( winch_OUT_Signal == 1 )
                // ************************************
                
             // TURNS OFF THE ABOVE SECTION
             // ***************************
              if(winch_OUT_Signal == 0 )  //  TURN OFF  " OUT " WINCHING, CONTROLED BY ABOVE AUTO WINCHING SECTION  
                {
                 if(autoWinchOutStarted == 0) // IF THE AUTO WINCH OUT HAS STARTED BY ABOVE AUTO WINCH "OUT" OUTPUT SECTION
                 {
                 autoWinchOutStarted = 1; // TURNS OFF THIS SECTION. IMPORTANT NOTE : THIS STOPS THIS SECTION FROM CONTINUALLY SWITCHING OFF TRAVELLER AND MAINSHEET 
                 digitalWrite(travellerDriveRight , LOW ); //                         *******************************************************************************
                 digitalWrite(travellerDriveLeft , LOW );
                 digitalWrite(  winch_OUT_Mainsheet_pin ,LOW );
                 digitalWrite(  winch_IN_Mainsheet_Pin ,LOW );
                 }
                }
                //***************************************************************************
                
                //BELT AND BRACES
                //*****************************************************************
                if(digitalRead( mainSheet_OUT_Stop) == LOW ) //Push Button Switches on Coach Roof RS No. 4994678 // HIGH When NOT Made (between brown & black ) BELT & BRACES IF MAIN SHEET AT OUT STOP
                            {
                             
                              digitalWrite(  winch_OUT_Mainsheet_pin ,LOW );
                              
                            }
                 //******************************************************************* 
               
                 
                  if( digitalRead (travellerStopLeft ) == LOW ) // AND IF TRAVELLER IS AT LEFT STOP
                    {
                      digitalWrite(travellerDriveLeft , LOW); // TURN OFF DRIVE LEFT 
                      
                     }
                  
                //*****************************************************************  
                
                    if( digitalRead (travellerStopRight ) == LOW ) // AND IF TRAVELLER IS AT RIGHT STOP
                      {
                        digitalWrite(travellerDriveRight , LOW); // TURN OFF DRIVE RIGHT 
                        
                      }
                  
               // *****************************************************************
       //   
       //  
          // END OF WINCH OUT SECTION
          // *********************************************************************


            /*AUTO  WINCH  " IN  " OUTPUT SECTION
           ************************ */
           
             if(winch_IN_Signal == 1 ) // Signal " IN " WINCHING THIS IS THE " AUTOMATED " SIGNAL
               {
               // TRAVELLER IN
               if(digitalRead (mainSheet_IN_Stop) == LOW ) //IF MAIN IN
                 {
                 if(boomSide == 2) // STARBOARD
                   {
                   if(digitalRead (travellerStopLeft) == HIGH ) // HIGH = NOT MADE see Hall Sensor MP101401
                     {
                      digitalWrite( travellerDriveLeft , HIGH );
                      digitalWrite( travellerDriveRight , LOW );//OFF
                      digitalWrite( winch_IN_Mainsheet_Pin ,LOW );//OFF
                      digitalWrite( winch_OUT_Mainsheet_pin ,LOW );//OFF
                      autoWinchInStarted  = 0; // AUTO WINCH IN HAS STARTED
                     }
                   }
                 else if(boomSide == 1) // PORT
                   {
                    if(digitalRead (travellerStopRight) == HIGH )  // HIGH = NOT MADE see Hall Sensor MP101401
                      {
                       digitalWrite( travellerDriveRight , HIGH );
                       digitalWrite( travellerDriveLeft , LOW );//OFF
                       digitalWrite( winch_IN_Mainsheet_Pin ,LOW );//OFF
                       digitalWrite( winch_OUT_Mainsheet_pin ,LOW );//OFF
                       autoWinchInStarted  = 0; // AUTO WINCH IN HAS STARTED
                      }
                   }
                 }
                 //**********************************
                 
              // MAIN IN   
              if( digitalRead (mainSheet_IN_Stop )== HIGH ) // HIGH = NOT MADE  Push Button Switches on Coach Roof RS No 4994678 ( between brown & black )
                {                              //                                           *******            ***    *********          *********           *************
                 digitalWrite( winch_IN_Mainsheet_Pin , HIGH );
                 digitalWrite( travellerDriveRight , LOW );//OFF
                 digitalWrite( travellerDriveLeft , LOW );//OFF
                 digitalWrite( winch_OUT_Mainsheet_pin ,LOW );//OFF
                 autoWinchInStarted  = 0; // AUTO WINCH IN HAS STARTED
                }
               }
               //*******************************************
               
          // TURNS OFF ABOVE SECTION
          // ***********************
          if(winch_IN_Signal == 0) // TURN OFF " IN " WINCHING, CONTROLLED BY AUTO WINCHING SIGNAL SECTIONS ABOVE
            {
            if(autoWinchInStarted  == 0)// IF AUTO WINCH IN IS INITIATED FROM THE ABOVE AUTO WINCH IN OUTPUT SECTION
              {
               autoWinchInStarted  = 1;// STOPS THIS SECTION . STOPS THIS SECTION FROM CONTINUALLY SHUTTING DOWN THE TRAVELLER AND THE MAINSHEET IN .
               digitalWrite( travellerDriveLeft , LOW );
               digitalWrite( travellerDriveRight , LOW );
               digitalWrite(  winch_IN_Mainsheet_Pin , LOW );
               digitalWrite(  winch_OUT_Mainsheet_pin , LOW );
              }
            }
             // BELT & BRACES TURN OFF SECTION
             //*********************************
            if( digitalRead (mainSheet_IN_Stop )== LOW ) // IF MAIN SHEET IN
               {
                digitalWrite(  winch_IN_Mainsheet_Pin , LOW ); // TURN OFF MAIN IN
               }
              //******************************************************************* 
               
                 
                  if( digitalRead (travellerStopLeft ) == LOW ) // AND IF TRAVELLER IS AT LEFT STOP
                    {
                      digitalWrite(travellerDriveLeft , LOW); // TURN OFF DRIVE LEFT 
                      
                     }
                  
                //*****************************************************************  
                
                    if( digitalRead (travellerStopRight ) == LOW ) // AND IF TRAVELLER IS AT RIGHT STOP
                      {
                        digitalWrite(travellerDriveRight , LOW); // TURN OFF DRIVE RIGHT 
                        
                      }
                  
               // *****************************************************************
        } // END OF "  if( manualWinch_IN_Lock == 0 &&  manualWinch_OUT_Lock == 0   ) "
      } // END OF "  if(tackInProgressLock == 0 && gybeInProgressLock == 0) "
    }  //  END OF "  if (digitalRead( auto_winch_isolate)==HIGH) "
    // ******************************************************************************************************************END OF WINCH IN SECTION



          
      // ******************** END OF  SECTIONS CONTROL  OUTPUTS TO MAINSHEET WINCH AND TRAVELLER , THAT ARE AUTOMATED, BUT NOT CONTROLLED BY THE GYBE OR TACK SECTIONS
      //                     ********                                                                      ************







      
      //              ******************************************************
      
      //                             MANUAL WINCH SECTIONS
      
      //              ******************************************************
      
             // DEBOUNCE UNLOCK SECTION
             // ***********************
                if( manualWinch_debounce_toggle == 1)
                  {
                    Serial.println (" IN DEBOUNCE SECTION , manualWinch_debounce_toggle == 1 .");
                    if( millis() >=  manualWinch_input_debounceTime + 50 )  // ADJUST TIME AS NECESSARY .
                      {
                        manualWinch_debounce_toggle = 0 ; // RELEASE THE DEBOUNCE LOCK FROM BELOW
                        Serial.println (" UNLOCKED , manualWinch_debounce_toggle = 0 .");
                      }
                  }
             // **********************************

             

             
            //MANUAL WINCH OUTPUTS  WITH " AUTO SHEETING SATISFIED "( WITH auto_winch_isolate HIGH / ON ) SECTONS
            // ********************************************************************************************** 

             
               // MANUAL WINCH " IN " TIMER AND REALLOCATION SECTION (IMPORTANT: THIS SECTION ALLOWS (IF NECESSARY ! ) TO CHANGE FROM TRAVELLER TO MAINSHEET OR VICE VERSA .)
               //              *******
               //    FOR FOLLOWING IN SECTIONS .. THIS TIMER IS OPERATED BY BOTH MANUAL IN SECTIONS THAT FOLLOW
               //       ********  **** ******                   *********        ********************************
               // DRIVE IN FOR 3000mS 
               // **********************
                 if (  manualWinch_IN_Lock == 1 ) // IF WINCH IN LOCK IS ON
                    if( millis() >=    manualWinch_IN_Timer + 3000 )// IF TIME UP ( ADJUST TIME HERE AS REQUIRED )
                      {
                         manualWinch_IN_Lock = 0;
                         digitalWrite(travellerDriveLeft , LOW);// SHUT EVERYTHING DOWN
                         digitalWrite(travellerDriveRight ,LOW);// SHUT EVERYTHING DOWN
                         digitalWrite( winch_IN_Mainsheet_Pin , LOW);// SHUT EVERYTHING DOWN
                         digitalWrite( winch_OUT_Mainsheet_pin , LOW);// SHUT EVERYTHING DOWN  
                      }
                     //*********************
                     
                    else if ( millis() <    manualWinch_IN_Timer + 3000 )// IF TIME NOT UP
                            {
                              // TRAVELLER IN
                            if(digitalRead (mainSheet_IN_Stop) == LOW ) //IF MAIN IN ( AFT Push Button Switch on Coach Roof , RS No. 4994678
                              {
                              if(boomSide == 2) // STARBOARD . Note: THIS IS BOOM ON STARBOARD , NOT STARBOARD TACK
                                {               //                                              ********************
                                if(digitalRead (travellerStopLeft) == HIGH )  // HIGH = NOT MADE: IF Traveller IS NOT AT LEFT STOP :see Hall Sensor MP101401 WITHIN SCOPE OF TRAVELLER
                                  {
                                    digitalWrite( travellerDriveLeft , HIGH );//IMPORTANT: TRANSFER TO TRAVELLER ; OR KEEP GOING WITH TRAVELLER
                                    digitalWrite( travellerDriveRight ,LOW ); // OFF
                                    digitalWrite( winch_IN_Mainsheet_Pin ,LOW );// OFF; 
                                    digitalWrite( winch_OUT_Mainsheet_pin ,LOW );// OFF; 
                                        
                                    if(digitalRead(travellerStopCenter) == LOW) // LOW = MADE : IF TRAVELLER IS  AT CENTER
                                       {                                        //                  ************    ***    ******
                                          // DO SOMETHING ?
                                       }
                                  }
                                } 
                             else if(boomSide == 1) // PORT
                               {
                               if(digitalRead (travellerStopRight) == HIGH )  // HIGH = NOT MADE see Hall Sensor MP101401 WITHIN SCOPE OF TRAVELLER
                                 {
                                  digitalWrite( travellerDriveRight , HIGH ); //IMPORTANT: TRANSFER TO TRAVELLER ; OR KEEP GOING WITH TRAVELLER
                                  digitalWrite( travellerDriveLeft , LOW ); // OFF
                                  digitalWrite( winch_IN_Mainsheet_Pin , LOW );// OFF
                                  digitalWrite( winch_OUT_Mainsheet_pin ,LOW );// OFF; 
                                  
                                   if(digitalRead(travellerStopCenter) == LOW) // LOW =  MADE : IF TRAVELLER IS  AT CENTER
                                       {                                        //                  ************    ***    ******
                                          // DO SOMETHING MAYBE ?
                                       }
                                 }
                                }
                              }   
                           //*****************************
                           
                              // MAIN IN
                               if( digitalRead (mainSheet_IN_Stop )== HIGH ) // HIGH = NOT MADE
                                  {
                                    digitalWrite( winch_IN_Mainsheet_Pin , HIGH ); // KEEP GOING
                                    digitalWrite( travellerDriveRight ,LOW ); // OFF
                                    digitalWrite( travellerDriveLeft , LOW ); // OFF
                                    digitalWrite( winch_OUT_Mainsheet_pin , LOW );//OFF
                                  }
                              
                            }
                      // ****************************************************************END OF MANUAL WINCH IN TIMER AND REALLOCATION SECTION  
                      


                      //******************************************************************************************************************    
                      // MANUAL WINCH " OUT " TIMER AND REALLOCATION SECTION(IMPORTANT: THIS SECTION ALLOWS (IF NECESSARY ! ) TO CHANGE FROM TRAVELLER TO MAINSHEET OR VICE VERSA .)
                      // ********************************************** THIS SECTION IS OPERATED BY BOTH THE MANUAL WINCH OUT OUTPUT SECTIONS
                      //                                                                **********          **************           ********       
                                if( manualWinch_OUT_Lock == 1 )// IF MANUAL WINCH  "OUT" LOCK IS " ON "
                                  {
                                    if (millis() >=  manualWinch_OUT_Timer + 3000 ) // IF TIME IS UP ( adjust time here as necessary )
                                       {
                                        manualWinch_OUT_Lock = 0 ; // UNLOCKS .
                                        digitalWrite(  winch_OUT_Mainsheet_pin , LOW );// SHUT EVERYTHING DOWN
                                        digitalWrite(travellerDriveLeft , LOW);// SHUT EVERYTHING DOWN
                                        digitalWrite(travellerDriveRight ,LOW);// SHUT EVERYTHING DOWN
                                        digitalWrite(  winch_IN_Mainsheet_Pin , LOW );// SHUT EVERYTHING DOWN
                                       }
                                    else if (millis() <  manualWinch_OUT_Timer + 3000 ) // IF TIME IS " NOT " UP ( adjust time here as necessary )
                                       {
                                           // TRAVELLER OUT
                                           if(boomSide == 2) // STARBOARD
                                                {
                                                  if(digitalRead( travellerStopRight) == HIGH) // IF TRAVELLER STOP RIGHT  NOT  MADE     HIGH = OPEN ;  SEE HALL SWITCH MP101401 
                                                  {                               //        ********    **    *********
                                                   digitalWrite( travellerDriveRight , HIGH );// KEEP GOING WITH TRAVELLER OUT
                                                   digitalWrite( travellerDriveLeft , LOW );//OFF
                                                   digitalWrite(  winch_OUT_Mainsheet_pin , LOW );//OFF
                                                   digitalWrite(  winch_IN_Mainsheet_Pin , LOW );//OFF
                                                       if(digitalRead(travellerStopCenter) == LOW) // LOW =  MADE : IF TRAVELLER IS  AT CENTER
                                                         {                                        //                  ************    ***    ******
                                                             // DO SOMETHING MAYBE ?
                                                         }
                                                  }
                                                } 
                                              else if(boomSide == 1) // PORT
                                                {
                                                  if(digitalRead (travellerStopLeft) == HIGH) // IF TRAVELLER STOP LEFT  NOT  MADE        HIGH = OPEN ;  SEE HALL SWITCH MP101401 
                                                    {                           //     ***********    **   **********
                                                     digitalWrite( travellerDriveLeft , HIGH ); // KEEP GOING WITH TRAVELLER OUT
                                                     digitalWrite( travellerDriveRight , LOW );//OFF
                                                     digitalWrite(  winch_OUT_Mainsheet_pin , LOW );//OFF
                                                     digitalWrite(  winch_IN_Mainsheet_Pin , LOW );//OFF
                                                          if(digitalRead(travellerStopCenter) == LOW) // LOW =  MADE : IF TRAVELLER IS  AT CENTER
                                                            {                                        //                  ************    ***    ******
                                                             // DO SOMETHING MAYBE ?
                                                            }
                                                    }
                                                }
                                                
                                         
                                              // MAIN OUT
                                               if(boomSide == 1)//IF BOOM ON PORT
                                                 {
                                                 if( digitalRead ( travellerStopLeft ) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP LEFT IS MADE
                                                   {
                                                    if(mainSheet_OUT_Stop == HIGH ) // IF MAIN IS NOT AT OUT STOP
                                                      {
                                                       digitalWrite( winch_OUT_Mainsheet_pin , HIGH );//( winch_OUT_Mainsheet , HIGH );
                                                       digitalWrite(travellerDriveLeft , LOW );// off
                                                       digitalWrite(travellerDriveRight , LOW );// off 
                                                       digitalWrite (winch_IN_Mainsheet_Pin , LOW);//off
                                                    // autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                                                      }
                                                   }
                                                  }
                                               else if (boomSide == 2 )//IF BOOM ON STARBOARD
                                                  {
                                                  if(digitalRead( travellerStopRight) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP RIGHT IS MADE
                                                    {
                                                     if(mainSheet_OUT_Stop == HIGH ) // IF MAIN IS NOT AT OUT STOP
                                                      {
                                                        digitalWrite( winch_OUT_Mainsheet_pin , HIGH );//( winch_OUT_Mainsheet , HIGH );
                                                        digitalWrite(travellerDriveLeft , LOW );// off
                                                        digitalWrite(travellerDriveRight , LOW );// off 
                                                        digitalWrite (winch_IN_Mainsheet_Pin , LOW);//off
                                                     // autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                                                      }
                                                    }
                                                  }
                                           
                                     }// END OF  else if (millis() <=  manualWinch_OUT_Timer + 3000 )
                                  }// END OF "  if( manualWinch_OUT_Lock == 1 ) "
                            // END OF "  MANUAL WINCH OUT TIMER AND REALLOCATION SECTION " **************************************************************************
                            
                            

              //****************************************************
              // MANUAL WINCH IN , WITH AUTO SHEETING SATISFIED
              //**************************************************** 
             if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  " 
                {      
                if( tackInProgressLock == 0  ) // auto tack not engaged ALL AUTO SHEETING SYSTEMS ARE SATISFIED 
                  {
                  if( gybeInProgressLock == 0 ) // auto gybe not engaged  ALL AUTO SHEETING SYSTEMS ARE SATISFIED 
                    {
                      if (  winchDelayResetFour == 3 ||  winchDelayResetFour == 0 && winchDelayResetThree == 3 ||  winchDelayResetThree == 0  )// ALL AUTO SHEETING SYSTEMS ARE SATISFIED 
                         {
                          if (  winchDelayResetTwo == 3 ||  winchDelayResetTwo == 0 &&  winchDelayResetOne == 3 ||  winchDelayResetOne == 0) //   ALL AUTO SHEETING SYSTEMS ARE SATISFIED 
                             {
                               if( manualWinch_debounce_toggle = 0 )
                                  {
                                    //***************************
                                 // MANUAL IN  ( WITH AUTO SHEETING SATISFIED ( WITH auto_winch_isolate HIGH / ON )
                                 //************ 
                                    if( manualWinch_IN_Lock == 0) // IF MANUAL WINCH IN LOCK NOT ACTIVATED
                                      {
                                         if(manualWinch_OUT_Lock == 0) 
                                           {
                                           if(digitalRead( manualWinch_IN_input ) == HIGH )
                                             {
                                              manualWinch_debounce_toggle = 1; // START ABOVE DEBOUNCE SECTION
                                              manualWinch_input_debounceTime = millis(); // SET TIMER
                                              
                                             // TRAVELLER IN
                                           //*******************************************************************
                                             if(digitalRead ( mainSheet_IN_Stop )== LOW   ) //IF MAIN IN : WITHIN THE SCOPE OF THE TRAVELLER  MAIN SHEET IS IN
                                               {
                                               if(boomSide == 2) // STARBOARD
                                                 {
                                                  if(digitalRead(travellerStopLeft) == HIGH ) //IF NOT MADE
                                                     {
                                                      digitalWrite( travellerDriveLeft , HIGH );
                                                      digitalWrite( travellerDriveRight , LOW );//OFF
                                                      digitalWrite ( winch_IN_Mainsheet_Pin , LOW );// OFF
                                                      digitalWrite ( winch_OUT_Mainsheet_pin , LOW );//OFF
                                                      manualWinch_IN_Timer = millis(); //START TIMER
                                                      manualWinch_IN_Lock = 1;        // START LOCK Switches To ABOVE SECTIONS .
                                                     if(digitalRead(travellerStopCenter) == LOW) // IF BOOM  IN CENTER
                                                       {
                                                         // DO SOMETHING MAYBE ?
                                                       }
                                                     }
                                                  }
                                              //****************************************************************
                                              else if(boomSide == 1) // PORT
                                                 {
                                                   if(digitalRead(travellerStopRight) == HIGH ) //IF NOT MADE
                                                    {
                                                      digitalWrite( travellerDriveRight , HIGH );
                                                      digitalWrite( travellerDriveLeft , LOW );//OFF
                                                      digitalWrite ( winch_IN_Mainsheet_Pin , LOW );//OFF
                                                      digitalWrite ( winch_OUT_Mainsheet_pin , LOW );//OFF
                                                      manualWinch_IN_Timer = millis(); //START TIMER
                                                      manualWinch_IN_Lock = 1;        // START LOCK
                                                       if(digitalRead(travellerStopCenter) == LOW) // IF BOOM  IN CENTER
                                                        {
                                                          // DO SOMETHING , MAYBE ?
                                                        }
                                                    }
                                                  }
                                               } // END IF mainSheet_IN_Stop MADE .
                                              //******************************************************************* 
                                             // MAIN IN
                                             if( digitalRead (mainSheet_IN_Stop )== HIGH ) // HIGH = NOT MADE
                                               {
                                                digitalWrite ( winch_IN_Mainsheet_Pin , HIGH );
                                                digitalWrite( travellerDriveRight , LOW );//OFF
                                                digitalWrite( travellerDriveLeft , LOW );//OFF
                                                digitalWrite ( winch_OUT_Mainsheet_pin , LOW );//OFF
                                                manualWinch_IN_Timer = millis(); //START TIMER
                                                manualWinch_IN_Lock = 1;        // START LOCK
                                               }
                                            //*********************************************************************
                                           }
                                         }
                                      } // END OF " if( manualWinch_IN_Lock == 0) " .
                                      
                           // ******************************END OF MANUAL SHEET IN WITH AUTO SHEET SATISFIED .
                           //                                      *****       ****     ******************** 

                           //************************************************************************
                           // MANUAL OUT ( WITH AUTO SHEETING SATISFIED ( WITH auto_winch_isolate HIGH / ON )
                           //************************************************************************** 
                           if( manualWinch_IN_Lock == 0) // IF MANUAL WINCH IN LOCK NOT ACTIVATED
                              {
                                if(manualWinch_OUT_Lock == 0) // IF MANUAL WINCH OUT LOCK NOT ACTIVATED
                                  {
                                if(digitalRead (  manualWinch_OUT_input ) == HIGH )
                                   {
                                     manualWinch_debounce_toggle = 1; // START ABOVE DEBOUNCE SECTION
                                     manualWinch_input_debounceTime = millis(); // SET TIMER

                                    //  TRAVELLER OUT
                                    //           *****
                                   //******************************************************************************** 
                                    if(boomSide == 2) // STARBOARD
                                      {
                                       if(digitalRead (travellerStopRight) == HIGH ) // if traveller stop right is NOT MADE ;   HIGH = OPEN ;  SEE HALL SWITCH MP101401
                                         {
                                          digitalWrite( travellerDriveRight , HIGH );
                                          digitalWrite( travellerDriveLeft , LOW);//OFF
                                          digitalWrite ( winch_OUT_Mainsheet_pin ,LOW );//OFF
                                          digitalWrite ( winch_IN_Mainsheet_Pin ,LOW );//OFF
                                          manualWinch_OUT_Timer = millis(); //START TIMER
                                          manualWinch_OUT_Lock = 1;        // START LOCK
                                         }
                                       }
                                    //*****************************************************************************
                                     else if(boomSide == 1) // PORT
                                       {
                                       if(digitalRead (travellerStopLeft) == HIGH ) // if traveller stop left is NOT MADE ;   LOW = OPEN ;  SEE HALL SWITCH MP101401
                                         {
                                          digitalWrite( travellerDriveLeft , HIGH );
                                          digitalWrite( travellerDriveRight , LOW );//OFF
                                          digitalWrite ( winch_OUT_Mainsheet_pin , LOW );//OFF
                                          digitalWrite ( winch_IN_Mainsheet_Pin ,LOW );//OFF
                                          manualWinch_OUT_Timer = millis(); //START TIMER
                                          manualWinch_OUT_Lock = 1;        // START LOCK
                                         }
                                        }
                                    //******************************************************************************  
                                      
                                       // MAIN OUT
                                       //***************************************************************************
                                               if(boomSide == 1)//IF BOOM ON PORT
                                                 {
                                                 if( digitalRead ( travellerStopLeft ) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP LEFT IS MADE
                                                   {
                                                    digitalWrite( winch_OUT_Mainsheet_pin , HIGH );//( winch_OUT_Mainsheet , HIGH );
                                                    digitalWrite(travellerDriveLeft , LOW );// off
                                                    digitalWrite(travellerDriveRight , LOW );// off 
                                                    digitalWrite (winch_IN_Mainsheet_Pin , LOW);//off
                                                    manualWinch_OUT_Timer = millis(); //START TIMER
                                                    manualWinch_OUT_Lock = 1;        // START LOCK
                                                   // autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                                                   }
                                                  }
                                               else if (boomSide == 2 )//IF BOOM ON STARBOARD
                                                  {
                                                  if(digitalRead( travellerStopRight) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP RIGHT IS MADE
                                                    {
                                                     digitalWrite( winch_OUT_Mainsheet_pin , HIGH );//( winch_OUT_Mainsheet , HIGH );
                                                     digitalWrite(travellerDriveLeft , LOW );// off
                                                     digitalWrite(travellerDriveRight , LOW );// off 
                                                     digitalWrite (winch_IN_Mainsheet_Pin , LOW);//off
                                                     manualWinch_OUT_Timer = millis(); //START TIMER
                                                     manualWinch_OUT_Lock = 1;        // START LOCK
                                                   //  autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                                                    }
                                                  } 
                                        //*****************************************************************************
                                     }
                                   }
                                 }// END OF "  if( manualWinch_IN_Lock == 0)  "
                                 //****************************************************************************************************
                                 // END OF MANUAL SHEET OUT WITH AUTO SHEETING SATISFIED

                                 
                             //******************************************************************************************************************    
                           
                              
                          }// end manualWinch_debounce_toggle == 0 ;
                        }
                      }
                    }
                  }// END OF IF TACK IN PROGRESS LOCK
                }  // END OF  if (digitalRead( auto_winch_isolate)==HIGH)// IF AUTO WINCH ISOLATE SWITCH " ON  " 
                   
                   //END OF MANUAL WINCH OUTPUTS ," WITH AUTO SATISFIED " SECTIONS
                  // *****************************************************************************************************************************************************************

                  

                  
                  //**************************************************************************
                  // MANUAL WINCH OUTPUTS IF " AUTO SHEETING OFF " , UNLOCK GYBE AND TACK LOCKS
                  //**************************************************************************
                  
                  if(digitalRead(auto_winch_isolate) == LOW ) // IF AUTO OFF
                  {
                    gybeInProgressLock = 0; //RESET TO UNLOCKED
                    tackInProgressLock = 0; //RESET TO UNLOCKED
                  }
                  //*********************************************

                  
                  
          // // MANUAL WINCH SECTION IF AUTO WINCH TURNED " OFF " ( WITH auto_winch_isolate LOW / OFF )
          //   ***************************************************
                   if (digitalRead (auto_winch_isolate) == LOW)// MANUAL WINCH SECTION IF AUTO WINCH TURNED " OFF "
                    {
                      Serial.println(" A, auto_winch_isolate = LOW . ");
                      
                      Serial.println( " manualWinch_debounce_toggle = ");
                      Serial.print( manualWinch_debounce_toggle );
                      if(digitalRead(manualWinch_IN_input)  == HIGH) // FOR DEBUGING ONLY
                        {
                          Serial.println(" manualWinch_IN_input  == HIGH  ");
                        }
                       if(digitalRead(manualWinch_OUT_input)  == HIGH) // FOR DEBUGING ONLY
                        {
                          Serial.println(" manualWinch_OUT_input  == HIGH  ");
                          
                        }
                         if( manualWinch_debounce_toggle == 0 )
                                  {
                                    Serial.println (" B, manualWinch_debounce_toggle = 0 . ");
                                    //***************************
                                 // MANUAL IN  ( WITH AUTO SHEETING OFF ( WITH auto_winch_isolate LOW / OFF )
                                 //************ 
                                    if( manualWinch_IN_Lock == 0) // IF MANUAL WINCH IN LOCK NOT ACTIVATED
                                      {
                                        Serial.println ( " C, manualWinch_IN_Lock = 0 . ");
                                         if(manualWinch_OUT_Lock == 0) //IF MANUAL WINCH OUT LOCK NOT ACTIVATED
                                           {
                                            Serial.println ( " D , manualWinch_OUT_Lock = 0 .");
                                           if(digitalRead( manualWinch_IN_input ) == HIGH )
                                             {
                                              Serial.println ( " E , manualWinch_IN_input = HIGH  . " ) ;
                                              manualWinch_debounce_toggle = 1; // START ABOVE DEBOUNCE SECTION
                                              Serial.println ( " MANUAL IN  WITH AUTO SHEETING OFF SET ,  manualWinch_debounce_toggle = 1 ");
                                              manualWinch_input_debounceTime = millis(); // SET TIMER
                                              
                                             // TRAVELLER IN
                                             //**********************************************************************
                                             if( digitalRead( mainSheet_IN_Stop) == LOW ) //IF MAINSHEET IN . WITHIN THE SCOPE OF THE TRAVELLER
                                               {
                                                Serial.println (" F, mainsheet_IN_Stop = LOW . ");
                                               if(boomSide == 2) // STARBOARD
                                                 {
                                                  Serial.println ( " G, boomSide = 2 , Starboard  . ");
                                                  if(digitalRead (travellerStopLeft) == HIGH ) // TRAVELLER STOP LEFT NOT MADE
                                                    {
                                                      Serial.println ( " H, travellerStopLeft = HIGH , NOT MADE . " );
                                                       digitalWrite( travellerDriveLeft , HIGH ); // DRIVE TRAVELLER LEFT
                                                         digitalWrite( travellerDriveRight , LOW ); //OFF
                                                         digitalWrite ( winch_IN_Mainsheet_Pin ,LOW );//OFF 
                                                         digitalWrite ( winch_OUT_Mainsheet_pin ,LOW );//OFF 
                                                         manualWinch_IN_Timer = millis(); //START TIMER
                                                         manualWinch_IN_Lock = 1;        // START LOCK
                                                     if(digitalRead(travellerStopCenter) ==HIGH) // IF BOOM NOT AT CENTER
                                                        {
                                                          // DO SOMETHING ? HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE
                                                        }
                                                    }
                                                 }
                                               //**********************************************************************
                                              else if(boomSide == 1) // PORT
                                                 {
                                                  if(digitalRead (travellerStopRight) == HIGH ) //if traveller stop right NOT MADE
                                                    {
                                                      digitalWrite( travellerDriveRight , HIGH );
                                                         digitalWrite( travellerDriveLeft , LOW ); //OFF
                                                         digitalWrite ( winch_IN_Mainsheet_Pin ,LOW );//OFF 
                                                         digitalWrite ( winch_OUT_Mainsheet_pin ,LOW );//OFF
                                                         manualWinch_IN_Timer = millis(); //START TIMER
                                                         manualWinch_IN_Lock = 1;        // START LOCK
                                                      if(digitalRead(travellerStopCenter) ==LOW) // IF BOOM  AT CENTER
                                                        {
                                                           // DO SOMETHING MAYBE ?
                                                        }
                                                    }
                                                  }
                                                 //**********************************************************************
                                                }// END OF IF mainSheet_IN_Stop IS MADE .
                                                
                                             // MAIN IN
                                             //***************************************************************************
                                             if( digitalRead (mainSheet_IN_Stop )== HIGH ) // HIGH = NOT MADE
                                               {
                                                digitalWrite ( winch_IN_Mainsheet_Pin , HIGH );
                                                digitalWrite( travellerDriveRight , LOW ); //OFF
                                                digitalWrite( travellerDriveLeft , LOW ); //OFF
                                                digitalWrite ( winch_OUT_Mainsheet_pin ,LOW );//OFF
                                                manualWinch_IN_Timer = millis(); //START TIMER
                                                manualWinch_IN_Lock = 1;        // START LOCK
                                               }
                                             //****************************************************************************
                                           }
                                         }
                                      }
                                    // END OF MANUAL WINCH  IN  WITH AUTO OFF ***************************************************************


                                    
                           // ****************************************************************************
                           // MANUAL OUT ( WITH AUTO SHEETING " OFF " ( WITH auto_winch_isolate LOW / OFF )
                           //****************************************************************************** 
                           if( manualWinch_IN_Lock == 0) // IF MANUAL WINCH IN LOCK NOT ACTIVATED
                              {
                                if(manualWinch_OUT_Lock == 0)//IF MANUAL WINCH OUT LOCK NOT ACTIVATED
                                  {
                                if(digitalRead (  manualWinch_OUT_input ) == HIGH )
                                   {
                                     manualWinch_debounce_toggle = 1; // START ABOVE DEBOUNCE SECTION
                                      Serial.println ( " MANUAL OUT  WITH AUTO SHEETING OFF SET ,  manualWinch_debounce_toggle = 1 ");
                                     manualWinch_input_debounceTime = millis(); // SET TIMER

                                   // TRAVELLER OUT
                                   //***********************************************************************
                                   if(boomSide == 2) // STARBOARD
                                     {
                                     if(digitalRead (travellerStopRight) == HIGH ) // if traveller stop right is NOT MADE
                                       {
                                        digitalWrite( travellerDriveRight , HIGH );
                                        digitalWrite( travellerDriveLeft , LOW );//OFF
                                        digitalWrite ( winch_OUT_Mainsheet_pin , LOW );//OFF
                                        digitalWrite ( winch_IN_Mainsheet_Pin , LOW );//OFF
                                        manualWinch_OUT_Timer = millis(); //START TIMER
                                        manualWinch_OUT_Lock = 1;        // START LOCK
                                       }
                                      }
                                    //*************************************************************************
                                    else if(boomSide == 1) // PORT
                                      {
                                      if(digitalRead (travellerStopLeft) == HIGH ) // if traveller stop left is NOT MADE
                                        {
                                         digitalWrite( travellerDriveLeft , HIGH );
                                         digitalWrite( travellerDriveRight , LOW );//OFF
                                         digitalWrite ( winch_OUT_Mainsheet_pin , LOW );//OFF
                                         digitalWrite ( winch_IN_Mainsheet_Pin , LOW );//OFF
                                         manualWinch_OUT_Timer = millis(); //START TIMER
                                         manualWinch_OUT_Lock = 1;        // START LOCK
                                        }
                                       }
                                     //*************************************************************************  
                                       
                                       // MAIN OUT
                                       //***********************************************************************
                                       if(boomSide == 1)//IF BOOM ON PORT
                                          {
                                          if( digitalRead ( travellerStopLeft ) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP LEFT IS MADE
                                            {
                                             digitalWrite( winch_OUT_Mainsheet_pin , HIGH );//( winch_OUT_Mainsheet , HIGH );
                                             digitalWrite(travellerDriveLeft , LOW );// off
                                             digitalWrite(travellerDriveRight , LOW );// off 
                                             digitalWrite (winch_IN_Mainsheet_Pin , LOW);//off
                                             manualWinch_OUT_Timer = millis(); //START TIMER
                                             manualWinch_OUT_Lock = 1;        // START LOCK
                                            // autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                                            }
                                           }
                                           //*****************************************************
                                        else if (boomSide == 2 )//IF BOOM ON STARBOARD
                                           {
                                           if(digitalRead( travellerStopRight) == LOW) // LOW = MADE   SEE HALL SWITCH MP101401 .IF BOOM OUTSIDE OF TRAVELLER ;TRAVELLER STOP RIGHT IS MADE
                                             {
                                              digitalWrite( winch_OUT_Mainsheet_pin , HIGH );//( winch_OUT_Mainsheet , HIGH );
                                              digitalWrite(travellerDriveLeft , LOW );// off
                                              digitalWrite(travellerDriveRight , LOW );// off 
                                              digitalWrite (winch_IN_Mainsheet_Pin , LOW);//off
                                              manualWinch_OUT_Timer = millis(); //START TIMER
                                              manualWinch_OUT_Lock = 1;        // START LOCK
                                             // autoWinchOutStarted = 0 ; // AUTO WINCH OUT HAS STARTED
                                             }
                                           }
                                         //**********************************************************************
                                     }
                                   }// END OF   if(manualWinch_OUT_Lock == 0)
                                 }
                                 // END OF MANUAL WINCH OUT WITH AUTO SHEETING OFF**********************************************************************


                             //***********************************************    
                     
                                  
                          }// end manualWinch_debounce_toggle == 0 ;  
                                  
                     } // END OF "  if (digitalRead (auto_winch_isolate) == LOW) "
                  
                 //*****************************************************************************

       //     *************************************************************************END ALL MANUAL WINCH SECTIONS 9/2/19

       




       //     LED SECTIONS FOLLOW
      //   *************************************************      
            
         
            /* TRYING TO ASSURE THAT AT NO TIME DOES THE ORANGE & BLUE LIGHTS ILLUMINATE AT THE SAME TIME !!!!!
               ************************************************************************************************   */
            
            if ( winch_IN_Signal == 1 )
            {
             // winch_OUT_Signal = 0; // 0 = OUT Signal OFF , 1 = OUT Signal ON .// digitalWrite (winch_OUT_pin , LOW);
              digitalWrite (ledBlueOut , LOW);
            }
            
            if ( winch_OUT_Signal  == 1)
            {
             // winch_IN_Signal = 0; // change from winch_IN_Pin 6/2/19 // digitalWrite(winch_IN_pin , LOW);
              digitalWrite(ledOrangeIn , LOW);
            }
    

      
           
                
               
               
               /* THE FOLLOWING SECTIONS MAINTAIN  LED_STATUS    / IF WINCHES ARE OPERATED MANUALLY WITHOUT A CHANGE IN WIND DIRECTION RELATIVE TO HEADING ( EG WITHOUT CHANGE OF COURSE ) '
                  ****************************************************************************************************************************************************    */
                  
                  
              
                  
                  /* NEW  SECTION TO CHECK IF WINCHES HAVE BEEN ALTERED MANUALLY     20/8/15
                         **********************************************************   */
                         
                  if((winchInToggle == 0) && (winchOutToggle == 0))  // checks to see if AUTO winch is INACTIVE .IMPORTANT:  winchInToggle and winchOutToggle are only activate when winches achieve requiredboomangle AND when GREEN LIGHT is ON
                  {
                  if( actualBoomAngle < requiredBoomAngle-5) // IF requiredBoomAngle (changed from, autoSetBoomAngle) IS LESS THAN  autoSetBoomAngle - ( at moment ) 10  THIS MUST HAVE BEEN A MANUAL BOOM ALTERATION 
                      {
                        
                        manualWinchINtoggle = 1;
                        manualWinchOUTtoggle = 0;
                      }
                   }
                   
                   if((winchOutToggle == 0)&& (winchInToggle == 0))
                   {
                     
                     if( actualBoomAngle > requiredBoomAngle+5)
                       {
                         
                          manualWinchOUTtoggle = 1;
                          manualWinchINtoggle = 0;
                       }
                     
                    }
                    
                    
               
             
                                              
    
          
        /* LED  IN  SECTION  IF BOOM OPERATED MANUALLY    (EG NO CHANGE OF COURSE BUT CHANGE OF BOOM ANGLE )
               
          ********************************************      */
       
      if(currentWindA_Toggle == 1 )
        {
           if(manualWinchOUTtoggle == 1)   // TRYING THIS 20/8/15
             {
              ledBoomOperatedMANUALLY_Timer = millis();
             if(ledBoomOperatedMANUALLY_Timer >= ledManuallyOperated_count + 300) // TIMER   ADDED  = AFTER > TO INCLUDE THE + 5
               {
                if(digitalRead(ledWhiteManualIn) == LOW)
                  {
                    manualWinchOUTtoggle = 0;
                    digitalWrite(ledWhiteManualIn , HIGH );
                    digitalWrite (ledGreenOK , LOW );
                    ledManuallyOperated_count = ledBoomOperatedMANUALLY_Timer ;
                  }
                else
                {
                 
                  manualWinchINtoggle = 0;
                  digitalWrite(ledWhiteManualIn , LOW );
                  digitalWrite (ledGreenOK , LOW );
                  ledManuallyOperated_count = ledBoomOperatedMANUALLY_Timer ;
                }
               }
            }
      
         }
      
         
    
            
           /* LED   OUT  SECTION IF BOOM OPERATED MANUALLY (EG NO CHANGE OF COURSE BUT CHANGE OF BOOM ANGLE )
              ********************************************     */
  
        
     
            if(currentWindA_Toggle == 1 )
            {
             if(manualWinchINtoggle == 1)
               {
                ledBoomOperatedMANUALLY_Timer = millis();
                if(ledBoomOperatedMANUALLY_Timer >= ledManuallyOperated_count + 300)
                  {
                  if(digitalRead(ledWhiteManualOut) == LOW)
                    {
                     digitalWrite(ledWhiteManualOut , HIGH );
                     digitalWrite(ledWhiteManualIn , LOW );
                     digitalWrite (ledGreenOK , LOW );
                     ledManuallyOperated_count = ledBoomOperatedMANUALLY_Timer ;
                    }
                  else
                   {
                    digitalWrite(ledWhiteManualOut , LOW );
                    digitalWrite(ledWhiteManualIn , LOW );
                    digitalWrite (ledGreenOK , LOW );
                    ledManuallyOperated_count = ledBoomOperatedMANUALLY_Timer ;
                   }
                 }
              }
           }
         
         
        
     
               
               
            
      
     
        /* GREEN LED SECTIONS FOLLOW 
          **************************   */
          
          
              /* ACTUAL BOOMANGLE  IS EQUAL TO  REQUIRED BOOMANGLE SECTION
             **********************************************************   */
            
             if(actualBoomAngle == requiredBoomAngle )
            {
               if(currentWindA_Toggle == 1 )  //  THIS SHOULD ENCOMPASS THE ENTIRE WINCH SECTIONS .
                 {
                  digitalWrite(ledGreenOK , HIGH );
                  if(gybeInProgressLock == 0 && tackInProgressLock == 0 )
                  {
                   winch_IN_Signal = 0; // change from winch_IN_Pin 6/2/19   // COULD THIS SHUT DOWN A GYBE OR TACK ???? !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                  }
                  digitalWrite(ledOrangeIn ,  LOW );
                  if(gybeInProgressLock == 0 && tackInProgressLock == 0 )
                  {
                   winch_OUT_Signal = 0; // 0 = OUT Signal OFF , 1 = OUT Signal ON .// digitalWrite(winch_OUT_pin , LOW );// COULD THIS SHUT DOWN A GYBE OR TACK ???? !!!!!!!!!!!!!!!!!!!
                  }
                  digitalWrite(ledBlueOut , LOW);
                  digitalWrite(ledWhiteManualIn , LOW );
                  digitalWrite(ledWhiteManualOut , LOW );
                  toggleActivateWinchOUT = 0;
                  toggleActivateWinchIN = 0;
                  manualWinchINtoggle = 0;
                  manualWinchOUTtoggle = 0;
                  winchOutToggle = 0;
                  winchInToggle = 0;
                 }
             }
       
     
     
            
            
        
              
       /*  GREEEN FLASHING LED SECTION IF ACTUALBOOMANGLE GREATER THAN REQUIREDBOOMANGLE BUT  LESS THAN REQUIREDBOOMANGLE + 15
           *********************************************************************************************************************   */       
       if((actualBoomAngle > requiredBoomAngle) && (actualBoomAngle <= requiredBoomAngle +  10)) // changed value from +15 TO ( +5 CAN NEVER BE  " TRUE " as is measured in 5's ) TRYING <= NOT JUST <
         {
          if(currentWindA_Toggle == 1 )  //  THIS SHOULD ENCOMPASS THE ENTIRE WINCH SECTIONS .
            {
             ledTimer = millis();
            if(ledTimer - ledCount > 300 )
              {
              if(ledGreenSTATE == LOW)
                { 
                 digitalWrite (ledGreenOK , HIGH);
                 ledGreenSTATE = HIGH ;
                 digitalWrite(ledWhiteManualIn , LOW );
                 digitalWrite(ledWhiteManualOut , LOW );
                 manualWinchINtoggle = 0;
                 manualWinchOUTtoggle = 0;
                 ledCount = ledTimer;
               }
             else if(ledGreenSTATE == HIGH)
               {
                digitalWrite (ledGreenOK , LOW);
                ledGreenSTATE = LOW ;
                digitalWrite(ledWhiteManualIn , LOW );
                digitalWrite(ledWhiteManualOut , LOW );
                manualWinchINtoggle = 0;
                manualWinchOUTtoggle = 0;
                ledCount = ledTimer;
               }
             }
           }
         }
         if(actualBoomAngle > requiredBoomAngle +  10)
         {
            digitalWrite (ledGreenOK , LOW);
         }
         
         
         /* GREEN FLASHING LED SECTION IF ACTUALBOOMANGLE LESS THAN REQUIREDBOOMANGLE BUT GREATER THAN REQUIREDBOOMANGLE -15
           *****************************************************************************************************************  */
           
            if((actualBoomAngle < requiredBoomAngle) && (actualBoomAngle >= requiredBoomAngle - 10)) //   CHANGED VALUE FROM -15  TO ( -5 .CAN NEVER BE  " TRUE " as boomAngle is measured in 5's )
              {
              if(currentWindA_Toggle == 1 )  //  THIS SHOULD ENCOMPASS THE ENTIRE WINCH SECTIONS .
                {
                 ledTimer = millis();
                 if(ledTimer - ledCount > 300 )
                   {
                   if(ledGreenSTATE == LOW)
                     { 
                      digitalWrite (ledGreenOK , HIGH);
                      ledGreenSTATE = HIGH ;
                      digitalWrite(ledWhiteManualIn , LOW );
                      digitalWrite(ledWhiteManualOut , LOW );
                      manualWinchINtoggle = 0;
                      manualWinchOUTtoggle = 0;
                      ledCount = ledTimer;
                     }
                   else if(ledGreenSTATE == HIGH)
                     {
                      digitalWrite (ledGreenOK , LOW);
                      ledGreenSTATE = LOW ;
                      digitalWrite(ledWhiteManualIn , LOW );
                      digitalWrite(ledWhiteManualOut , LOW );
                      manualWinchINtoggle = 0;
                      manualWinchOUTtoggle = 0;
                     ledCount = ledTimer;
                     }
                  }
                }
             }
       
       if(actualBoomAngle < requiredBoomAngle -  10)
         {
            digitalWrite (ledGreenOK , LOW);
         } 

 
 
  /*       Serial.print SECTION 
           *********************    */
   if(millis()>= count+3000)
  { 
   /* digitalRead( travellerStopRight );
    if ( travellerStopRight == LOW )
       {
        Serial.print( "travellerStopRight = LOW ");
       }
     else if( travellerStopRight == HIGH)
            {
              Serial.print(" travellerStopRight == High ");
            }  */
            
  /*  if(digitalRead(winch_IN_pin == LOW ))
    {
     Serial.print (" winch IN pin is LOW  ? ");
    }  
    if(digitalRead(winch_IN_pin == HIGH))
    {
      Serial.print(" winch_IN_pin == HIGH  ");
    }
    if(digitalRead(winch_OUT_pin == HIGH))
    {
      Serial.print(" winch_OUT_pin == HIGH  ");
    }
    if(digitalRead(winch_OUT_pin == LOW))
    {
      Serial.print(" winch_OUT_pin == LOW  ");
    }                                          */ // IMPORTANT NOTE TO SELF digitalRead DOES NOT WORK ON AN OUTPUT PIN ,SHOULD USE STATE
    
   //  if(  toggleActivateWinchIN == 1 )
    // {
    //   Serial.print("  toggleActivateWinchIN == 1  ");
   //  }
   //  if(  toggleActivateWinchIN == 0 )
   //  {
    //   Serial.print("  toggleActivateWinchIN == 0  ");
   //  }
   //  if(  toggleActivateWinchOUT == 1 )
   //  {
   //    Serial.print("  toggleActivateWinchOUT == 1  ");
   //  }
   //  if(  toggleActivateWinchOUT == 0 )
   //  {
   //    Serial.print("  toggleActivateWinchOUT == 0  ");
   //  }
     
 // Serial.print("   testVariable =");
  //Serial.print(testVariable);  
 // Serial.print(".. winchInToggle= ");
//  Serial.print(winchInToggle);  
//  Serial.print("Differential 0_1: "); Serial.print(results0_1);// Serial.print("(");Serial.print("....results0_1tempCorrect = ");Serial.print(results0_1tempCorrect);Serial.print("...... ");
  //Serial.print(results0_1 * multiplier); Serial.println("mV)");
//  Serial.print("Differential 2_3: "); Serial.print(results2_3);// Serial.print("(");Serial.print("...results2_3tempCorrect =  ");Serial.print(results2_3tempCorrect);Serial.print(".........");
 // Serial.print(results2_3 * multiplier); Serial.println("mV)");
//  Serial.print("TP36 channel 0 : "); Serial.println(TP0); Serial.print("("); Serial.print(TP0 * multiplier); Serial.println("mV)");
//  Serial.print("Temperature = "); Serial.print(temp);
 // Serial.print("..... int temp =");Serial.print(intTemp);
  //Serial.print("   currentWindA_Toggle =");
  //Serial.print(currentWindA_Toggle );
 // Serial.print("~ ~ ~ ~ ~ ");
 // Serial.print("  currentWindA = ");
 // Serial.print(currentWindA);
  //Serial.print("..winchDelayReset =");
  //Serial.print(winchDelayReset);
 // Serial.print("-------- .");
 //Serial.print("  gust_ProgramWinchInterrupt =");Serial.println(gust_ProgramWinchInterrupt);
   
 // Serial.print("  windAngleInt  =  "); //
 // Serial.print(windAngleInt);          //
  //Serial.print(" ;;;;;;;;;  ");        //
  //Serial.print(F("....windA ")); Serial.print(windA.value() );
 // Serial.print(F("...R or T =")); Serial.print(RorT.value() );
  //Serial.print( " required Boom Angle is ..");//
  //Serial.print(requiredBoomAngle);            //
 // Serial.print ("_ _ _ . ");
 // Serial.print  (" actualBoomAngle =...  ");
 // Serial.print (actualBoomAngle);
 // Serial.print ("....");
 // Serial.print ("boomside = ...");
//  Serial.print (boomSide);
 // Serial.print ("....");
  //Serial.println ("toggleActivateWinchIN =");
 // Serial.print (toggleActivateWinchIN);
 // Serial.print("toggleActivateWinchOUT =");
 // Serial.print (toggleActivateWinchOUT);
 // Serial.println ("winchDelayReset =");
 // Serial.print  (winchDelayReset);
 // Serial.println("..........");
  //Serial.print("failResults0_1 =  ");
 // Serial.print(failResults0_1);
 // Serial.print("....");
 // Serial.print("failResults2_3 = ");
 // Serial.print(failResults2_3);
 // Serial.print(",,,,,,");
 // Serial.print("boomAngleHasBeenAutoSet =");
 // Serial.print(boomHasBeenAutoSet);
 // Serial.print("...... _ ");
 // Serial.print("manualWinchINtoggle =");
//  Serial.print(manualWinchINtoggle);
 // Serial.print(". . . / ");
 // Serial.print("manualWinchOUTtoggle =");
 // Serial.print(manualWinchOUTtoggle);
 // Serial.print("-------- .");
  //Serial.print("_ _ HEADING IS = ");
 // Serial.print(headingInt);
  //Serial.print("_._._._._. ");
  //Serial.print(headingString);
   
              
                
              
  count = millis();
  
   //   TEMPORARILLY MOVED THE CURRLY END BRACKET TO BOTTOM TO ENSURE Serial.print IS CO_ORDINATED
  }
  
 
   
} // LOOP END
   /* INTERRUPT SECTIONS 
      ******************    */  

      
    
/*  void gustOn ()  // ADDED 11/1/16
  {
   
   gust_ProgramWinchInterrupt = 1;

/*   NOTE : THE BELOW SECTION DIDN'T WORK THE WHILE STATEMENT FROZE THE PROGRAM AS IT COULD NOT UPDATE , SO MOVED THIS SECTION TO MAIN PROG JUST AT START OF WINCH SECTIONS
    digitalWrite(winch_IN_pin , LOW );  // ENSURE EVERTHING IS   OFF
  // DON'T WNAT TO ALTER THE MAIN PROGRAMS STATE , OTHER THAN TO ENSURE WINCH PINS ARE  OFF , SO WILL NOT INCLUDE THESE HERE // digitalWrite(ledOrangeIn ,  LOW );  // ENSURE EVERTHING IS   OFF
    digitalWrite(winch_OUT_pin,LOW);   // ENSURE EVERTHING IS   OFF
  // DO NOT INCLUDE HERE SEE ABOVE //  digitalWrite(ledBlueOut , LOW);    // ENSURE EVERTHING IS   OFF
  // DO NOT INCLUDE HERE SEE ABOVE ???? // winchInToggle = 0;                 // ENSURE EVERTHING IS   OFF
  // DO NOT INCLUDE HERE SEE ABOVE ????  //toggleActivateWinchIN = 0;         // ENSURE EVERTHING IS   OFF
    // THINK WE MIGHT NEED A    WHILST   RATHER THAN AN  IF   TO TRY AND ENSURE THE INTERRUPT CONTINUES TO SHEET OUT TO THE DESIRED SETTING AND DOESNT JUST SWITCH WINCH OUT ON AND THEN RETURN TO MAIN PRGRAM
    // WILL NEED TO SPECIFY IF BELOW OR ABOVE 180
    if(requiredBoomAngle <= 35 )// TRYING TO ENSURE THE WINCH CAN ACHIEVE  " requiredBoomAngle + 10 " . IF THE " requiredBoomAngle " WERE GREATER THAN " 40 " IT COULD NOT ACHIEVE  IT AS IT'S MAXIMUM OUT ON MItoo IS 50 . 
    {
    while(actualBoomAngle < requiredBoomAngle + 10)  // HOPEFULLY THIS WILL HOLD THE PROGRAM HERE UNTIL THIS BECOMES FALSE ?????????????????????????????????
    {
      digitalWrite ( winch_OUT_pin , HIGH );
      digitalWrite ( gustOnLed , HIGH );  //  PIN 5 ,
    }
    }
    if (actualBoomAngle >= requiredBoomAngle + 10 )
      {
        digitalWrite ( winch_OUT_pin , LOW );
      }      */
      
      
     /*            digitalWrite(winch_IN_pin , LOW );  // ENSURE EVERTHING IS   OFF
              // DON'T WNAT TO ALTER THE MAIN PROGRAMS STATE , OTHER THAN TO ENSURE WINCH PINS ARE  OFF , SO WILL NOT INCLUDE THESE HERE // digitalWrite(ledOrangeIn ,  LOW );  // ENSURE EVERTHING IS   OFF
                 digitalWrite(winch_OUT_pin,LOW);   // ENSURE EVERTHING IS   OFF
                 gustInterruptWinchReset = 0; // IF A SECOND gustOn INTERRUPT WAS TRIGGERED WITHIN THE ( 5 sec ) INTERVAL BETWEEN gustOn AND gustOff . NOTE: NOT REALLY REQUIRED gustInitialise TAKES CARE OF THIS ON " sketch_dec14a "
              if(requiredBoomAngle <= 35 )// TRYING TO ENSURE THE WINCH CAN ACHIEVE  " requiredBoomAngle + 10 " . IF THE " requiredBoomAngle " WERE GREATER THAN " 40 " IT COULD NOT ACHIEVE  IT AS IT'S MAXIMUM OUT ON MItoo IS 50 . 
                 {
                   if(actualBoomAngle < requiredBoomAngle + 10)  // MAY NEED TO ALTER THIS TO <= , NO . 
                       {
                          digitalWrite ( winch_OUT_pin , HIGH );
                          digitalWrite ( gustOnLed , HIGH );  //  PIN 5 ,
                          gustInterruptWinchReset = 1;
                          gustStopMainWinch = 1;
                        }
                  }
    
   } 
  

  void gustOff ()
  {

    gust_ProgramWinchInterrupt = 0; // may put a timer in main loop to ensure this returns to " 0 " after a period ( 20 sec ?? ) in the event " gustOff " has not activated think this is a safe option as if the gust has remained for 20 sec it should have then become the " average " anyway .
    //digitalWrite (gustOnLed , LOW );
  }

  /* The ADC input range (or gain) can be changed via the following
     functions, but be careful never to exceed VDD +0.3V max, or to
     exceed the upper and lower limits if you adjust the input range!
     Setting these values incorrectly may destroy your ADC!
                                                                    ADS1015  ADS1115
                                                                    -------  -------
     ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
     ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
     ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
     ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
     ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
     ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV   */
