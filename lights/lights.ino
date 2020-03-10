         
         
         // GRAPHICS SKETCH BLOWN AWAY 14/9/19
         
int startTransmissionRequest = 0 ;
long transmissionCount = 0 ;
long transmissionCount1 = 0;

int actualBoomAngle = -1 ;
int windAngleInt = -1;
int boomSide = -1;
const int port = 1;
const int starboard = 2;

int requiredBoomAngle = -10 ;

int airMarMin = 0 ; // minimum wind sensor value
int airMarMax = 360 ; // maximum wind sensor value

const int  ledBoomAngPort5 =  30;  // start of Actual boom angle leds . don't forget pinmodes 15/2/17 // wire Orange black stripe
const int  ledBoomAngPort15 = 46;  // changed from pin 31    22/8/17  // wire green no stripe
const int  ledBoomAngPort30 = 32;  // wire Green white stripe
const int  ledBoomAngPort38 = 33;  // wire Green black stripe
const int  ledBoomAngStar5 =  34;  // wire White
const int  ledBoomAngStar15 = 35;  // wire Orange
const int  ledBoomAngStar30 = 36;  // wire Orange red stripe
const int  ledBoomAngStar38 = 37;  // wire Blue

const int  ledBoomAng_Req_Port5  = 38;// Start of Required Boom angle leds 15/2/17  // wire  Red black stripe
const int  ledBoomAng_Req_Port15 = 39; //  wire Red green stripe
const int  ledBoomAng_Req_Port30 = 40; //  wire White black stripe
const int  ledBoomAng_Req_Port38 = 41; //  wire White red stripe
const int  ledBoomAng_Req_Star5  = 42; //  wire Blue white stripe
const int  ledBoomAng_Req_Star15 = 43; //  wire Red  white stripe
const int  ledBoomAng_Req_Star30 = 44; //  wire Black white stripe
const int  ledBoomAng_Req_Star38 = 45; //  wire Orange green stripe
int countToggle = 0 ;

 
void setup() {
  // put your setup code here, to run once:

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

  delay (2000); // put in 14/9/19 , as serial comms were not establishing , winch sketch has a delay of 1000 . so we are Staggering the Initialisation of the Serial Ports .
  
Serial.begin (9600);
Serial3.begin(4800);  // transmission between winch arduino and this graphic arduino TX 14  RX 15
Serial2.begin(9600);
delay(500);
}

void sendAnglesToWiFi() {
  int sendBoomAngle = actualBoomAngle;
  int sendPredictedAngle = requiredBoomAngle;
  
  if (boomSide == 2) {
    sendBoomAngle = sendBoomAngle * (-1);
    requiredBoomAngle = requiredBoomAngle * (-1);
  }

  String result = "<";
  result = result + sendBoomAngle;
  result = result + ",";
  result = result + sendPredictedAngle;
  result = result + ">";

  Serial2.println(result);
  Serial.println(result);
}

void loop() {

  sendAnglesToWiFi();
  // put your main code here, to run repeatedly:
  
           /* BOOM ANGLE RECEIVING  SECTION FROM BOOM ANGLE SKETCH ON SERIAL 2, PINS TX 16 , RX 17
             ****************************** */
   Serial.print("startTransmissionRequest = " );
   Serial.println(startTransmissionRequest );

   if(startTransmissionRequest == 0)
    {
     Serial3.write('v');    // changed from Serial1.print("x");
     startTransmissionRequest = 1;
     transmissionCount = millis();
    }

   if (Serial3.available()) 
    {        // NOTE THIS IS Serial 3 14 TX , 15 RX .
      Serial.print (" HERE " );
     char  inChar = Serial3.read();
      if(inChar == 'c') 
       {
        Serial.print (" HERE AGAIN " );
        windAngleInt = Serial3.parseInt();  // STOPPED USING BOOM SENSOR TO DETERMINE BOOM SIDE AND STARTED USING WIND ANGLE  16/8/19
        Serial3.write('w');               //******************************************************************************************
      
       }

      if(inChar == 'd')
       {
        Serial.print (" HERE STILL " );
        actualBoomAngle = Serial3.parseInt();
        startTransmissionRequest = 0;
        Serial.print ( " Time Taken for Serial 2 Section = " );
        Serial.println( millis() - transmissionCount );
       }

    }// END OF IF SERIAL3 AVAILABLE

    // Determine Boom Side Section
    //******************************
    if(windAngleInt > 0 && windAngleInt < 180 )
      {
        boomSide = 1 ; // PORT
      }

     if(windAngleInt > 180 && windAngleInt < 360 )
       {
        boomSide = 2 ; // STARBOARD
       }


        // MAPPING WIND ANGLE TO REQUIRED BOOM ANGLE
            //  ********************************************    
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

             //  } // 22/8/17     closes " if( requiredBoomAngle_delayReset ==0 )
  
    
             // } 
             // } 
     
   // **********************************************END OF REQUIRED BOOM ANGLE SECTION           


   //   actualBoomAngle LED SECTION     ( Note ; LED POSITIONS ARE CURRENTLY     5 , 15 , 30 , 38 .
   
   // ************************************

   // Starboard Section
   //********************
   if(boomSide == 2)  // starboard
   {
    if(actualBoomAngle >0 && actualBoomAngle <= 6)
      {
        digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
        digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
        digitalWrite ( ledBoomAngStar5 , HIGH);
      }
    if(actualBoomAngle > 6 && actualBoomAngle < 14 )
       {
        digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
        digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
        digitalWrite ( ledBoomAngStar5 , HIGH); digitalWrite ( ledBoomAngStar15 , HIGH);
        
       }
     if(actualBoomAngle > 14 && actualBoomAngle <= 16)
       {
        digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
        digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
        digitalWrite ( ledBoomAngStar15 , HIGH);
       }
     if(actualBoomAngle > 16 && actualBoomAngle < 29)
        {
         digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
         digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
         digitalWrite ( ledBoomAngStar15 , HIGH);digitalWrite ( ledBoomAngStar30 , HIGH); 
        }
      if(actualBoomAngle >= 29 && actualBoomAngle <= 31 )
        {
         digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
         digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
         digitalWrite ( ledBoomAngStar30 , HIGH); 
        }
      if(actualBoomAngle > 31 && actualBoomAngle < 37)
        {
         digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
         digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
         digitalWrite ( ledBoomAngStar30 , HIGH); digitalWrite ( ledBoomAngStar38 , HIGH);  
        }
      if(actualBoomAngle >= 37 )
        {
         digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
         digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
         digitalWrite ( ledBoomAngStar38 , HIGH);
        }
    
   }// END OF STARBOARD

   // PORT Section
   // **************
    if(boomSide == 1)  // PORT
      {
        if(actualBoomAngle >0 && actualBoomAngle <= 6)
      {
        digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
        digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
        digitalWrite ( ledBoomAngPort5 , HIGH);
      }
    if(actualBoomAngle > 6 && actualBoomAngle < 14 )
       {
        digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
        digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
        digitalWrite ( ledBoomAngPort5 , HIGH); digitalWrite ( ledBoomAngPort15 , HIGH);
        
       }
     if(actualBoomAngle >= 14 && actualBoomAngle <= 16)
       {
        digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
        digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
        digitalWrite ( ledBoomAngPort15 , HIGH);
       }
     if(actualBoomAngle > 16 && actualBoomAngle < 29)
        {
         digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
         digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
         digitalWrite ( ledBoomAngPort15 , HIGH);digitalWrite ( ledBoomAngPort30 , HIGH); 
        }
      if(actualBoomAngle >= 29 && actualBoomAngle <= 31 )
        {
         digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
         digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
         digitalWrite ( ledBoomAngPort30 , HIGH); 
        }
      if(actualBoomAngle > 31 && actualBoomAngle < 37)
        {
         digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
         digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
         digitalWrite ( ledBoomAngPort30 , HIGH); digitalWrite ( ledBoomAngPort38 , HIGH);  
        }
      if(actualBoomAngle >= 37 )
        {
         digitalWrite(ledBoomAngStar5 , LOW); digitalWrite(ledBoomAngStar15 , LOW);digitalWrite(ledBoomAngStar30 , LOW); digitalWrite(ledBoomAngStar38 , LOW);
         digitalWrite(ledBoomAngPort5 , LOW); digitalWrite(ledBoomAngPort15 , LOW);digitalWrite(ledBoomAngPort30 , LOW); digitalWrite(ledBoomAngPort38 , LOW);
         digitalWrite ( ledBoomAngPort38 , HIGH);
        }
      }

     if( millis() >= transmissionCount1 + 2000 )
       {
         countToggle = 0;
       }
  

     if(countToggle == 0 )
      {
       Serial.print ("windAngleInt is ");
       Serial.println (windAngleInt);
       Serial.print (" actualBoomAngle is ");
       Serial.println(actualBoomAngle);
       Serial.print( " Boom Side is ");
       Serial.println(boomSide);
       Serial.print(" requiredBoomAngle is " );
       Serial.println(requiredBoomAngle);
    // Serial.print("intTemp is ");
    // Serial.println(intTemp);
       countToggle = 1;
       transmissionCount1 = millis();
      }
  
    // *********************************************************************************  

}
