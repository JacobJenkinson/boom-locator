#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

#ifndef APSSID
#define APSSID "ESPap"
#define APPSK  "thereisnospoon"
#endif

#define SERIAL_MONITOR Serial
#define SERIAL_IN Serial

const char *PAGE = "<!DOCTYPE html><html lang=\"en\"><head> <meta charset=\"UTF-8\"> <title>Boom Locator</title></head><style>.container{position: absolute; top: 35%; left: 50%;}.boom{width: 17px; height: 330px; transform: rotate(0deg); transform-origin: top; background: #ff0000; position: absolute;}.predicted{width: 17px; height: 170px; transform: rotate(0deg); transform-origin: top; background: #000000; position: absolute;}.angle{text-align: center; margin-top: 7%;}.predictedAngle{text-align: center;}</style><body><!--<svg width=\"400\" height=\"110\"> <rect width=\"300\" height=\"20\" style=\"fill:rgb(0,0,255)\"/></svg>--><div class=\"container\" id=\"container\"> <div class=\"boom\" id=\"boom\"></div><div class=\"predicted\" id=\"predicted\"></div></div><h1 id=\"angle\" class=\"angle\">not this</h1><h1 id=\"predictedAngle\" class=\"predictedAngle\">not this</h1><script>let boomAngle=0; let predictedAngle=0; setInterval(function (){updateAngles()}, 500); function updateAngles(){let request=new XMLHttpRequest(); request.open('GET', '/api/angles'); request.onload=function (){let data=JSON.parse(this.response); if (request.status===200){console.log(data); updateUI(data);}else{console.log('error')}}; request.send();}function updateUI(angles){boomAngle=angles['boom']; predictedAngle=angles['predicted']; let boomElement=document.getElementById(\"boom\"); let predictedElement=document.getElementById(\"predicted\"); if (boomAngle % 360 < 0){boomElement.style.background=\"#00FF00\";}else{boomElement.style.background=\"#ff0000\";}function limitAngle(angle, movement){const ZERO_ANGLE=90; const LEFT_LIMIT=ZERO_ANGLE + 65; const RIGHT_LIMIT=ZERO_ANGLE - 65; if (angle > LEFT_LIMIT){return -1}else if (angle < RIGHT_LIMIT){return +1}return movement}boomElement.style.transform=\"rotate(\" + boomAngle + \"deg)\"; predictedElement.style.transform=\"rotate(\" + predictedAngle + \"deg)\"; document.getElementById(\"angle\").textContent=\"BOOM: \" + boomAngle + \"°\"; document.getElementById(\"predictedAngle\").textContent=\"PREDICTED: \" + predictedAngle + \"°\";}</script></body></html>";

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;

// --------------------------------------------------------------------------------
// setup for receiving angles in format <boom,predicted>
const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

int boomAngle = 0;
int predictedAngle = 0;
// --------------------------------------------------------------------------------


WebServer server(80);

/* Just a little test message.  Go to http://192.168.4.1 in a web browser
   connected to this access point to see it.
*/
void handleRoot() {
  Serial.println("Got request for /");
  server.send(200, "text/html", PAGE);
}

void handleAngleRequest() {
  String result = "";
  result = result + "{\"predicted\":";
  result = result + predictedAngle;
  result = result + ",\"boom\":";
  result = result + boomAngle;
  result = result + "}";
  
  server.send(200, "application/json", result);
}

void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial1.begin(9600);
  SERIAL_MONITOR.println();
  SERIAL_MONITOR.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  SERIAL_MONITOR.print("AP IP address: ");
  SERIAL_MONITOR.println(myIP);
  server.on("/", handleRoot);
  server.on("/api/angles", handleAngleRequest);
  server.begin();
  SERIAL_MONITOR.println("HTTP server started");
}

void loop() {
  
  recvWithStartEndMarkers();
  parseData();
  
  server.handleClient();
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (SERIAL_IN.available() > 0 && newData == false) {
        rc = SERIAL_IN.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
 
void parseData() {
  if (newData) {
    char * strtokIndx;
    
    strtokIndx = strtok(receivedChars, ",");
    boomAngle = atoi(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    predictedAngle = atoi(strtokIndx);

    newData = false;
  }
  
}
