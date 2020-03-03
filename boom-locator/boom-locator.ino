#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

#ifndef APSSID
#define APSSID "ESPap"
#define APPSK  "thereisnospoon"
#endif

const char *PAGE = "<!DOCTYPE html><html lang=\"en\"><head> <meta charset=\"UTF-8\"> <title>Boom Locator</title></head><style>.container{position: absolute; top: 35%; left: 39%; margin-right: -50%; transform: translate(-50%, -50%)}.boom{width: 330px; height: 17px; transform: translate(150px, 10px) rotate(90deg); transform-origin: left; background: #ff0000; position: absolute;}.predicted{width: 330px; height: 17px; transform: translate(150px, 10px) rotate(100deg); transform-origin: left; background: #000000; position: absolute;}.angle{text-align: center; margin-top: 7%;}.predictedAngle{text-align: center;}</style><body><!--<svg width=\"400\" height=\"110\"> <rect width=\"300\" height=\"20\" style=\"fill:rgb(0,0,255)\"/></svg>--><div class=\"container\" id=\"container\"> <div class=\"boom\" id=\"boom\"></div><div class=\"predicted\" id=\"predicted\"></div></div><h1 id=\"angle\" class=\"angle\">not this</h1><h1 id=\"predictedAngle\" class=\"predictedAngle\">not this</h1><script>let boomAngle=(90 - 65); let predictedAngle=boomAngle + 10; let boomMovement=+1; let predictedMovement=+1; setInterval(function (){updateAngles()}, 500); function updateAngles(){let request=new XMLHttpRequest(); request.open('GET', '/api/angles'); request.onload=function (){let data=JSON.parse(this.response); if (request.status===200){console.log(data); updateUI(data);}else{console.log('error')}}; request.send();}function updateUI(angles){boomAngle=angles['actual']; predictedAngle=angles['predicted']; let boomElement=document.getElementById(\"boom\"); let predictedElement=document.getElementById(\"predicted\"); if (boomAngle % 360 < 90){boomElement.style.background=\"#ff0000\";}else{boomElement.style.background=\"#00FF00\";}function limitAngle(angle, movement){const ZERO_ANGLE=90; const LEFT_LIMIT=ZERO_ANGLE + 65; const RIGHT_LIMIT=ZERO_ANGLE - 65; if (angle > LEFT_LIMIT){return -1}else if (angle < RIGHT_LIMIT){return +1}return movement}/*boomMovement=limitAngle(boomAngle, boomMovement); predictedMovement=limitAngle(predictedAngle, predictedMovement);*/ boomElement.style.transform=\"translate(150px, 10px) rotate(\" + boomAngle + \"deg)\"; predictedElement.style.transform=\"translate(150px, 10px) rotate(\" + predictedAngle + \"deg)\"; document.getElementById(\"angle\").textContent=\"Current: \" + (boomAngle - 90) + \"°\"; document.getElementById(\"predictedAngle\").textContent=\"Optimum: \" + (predictedAngle - 90) + \"°\";}</script></body></html>";

/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;

WebServer server(80);

/* Just a little test message.  Go to http://192.168.4.1 in a web browser
   connected to this access point to see it.
*/
void handleRoot() {
  Serial.println("Got request for /");
  server.send(200, "text/html", PAGE);
}

void handleAngleRequest() {
  Serial.println("Got API angle request");
  server.send(200, "application/json", "{\"predicted\":40,\"boom\":80}");
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/", handleRoot);
  server.on("/api/angles", handleAngleRequest);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
