const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

int boomAngle = 0;
int predictedAngle = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
}

void loop() {
    recvWithStartEndMarkers();
    parseData();
    showParsedData();
    delay(1000);
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

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
    Serial.println(receivedChars);
      
    char * strtokIndx;
    
    strtokIndx = strtok(receivedChars, ",");
    boomAngle = atoi(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    predictedAngle = atoi(strtokIndx);

    newData = false;
  }
  
}


void showParsedData() {
 Serial.print("Boom angle ");
 Serial.println(boomAngle);
 Serial.print("Predicted angle ");
 Serial.println(predictedAngle);
}
