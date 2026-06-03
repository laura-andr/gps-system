//import libraries 
#include  <Arduino.h>

//define physical pins
#define RX2 16
#define TX2 17

//every week? send the error messages as part of the overnight
#define ERR_AT_TIMEOUT 100
#define ERR_AT_FAILURE 101
#define ERR_GPS_INIT 200
#define ERR_GPS_INFO 201
#define ERR_GPS_FIX_TIMEOUT 202
#define ERR_NET_REGISTRATION 300
#define ERR_NET_APN 301
#define ERR_NET_OPEN 302
#define ERR_NET_CLOSE 303
#define ERR_NET_ACTIVATION 304
#define ERR_NET_IP 305
#define ERR_INIT_SOCKET_CONNECTION 400
#define ERR_SOCKET_MSG_FAILURE 401

//define error log
int errorLog[64];
int errorIndex = 0;

//define constants
const String APN = "fast.t-mobile.com";
const String HOST = "www.botech.com.co";
const String PORT = "9504";
const String PATH = "/";
const String DEVICE_ID = "189";
const int GPS_INTERVAL = 10; // tracking interval in seconds
const int NMEA_MAX_LENGTH = 82;
const int GPS_DATA_ARRAY_SIZE = 1500;
const int PACKAGE_SIZE = 100;

const int THRESH_SPEED = 13; //in knots
const int THRESH_DIST = 300; //in meters
const int THRESH_BEARING = 10; //in degrees

//timeouts
const unsigned long TO_LOCAL = 2000;
const unsigned long TO_CELL = 10000;
const unsigned long TO_SOCKET = 30000;

//global variables
unsigned long lastUpdate;
unsigned long gpsStartTime;
String gpsData;
String gpsFields[18];
String nmeaSentence;
char nmeaArray[NMEA_MAX_LENGTH][GPS_DATA_ARRAY_SIZE];
int nmeaIndex = 0;

//function headers
void stop();
void addError(int code);
void waitForResponse();
String ddToDegMin(String dd_str, bool isLatitude);
double distanceMeters(double lat1, double lon1, double lat2, double lon2);
int gpsDataToArray(String data, char separator, String* arrayOut, int maxItems);
String addChecksum(String sentence);
bool ATNETOPEN();
bool ATCIPOPEN();
String sendAT(String msg, unsigned long timeout);
bool sendATincludes(String msg, String inclusion, unsigned long timeout);
void setupGPS();
void setupPDP();
void fixLocation();
void startSocket();
bool getGPS();
bool sendNmeaToSocket();
void addNmeaToArray();
void buildNmea();


void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
  delay(1000);
    
  Serial2.println("AT");
  delay(500);

  while(Serial2.available()){
    Serial.write(Serial2.read());
  }
  delay(1000);
  sendAT("AT+IPR=115200", TO_LOCAL);
  delay(1000);
  setupGPS();
  setupPDP();
  fixLocation();
  startSocket();

  lastUpdate = millis();
}

void loop() {
  if (millis() - lastUpdate >= (GPS_INTERVAL * 1000)){
    lastUpdate = millis();
    if (getGPS()){
      //every successful getGPS will change the global nmeaSentence var
      //we will also have the gpsFields include the current nmeaSentence
      startSocket();
      if(!sendNmeaToSocket()){
        addNmeaToArray();
        addError(ERR_SOCKET_MSG_FAILURE);
      }
    }
  }
}

//helper functions
void stop() {
  for (int i = 0; i < errorIndex; i++) {
    Serial.println(errorLog[i]);
  }

  Serial.println("going into deep sleep");
  while (true){
    yield();
  }
}

void addError(int code) {
  if (errorIndex < 64) {
    errorLog[errorIndex++] = code;
  }
}

void waitForResponse(){
  unsigned long startWait = millis();
  while (Serial2.available() == 0){
    if (millis() - startWait > 2000) {
      yield();
      return;
    }
  }
}

// Convert decimal degrees in string form (dd.dddd or -dd.dddd) to NMEA degrees-minutes format:
// latitude: ddmm.mmmm (2-digit degrees), longitude: dddmm.mmmm (3-digit degrees)
String ddToDegMin(String dd_str, bool isLatitude) {
  if (dd_str.length() == 0) return "";

  double dd = dd_str.toFloat();
  bool negative = dd < 0;
  if (negative) dd = -dd;

  int deg = (int)dd;
  double frac = dd - deg;
  double minutes = frac * 60.0;

  char buf[16];
  if (isLatitude) {
    sprintf(buf, "%02d%07.4f", deg, minutes);
  } else {
    sprintf(buf, "%03d%07.4f", deg, minutes);
  }

  String out = String(buf);
  return out;
}

int gpsDataToArray(String data, char separator, String* arrayOut, int maxItems) {
  int arrayIndex = 0;
  int fromIndex = 0;
  int sepIndex = 0;

  while ((sepIndex = data.indexOf(separator, fromIndex)) != -1 && arrayIndex < maxItems - 1) {
    arrayOut[arrayIndex++] = data.substring(fromIndex, sepIndex);
    fromIndex = sepIndex + 1;
  }
  if (arrayIndex < maxItems) {
    arrayOut[arrayIndex++] = data.substring(fromIndex);
  }
  return arrayIndex; 
}

String addChecksum(String sentence) {
    int check = 0;
    int startIndex = (sentence.charAt(0) == '$') ? 1 : 0;
    
    for (int i = startIndex; i < sentence.length(); i++) {
        check ^= sentence.charAt(i);
    }
    
    String hexCheck = String(check, HEX);
    hexCheck.toUpperCase();
    if (hexCheck.length() < 2) hexCheck = "0" + hexCheck;
    
    return sentence + "*" + hexCheck;
}

bool ATNETOPEN(){
  unsigned long timeout = TO_CELL;
  for (int tries = 0; tries<4; tries++) {
    bool retry = false;
    String curLine = "";

    Serial2.println("AT+NETOPEN");

    unsigned long start = millis();
    while (millis() - start < timeout) {
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.write(c);
        if (c == '\n') {
          curLine.trim();
          if (curLine.length() > 0) {
            if (curLine.indexOf("+NETOPEN:")!=-1){
              if(curLine.indexOf("+NETOPEN: 0")!=-1){
                return true;
              }
              retry = true;
              //otherwise there was some error, try again
              break;
            }
            if (curLine.indexOf("ERROR")!=-1){
              retry = true;
              break;
            }

          }
          curLine = "";
        }
        else if (c != '\r') {
          curLine += c;
        }
      }
      yield();
      if (retry){
        break;
      }
      delay(1);
    }
    addError(ERR_NET_OPEN);
    delay(500);
  }
  addError(ERR_AT_FAILURE);
  return false;
}

bool ATNETCLOSE(){
  unsigned long timeout = TO_SOCKET;
  for (int tries = 0; tries<4; tries++) {

    while(Serial2.available()){
      Serial.write(Serial2.read()); 
    }

    bool retry = false;
    String curLine = "";

    Serial2.println("AT+NETCLOSE");

    unsigned long start = millis();
    while (millis() - start < timeout) {
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.write(c);
        if (c == '\n') {
          curLine.trim();
          if (curLine.length() > 0) {
            if (curLine.indexOf("OK")!=-1){
              while(Serial2.available()){
                Serial.write(Serial2.read());
              }
              return true;
            }
            if (curLine.indexOf("OK")!=-1 || curLine.indexOf("+NETCLOSE: 2")!=-1 || curLine.indexOf("+NETCLOSE: 0")!=-1){
              while(Serial2.available()){
                Serial.write(Serial2.read());
              }
              return true;
            }
            if (curLine.indexOf("ERROR")!=-1){
              retry = true;
              break;
            }

          }
          curLine = "";
        }
        else if (c != '\r') {
          curLine += c;
        }
      }
      yield();
      if (retry){
        break;
      }
      waitForResponse();
    }
    addError(ERR_NET_CLOSE);
    delay(500);
  }
  addError(ERR_AT_FAILURE);
  return false;
}

bool ATCIPOPEN(){
  unsigned long timeout = TO_SOCKET;
  for (int tries = 0; tries<4; tries++) {

    while(Serial2.available()){
      Serial.write(Serial2.read());
    }

    bool retry = false;
    String curLine = "";

    String cmd = "AT+CIPOPEN=0,\"TCP\",\"" + HOST + "\"," + PORT;
    Serial2.println(cmd);

    unsigned long start = millis();
    while (millis() - start < timeout) {
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.write(c);
        if (c == '\n') {
          curLine.trim();
          if (curLine.length() > 0) {
            if (curLine.indexOf("+CIPOPEN:")!=-1){
              if(curLine.indexOf("+CIPOPEN: 0,0")!=-1){
                return true;
              }
              retry = true;
              break;
            }
            if (curLine.indexOf("ERROR")!=-1){
              retry = true;
              break;
            }

          }
          curLine = "";
        }
        else if (c != '\r') {
          curLine += c;
        }
      }
      yield();
      if (retry){
        break;
      }
      waitForResponse();
    }
    addError(ERR_INIT_SOCKET_CONNECTION);
    sendAT("AT+CIPCLOSE=0", TO_LOCAL);
    setupPDP();
    delay(500);
  }
  addError(ERR_AT_FAILURE);
  return false;
}

String sendAT(String msg, unsigned long timeout) {
  int numTries = 0;
  while (numTries < 4) {
    String out = "";
    String curLine = "";
    Serial2.println(msg);
    unsigned long start = millis();
    while (millis() - start < timeout) {
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.write(c);
        if (c == '\n') {
          curLine.trim();
          if (curLine.length() > 0) {
            out += curLine + "\n";
            if (curLine.indexOf("OK") != -1) {
              return out;
            }
            if (curLine.indexOf("ERROR") != -1) {
              return out;
            }
          }
          curLine = "";
        }
        else if (c != '\r') {
          curLine += c;
        }
      }
      yield();
    }
    addError(ERR_AT_TIMEOUT);
    delay(500);
    numTries++;
  }
  addError(ERR_AT_FAILURE);
  return "ERROR_TIMEOUT";
}

bool sendATincludes(String msg, String inclusion, unsigned long timeout){
  String response = sendAT(msg, timeout);
  return response.indexOf(inclusion) != -1;
}

void setupGPS(){
  sendAT("AT+CGNSSPWR=0", TO_LOCAL);
  delay(500);
  Serial2.println("AT+CGNSSPWR=1");
  unsigned long start = millis();
  bool ready = false;
  String curLine = "";
  while (millis() - start < 10000){
    while (Serial2.available()){
      char c = Serial2.read();
      Serial.write(c);
      if (c == '\n'){
        curLine.trim();
        if (curLine.indexOf("OK") != -1){
          ready = true;
          break;
        }
        curLine = "";
      } else if (c != '\r') curLine += c;
    }
    if (ready) break;
    yield();
  }

  if (!ready){
    addError(ERR_GPS_INIT);
    stop();
  }

  gpsStartTime = millis();
}

void setupPDP(){

  if (sendATincludes("AT+CEREG=1", "ERROR", TO_CELL)){
    addError(ERR_NET_REGISTRATION);
    stop();
  }

  sendAT("AT+CGACT=0,1", TO_CELL);

  String cmd = "AT+CGDCONT=1,\"IPV4V6\",\"" + APN + "\"";
  if (sendATincludes(cmd, "ERROR", TO_CELL)){    
    addError(ERR_NET_APN);
    stop();
  }

  if (sendATincludes("AT+CGACT=1,1", "ERROR", TO_CELL)){
    addError(ERR_NET_ACTIVATION);
    stop();
  }

  if (!sendATincludes("AT+NETOPEN?", "+NETOPEN: 1", TO_CELL)){
    if (!ATNETOPEN()){
      stop();
    }
  }

  String ipResponse = sendAT("AT+CGPADDR=1", TO_LOCAL);
  if (!(ipResponse.indexOf("+CGPADDR:") != -1 &&  ipResponse.indexOf("\"\"") == -1 && ipResponse.indexOf("ERROR") == -1)) {
   addError(ERR_NET_IP);
   stop();
  }

}

void fixLocation(){
  String response = sendAT("AT+CGNSSINFO", TO_LOCAL);
  if (response.indexOf("ERROR") != -1){
    addError(ERR_GPS_INFO);
    stop();
  }
  while (sendATincludes("AT+CGNSSINFO", ",,,,,,,,", TO_LOCAL)){
    if (millis() - gpsStartTime > 480000){
      setupGPS();
      gpsStartTime = millis();
      addError(ERR_GPS_FIX_TIMEOUT);
    }
    yield();
    delay(500);
  }
  Serial.println("GPS location fixed");
}

void startSocket(){
  sendAT("AT+CIPCLOSE=0", TO_LOCAL);
  delay(100);
  if (!ATCIPOPEN()){
    stop();
  }
}

bool getGPS(){
  gpsData = "";
  Serial2.println("AT+CGNSSINFO");

  waitForResponse();
  
  String curLine = "";
  bool foundData = false;

  while (Serial2.available() > 0){
    char c = Serial2.read();
    Serial.write(c);
    if (c == '\n'){
      curLine.trim();
      if (curLine.length() > 0 && curLine.indexOf("+CGNSSINFO:") != -1){
        gpsData = curLine.substring(curLine.indexOf("+CGNSSINFO:") + 11);
        gpsData.trim(); 
        foundData = true;
      }
      curLine = "";
    }
    else if (c != '\r'){
      curLine += c;
    }
    delay(1);
  }

  if (!foundData) return false;
  buildNmea();
  return true;
}

void buildNmea(){

  gpsDataToArray(gpsData, ',', gpsFields, 18);

  if (gpsFields[0] == "" || gpsFields[0] == "0" || gpsFields[5] == "" || gpsFields[7] == ""){
    nmeaSentence = "$GNRMC,,V,,,,,,,,,,N,V,"; 
  }
  else {
    String lat = ddToDegMin(gpsFields[5], true);
    String lon = ddToDegMin(gpsFields[7], false);
    nmeaSentence = "$GNRMC," + gpsFields[10] + ",A," + lat + "," + gpsFields[6] + "," + 
                  lon + "," + gpsFields[8] + "," + gpsFields[12] + "," + 
                  gpsFields[13] + "," + gpsFields[9] + ",,,A,V";
  }
  Serial.println(nmeaSentence);
}

void addNmeaToArray() {
  if (gpsFields[12].toDouble() > THRESH_SPEED) {
    nmeaIndex++;
    nmeaArray[nmeaIndex] = nmeaSentence;
    return;
  }

  if (nmeaIndex != 0) {
    double lastLat = nmeaArray[nmeaIndex - 1].substring(5, 12).toDouble();
    double lastLon = nmeaArray[nmeaIndex - 1].substring(13, 21).toDouble();
    double currLat = gpsFields[5].toDouble();
    double currLon = gpsFields[7].toDouble();
    double degrees = gpsFields[13].toDouble();

    if (distanceMeters(lastLat, lastLon, currLat, currLon) > THRESH_DIST) {
      nmeaIndex++;
      nmeaArray[nmeaIndex] = nmeaSentence;
      return;
    }

    if (abs(degrees - degrees) > THRESH_BEARING) {
      nmeaIndex++;
      nmeaArray[nmeaIndex] = nmeaSentence;
      return;
    }
  }

  //also if we have run out of spots, think of some way to reduce data size print without losing too much fidelity
  if (nmeaIndex < NMEA_MAX_LENGTH) {
    nmeaArray[nmeaIndex++] = nmeaSentence;
  }
}

bool sendNmeaToSocket(){
  while (Serial2.available()){
    Serial.write(Serial2.read()); 
  } 
  

  String payload = ">IU=" + DEVICE_ID + ",+QGPSGNMEA: " + addChecksum(nmeaSentence) + "<\r\n";
  
  String sendCmd = "AT+CIPSEND=0," + String(payload.length());
  unsigned long startAll = millis();
  int tries = 0;
  bool dataSent = false;

  while (tries < 3 && (millis() - startAll) < TO_SOCKET && !dataSent) {
    while (Serial2.available()) {
      Serial.write(Serial2.read());
    }
    Serial2.println(sendCmd);
    String curLine = "";
    while ((millis() - startAll) < TO_SOCKET) {
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.write(c);
        if (c == '>') {
          Serial2.print(payload);
          dataSent = true;
          curLine = "";
          break;
        }
      }
      if (dataSent) break;
      yield();
    }

    if (!dataSent) {
      sendAT("AT+CIPCLOSE=0", TO_LOCAL);
      tries++;
      delay(200);
    }
  }

  String curLine = "";
  bool remoteClosed = false;
  while ((millis() - startAll) < TO_SOCKET) {
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      Serial.write(c);
      if (c == '\n') {
        curLine.trim();
        if (curLine.length() > 0) {
          if (curLine.indexOf("+IPCLOSE: 0,1") != -1) {
            remoteClosed = true;
            break;
          }
        }
        curLine = "";
      } else if (c != '\r') {
        curLine += c;
      }
    }
    if (remoteClosed) break;
    yield();
  }

  return remoteClosed;
}

void sendOldNmeaToSocket(){
  String json = "{ \"nmea\": [";
  int index = PACKAGE_SIZE;
  if (nmeaIndex < PACKAGE_SIZE){
    index = nmeaIndex;
  }
  for (int i = 0; i < index; i++) {
    if (nmeaArray[i] != "") {
      json += "\"" + nmeaArray[i] + "\",";
    }
  }
  json.remove(json.length() - 1);
  json += "] }";

  // send json to socket
  Serial2.println("AT+CIPSEND=0," + String(json.length()));
  Serial2.print(json);
}