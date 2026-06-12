//import libraries 
#include <Arduino.h>

//define physical pins
#define RX2 16
#define TX2 17

//Error Logs
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
const String curLocHost = "www.botech.com.co";
const String curLocPort = "9504";
const String oldLocHost = "dev.botech.com.co";
const String oldLocPort = "9504";
const String DEVICE_ID = "189";
const int GPS_INTERVAL = 10; // tracking interval in seconds
const int NMEA_MAX_LENGTH = 82;
const int PACKAGE_SIZE = 100;

const int THRESH_SPEED = 13; //in knots
const int THRESH_DIST = 300; //in meters
const int THRESH_BEARING = 10; //in degrees

//timeouts
const unsigned long TO_LOCAL = 2000;
const unsigned long TO_CELL = 10000;
const unsigned long TO_SOCKET = 10000;

//global variables
unsigned long lastUpdate;
unsigned long gpsStartTime;
String gpsData;
String gpsFields[18];
String nmeaSentence;
String nmeaArray[NMEA_MAX_LENGTH];
int nmeaIndex = 0;
int failuresToSend = 0;
String curLocChannel = "0";
String oldLocChannel = "1";

//function headers
void stop();
void addError(int code);
void waitForResponse();
String ddToDegMin(String dd_str, bool isLatitude);
double calculateFlatDistance(double lat1, double lon1, double lat2, double lon2);
double distanceMeters(double lat1_dm, double lon1_dm, double lat2_dm, double lon2_dm);
int csvToArray(String data, char separator, String* arrayOut, int maxItems);
String addChecksum(String sentence);
bool ATNETOPEN();
bool ATNETCLOSE();
bool ATCIPOPEN(String host, String port, String channel);
String sendAT(String msg, unsigned long timeout);
bool sendATincludes(String msg, String inclusion, unsigned long timeout);
void setupGPS();
void setupPDP();
void fixLocation();
void startSocket();
bool getGPS();
bool CIPSEND(String payload, String channel);
void addNmeaToArray();
void buildNmea();
void sendOldNmeaToSocket();
bool isSocketConnected(String channel);

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
      startSocket(); 
      
      String payload = ">IU=" + DEVICE_ID + ",+QGPSGNMEA: " + addChecksum(nmeaSentence) + "<\r\n";
      if(!CIPSEND(payload, curLocChannel)){
        failuresToSend++;
        addNmeaToArray();
        addError(ERR_SOCKET_MSG_FAILURE);
      }
      else{
        failuresToSend = 0;
        if (nmeaIndex > 0) {
          sendOldNmeaToSocket();
        }
      }
    }
    
    if (failuresToSend >= 3){
      if (!sendATincludes("AT+CEREG?", "+CEREG: 0,1", TO_CELL)){
        addError(ERR_NET_REGISTRATION);
        sendAT("AT+COPS=2", TO_CELL);
        sendAT("AT+COPS=0", TO_CELL);
        
        unsigned long regStart = millis();
        while(millis() - regStart < 15000) {
          if (sendATincludes("AT+CEREG?", "+CEREG: 0,1", TO_CELL)) break;
          delay(1000);
        }
      }
      else{
        startSocket(); 
      }
      failuresToSend = 0;
    }
  }
}

void stop() {
  for (int i = 0; i < errorIndex; i++) {
    Serial.println(errorLog[i]);
  }
  ESP.restart();
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

  return String(buf);
}

double calculateFlatDistance(double lat1, double lon1, double lat2, double lon2) {
  double latMid = (lat1 + lat2) * 0.017453292519943295 / 2.0; 
  double m_per_deg_lat = 111132.954 - 559.822 * cos(2 * latMid) + 1.175 * cos(4 * latMid);
  double m_per_deg_lon = 111412.84 * cos(latMid) - 93.5 * cos(3 * latMid);

  double deltaLat = (lat1 - lat2) * m_per_deg_lat;
  double deltaLon = (lon1 - lon2) * m_per_deg_lon;

  return sqrt(deltaLat * deltaLat + deltaLon * deltaLon); 
}

double distanceMeters(double lat1_dm, double lon1_dm, double lat2_dm, double lon2_dm) {
  auto dmToDec = [](double dm)->double {
    int deg = (int)(dm / 100.0);
    double minutes = dm - (double)deg * 100.0;
    return (double)deg + (minutes / 60.0);
  };

  double lat1 = dmToDec(lat1_dm);
  double lon1 = dmToDec(lon1_dm);
  double lat2 = dmToDec(lat2_dm);
  double lon2 = dmToDec(lon2_dm);

  return calculateFlatDistance(lat1, lon1, lat2, lon2);
}

int csvToArray(String data, char separator, String* arrayOut, int maxItems) {
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
      if (retry) break;
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
            if (curLine.indexOf("OK")!=-1 || curLine.indexOf("+NETCLOSE: 2")!=-1 || curLine.indexOf("+NETCLOSE: 0")!=-1){
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
      if (retry) break;
      waitForResponse();
    }
    addError(ERR_NET_CLOSE);
    delay(500);
  }
  addError(ERR_AT_FAILURE);
  return false;
}

bool ATCIPOPEN(String host, String port, String channel){
  unsigned long timeout = TO_SOCKET;
  for (int tries = 0; tries<4; tries++) {
    while(Serial2.available()){
      Serial.write(Serial2.read());
    }

    bool retry = false;
    String curLine = "";

    String cmd = "AT+CIPOPEN=" + channel + ",\"TCP\",\"" + host + "\"," + port;
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
              if(curLine.indexOf("+CIPOPEN: " + channel + ",0")!=-1 || curLine.indexOf("+CIPOPEN: " + channel + ",4")!=-1){
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
      if (retry) break;
      waitForResponse();
    }
    addError(ERR_INIT_SOCKET_CONNECTION);
    sendAT("AT+CIPCLOSE=" + channel, TO_LOCAL);
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
            if (curLine.indexOf("OK") != -1) return out;
            if (curLine.indexOf("ERROR") != -1) return out;
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
  if (!sendATincludes("AT+CGNSSPWR=1", "OK", 10000)){
    if (sendATincludes("AT+CFUN?", "1", 1000)){
      addError(ERR_GPS_INIT);
      stop();
    }
    else if (sendATincludes("AT+CFUN=1", "ERROR", TO_LOCAL)){
        addError(ERR_GPS_INIT);
        stop();
    }
    sendAT("AT+CGNSSPWR=0", TO_LOCAL);
    delay(500);
    if (!(sendATincludes("AT+CGNSSPWR=1", "OK", 10000))){
      addError(ERR_GPS_INIT);
      stop();
    }
  }
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
      ATNETCLOSE();
      if(!ATNETOPEN()){
        stop();
      }
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
  if (!sendATincludes("AT+CIPOPEN?", "+CIPOPEN: " + curLocChannel + ",", TO_LOCAL)) {
    ATCIPOPEN(curLocHost, curLocPort, curLocChannel);
  }
  if (!sendATincludes("AT+CIPOPEN?", "+CIPOPEN: " + oldLocChannel + ",", TO_LOCAL)) {
    ATCIPOPEN(oldLocHost, oldLocPort, oldLocChannel);
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
  csvToArray(gpsData, ',', gpsFields, 18);

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
  if (nmeaIndex >= NMEA_MAX_LENGTH) return;

  if (nmeaIndex != 0) {
    if (!(gpsFields[12].toDouble() > THRESH_SPEED)) return;

    String lastNmea = nmeaArray[nmeaIndex-1];
    String lastNmeaArray[14];
    csvToArray(lastNmea, ',', lastNmeaArray, 14);

    String curNmeaArray[14];
    csvToArray(nmeaSentence, ',', curNmeaArray, 14);

    double lastLat = lastNmeaArray[3].toDouble();
    if (lastNmeaArray[4] == "S") lastLat = -lastLat;
    double lastLon = lastNmeaArray[5].toDouble();
    if (lastNmeaArray[6] == "W") lastLon = -lastLon;
    double lastDegrees = lastNmeaArray[8].toDouble();

    double curLat = curNmeaArray[3].toDouble();
    if (curNmeaArray[4] == "S") curLat = -curLat;
    double curLon = curNmeaArray[5].toDouble();
    if (curNmeaArray[6] == "W") curLon = -curLon;
    double curDegrees = curNmeaArray[8].toDouble();

    if (!(distanceMeters(lastLat, lastLon, curLat, curLon) > THRESH_DIST)) return;
    if (!(abs(curDegrees - lastDegrees) > THRESH_BEARING)) return;
  }

  nmeaArray[nmeaIndex++] = nmeaSentence;
  Serial.print("Saved offline point. Current index: ");
  Serial.println(nmeaIndex);
}

bool CIPSEND(String payload, String channel){
  while (Serial2.available()){
    Serial.write(Serial2.read()); 
  }   
  
  String sendCmd = "AT+CIPSEND=" + channel + "," + String(payload.length());
  unsigned long startAll = millis();
  int tries = 0;
  bool dataSent = false;

  while (tries < 3 && (millis() - startAll) < TO_SOCKET && !dataSent) {
    while (Serial2.available()) {
      Serial.write(Serial2.read());
    }
    Serial2.println(sendCmd);
    String curLine = "";
    while ((millis() - startAll) < TO_SOCKET/3) {
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.write(c);

        if (c == '>') {
          Serial2.print(payload);
          dataSent = true;
          curLine = "";
          break;
        }
        else if (c == '\n') {
          curLine.trim();
          if (curLine.length() > 0 && curLine.indexOf("+CIPERROR:") != -1) {
              break;
          }
          curLine = "";
        }
        else if (c != '\r') {
          curLine += c;
        }
      }
      if (dataSent) break;
      yield();
    }

    if (!dataSent) {
      sendAT("AT+CIPCLOSE=" + channel, TO_LOCAL); 
      tries++;
      delay(200);
    }
  }

  String curLine = "";
  bool remoteClosed = false;
  unsigned long waitAckStart = millis();
  String serverResponse = ""; 

  while ((millis() - waitAckStart) < 5000) { 
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      Serial.write(c);
      if (c == '\n') {
        curLine.trim();
        if (curLine.length() > 0) {
          serverResponse += curLine + "\n";
          if (curLine.indexOf("+IPCLOSE: " + channel + ",1") != -1) {
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
  
  if (channel == oldLocChannel && serverResponse.indexOf("Insertado con identificador") == -1) {
    return false; 
  }

  return dataSent;
}

void sendOldNmeaToSocket(){
  int itemsToSend = (nmeaIndex < PACKAGE_SIZE) ? nmeaIndex : PACKAGE_SIZE;
  if (itemsToSend == 0) return; 

  String json = "[";
  for (int i = 0; i < itemsToSend; i++) {
    if (nmeaArray[i] != "") {
      json += "{" + addChecksum(nmeaArray[i]).substring(1) + "\r\n},";
    }
  }

  if (json.endsWith(",")) {
    json.remove(json.length() - 1);
  }
  json += "]";

  String payload = ">IH=" + DEVICE_ID + "," + json + "<\r\n";

  if (CIPSEND(payload, oldLocChannel)) {    
    for (int i = 0; i < itemsToSend; i++) {
      nmeaArray[i] = ""; 
    }

    int writeIndex = 0;
    for (int i = 0; i < nmeaIndex; i++) {
      if (nmeaArray[i] != "") {
        if (writeIndex != i) {
          nmeaArray[writeIndex] = nmeaArray[i];
          nmeaArray[i] = ""; 
        }
        writeIndex++;
      }
    }    
    nmeaIndex = writeIndex;
  }
}