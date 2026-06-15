#include  <Arduino.h>
#include <Update.h>

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
#define ERR_HTTP_INIT_FAILURE 500
#define ERR_HTTP_GET_FAILURE 501
#define ERR_VERSION_FILE_SIZE 502
#define ERR_VERSION_FILE_READ 503
#define ERR_FIRMWARE_SIZE 504
#define ERR_INSUFFICIENT_SPACE 505
#define ERR_READ_UPDATE 506
#define ERR_UPDATE_FAILURE 507

//define error log
int errorLog[64];
int errorIndex = 0;

//define constants
const String APN = "fast.t-mobile.com";
const String HOST = "www.botech.com.co";
const String PORT = "9504";
const String PATH = "/";
const String DEVICE_ID = "189";

//CHANGE THIS EVERY UPDATE!!
const int LOCAL_VERSION = 0;
const String version_url = "https://dl.dropboxusercontent.com/scl/fi/r0rhfi73h7iciwrlumrls/version.txt?rlkey=ddjovrejh5wg06vr26z63uvmu&st=9zyuv7eo&dl=1";
const String firmware_url = "https://dl.dropboxusercontent.com/scl/fi/1x22iaxccj0rqc2e2tuvf/test-gps-code.ino.bin?rlkey=eh8112jyw16821s8k884o9d76&st=mp81a227&dl=1";
//https://dl.dropboxusercontent.com/scl/fi/[your_firmware_id]/test-gps-code.ino.bin?rlkey=[your_firmware_key]&dl=1

const int GPS_INTERVAL = 10; // tracking interval in seconds
const int NMEA_MAX_LENGTH = 82;
const int GPS_DATA_ARRAY_SIZE = 1500;
const int PACKAGE_SIZE = 100;

const int THRESH_SPEED = 13; //in knots
const int THRESH_DIST = 300; //in meters
const int THRESH_BEARING = 10; //in degrees

const String UPDATE_TIME_UTC = "000000.00"; //hhmmss.ss, set to 2am COT

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
bool otaExecutedToday = false; // Prevents re-trigger loop
String dateLastUpdated = "000000"; // ddmmyy
int serverVersion = 0;
int firmwareSize = 0; // Will be parsed dynamically from version.txt

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

String getTime() {
  if (getGPS()) {
    if (gpsFields[10].length() >= 6) {
      return gpsFields[10].substring(0, 6); 
    }
  }
  return "000000";
}

String getDate() {
  if(getGPS()){
    if (gpsFields[9].length() >= 2) {
      return gpsFields[9].substring(0, 2);
    }
  }
  return "00"; // Fallback
}

void loop() {
  // if (millis() - lastUpdate >= (GPS_INTERVAL * 1000)){
  //   lastUpdate = millis();
  //   if (getGPS()){
  //     startSocket();
  //     if(!sendNmeaToSocket()){
  //       addError(ERR_SOCKET_MSG_FAILURE);
  //     }
  //   }
  // }
  delay(10000);
  String curTime = getTime();
  Serial.println("curTime"+curTime);
  if (curTime >= UPDATE_TIME_UTC) {
    sendAT("AT+CIPCLOSE=0", TO_LOCAL);
    String curDate = getDate();
    Serial.println("lastUpdate:"+dateLastUpdated);
    Serial.println("curDate:"+curDate);
    if (dateLastUpdated != curDate) {
       
      doOTA();
    }
  }
}

long httpGET(String url) {
  int tries = 0;

  while (tries < 3){
    // 1. Clear out active or hung sessions first
    sendAT("AT+HTTPTERM", TO_CELL); 
    delay(100);
    
    // 2. Initialize the HTTP client application
    if (!sendATincludes("AT+HTTPINIT", "OK", TO_CELL)){
      addError(ERR_HTTP_INIT_FAILURE);
      tries++;
      delay(500);
      continue;
    } 

    // 3. Configure internal SSL Engine for SIM7670 (Context 0)
    sendAT("AT+CSSLCFG=\"sslversion\",0,4", TO_LOCAL); // Target TLS 1.2
    sendAT("AT+CSSLCFG=\"authmode\",0,0", TO_LOCAL);   // Ignore certificate validation chains
    sendAT("AT+CSSLCFG=\"enableSNI\",0,1", TO_LOCAL);  // Turn on Auto-SNI handling from URL
    sendAT("AT+CSSLCFG=\"ciphersuites\",0,0xFFFF", TO_LOCAL);
    
    // 4. Bind the target URL string
    if (!sendATincludes("AT+HTTPPARA=\"URL\",\"" + url + "\"", "OK", TO_CELL)){
      addError(ERR_HTTP_GET_FAILURE);
      sendAT("AT+HTTPTERM", TO_CELL);
      tries++;
      delay(500);
      continue;
    } 
    
    // 5. Fire asynchronous GET request
    Serial2.println("AT+HTTPACTION=0");
    
    // 6. Asynchronous Listener Loop
    String actionResponse = "";
    unsigned long actionStart = millis();
    bool foundActionNotification = false;
    
    while (millis() - actionStart < 20000) { 
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.write(c); // Mirror directly to hardware serial monitor
        actionResponse += c;
        
        if (actionResponse.indexOf("+HTTPACTION:") != -1 && actionResponse.endsWith("\n")) {
          foundActionNotification = true;
          break;
        }
      }
      if (foundActionNotification) break;
      yield();
    }
    
    Serial.println("FULL ACTION RESP: [" + actionResponse + "]");

    // 7. Evaluate the captured network status response
    int actionIdx = actionResponse.indexOf("+HTTPACTION:");
    if (actionIdx != -1) {
      int firstComma = actionResponse.indexOf(",", actionIdx);
      int secondComma = actionResponse.indexOf(",", firstComma + 1);
      String status = actionResponse.substring(firstComma + 1, secondComma);
      status.trim();
      
      if (status == "200") {
        String sizeStr = actionResponse.substring(secondComma + 1);
        sizeStr.trim();
        return sizeStr.toInt(); // Returns file size
      } else {
        Serial.printf("\nDropbox Connection Rejected. Error Code: %s\n", status.c_str());
      }
    }
    
    tries++;
    delay(500);
  }
  
  addError(ERR_HTTP_GET_FAILURE);
  return 0; 
}

int checkVersion() {
  int tries = 0;
  long size = httpGET(version_url); // This will correctly return 9
  
  if (size <= 0) {
    addError(ERR_VERSION_FILE_SIZE);
    return -1;
  }
  
  while (tries < 3){
    // FIX: Dynamically read the entire file size (9 bytes) instead of just 1 byte!
    String response = sendAT("AT+HTTPREAD=0," + String(size), TO_CELL);
    
    int index = response.indexOf("+HTTPREAD:");
    if (index == -1){
      addError(ERR_VERSION_FILE_READ);
      tries++;
      delay(200);
      continue;
    }

    int startIndex = response.indexOf("\n", index);
    if (startIndex == -1){
      addError(ERR_VERSION_FILE_READ);
      tries++;
      delay(200);
      continue;
    }
    startIndex++;

    // Isolate the text payload data block safely
    String payload = response.substring(startIndex);
    if (payload.endsWith("OK")) {
      payload = payload.substring(0, payload.lastIndexOf("OK"));
    }
    payload.trim();

    // Split the multi-line payload text cleanly by its newline delimiter
    int newlineIdx = payload.indexOf("\n");
    if (newlineIdx != -1) {
      String verStr = payload.substring(0, newlineIdx);
      String sizeStr = payload.substring(newlineIdx + 1);
      verStr.trim();
      sizeStr.trim();
      
      serverVersion = verStr.toInt();
      firmwareSize = sizeStr.toInt(); // Assigns 306384 globally!
      
      Serial.printf("\n[SUCCESS] Parsed metadata -> Version: %d | Size: %ld bytes\n", serverVersion, firmwareSize);
      return serverVersion;
    } else {
      // Fallback contingency layout
      serverVersion = payload.toInt();
      firmwareSize = 306384; 
      return serverVersion;
    }
  }
  return -1;
}

void downloadNewVersion() {
  Serial.println(sendAT("AT+CSQ", TO_LOCAL));
  if (firmwareSize <= 0) {
    Serial.println("Error: Valid firmware size not found. Aborting.");
    return;
  }

  if (!Update.begin(firmwareSize)) {
    Serial.printf("CRITICAL ERROR: File size (%d bytes) exceeds available space!\n", firmwareSize);
    return;
  }

  Serial.printf("Initializing download of %d bytes...\n", firmwareSize);

  // Single HTTP session, single GET. Let the modem buffer the whole file.
  // Reset the modem's IP/HTTP stack to clear any wedged session state.
  sendAT("AT+HTTPTERM", TO_CELL);
  delay(1000);
  sendAT("AT+HTTPINIT", TO_CELL);
  delay(100);
  sendAT("AT+CSSLCFG=\"sslversion\",0,4", TO_LOCAL);
  sendAT("AT+CSSLCFG=\"authmode\",0,0", TO_LOCAL);
  sendAT("AT+CSSLCFG=\"enableSNI\",0,1", TO_LOCAL);
  sendAT("AT+CSSLCFG=\"ciphersuites\",0,0xFFFF", TO_LOCAL);
  sendAT("AT+HTTPPARA=\"URL\",\"" + firmware_url + "\"", TO_CELL);

  // Fire the GET and wait for the action notification with the total size.
  Serial2.println("AT+HTTPACTION=0");
  String actionResponse = "";
  unsigned long actionStart = millis();
  while (millis() - actionStart < 30000) {
    if (Serial2.available() > 0) {
      char c = Serial2.read();
      actionResponse += c;
      if (actionResponse.indexOf("+HTTPACTION:") != -1 && actionResponse.endsWith("\n")) break;
    }
    yield();
  }
  Serial.println("FULL ACTION RESP: [" + actionResponse + "]");
  String body = sendAT("AT+HTTPREAD=0,512", TO_CELL);
  Serial.println("BODY: [" + body + "]");

  int actionIdx = actionResponse.indexOf("+HTTPACTION:");
  long totalAvailable = 0;
  if (actionIdx != -1) {
    int firstComma = actionResponse.indexOf(",", actionIdx);
    int secondComma = actionResponse.indexOf(",", firstComma + 1);
    String status = actionResponse.substring(firstComma + 1, secondComma);
    status.trim();
    if (status != "200") {
      Serial.printf("HTTP GET failed. Status: %s\n", status.c_str());
      Update.abort();
      sendAT("AT+HTTPTERM", TO_CELL);
      return;
    }
    String sizeStr = actionResponse.substring(secondComma + 1);
    sizeStr.trim();
    totalAvailable = sizeStr.toInt();
  } else {
    Serial.println("No HTTPACTION response. Aborting.");
    Update.abort();
    sendAT("AT+HTTPTERM", TO_CELL);
    return;
  }

  if (totalAvailable != firmwareSize) {
    Serial.printf("WARNING: server file size %ld != expected %d\n", totalAvailable, firmwareSize);
    // Trust the server's actual size for reading, but Update.begin used firmwareSize.
    // If these differ, abort rather than write a truncated/oversized image.
    Update.abort();
    sendAT("AT+HTTPTERM", TO_CELL);
    return;
  }

  long currentPosition = 0;

  while (currentPosition < firmwareSize) {
    long remaining = firmwareSize - currentPosition;
    // Ask for a generous window; the modem will cap each read at its own
    // internal limit (1024 on this firmware) and tell us the real count.
    int requestLen = (remaining > 1024) ? 1024 : (int)remaining;

    // Drain anything stale before issuing the read.
    while (Serial2.available() > 0) Serial2.read();

    Serial2.println("AT+HTTPREAD=0," + String(requestLen));

    // Parse the "+HTTPREAD: <len>" data header to learn the actual count.
    String headerBuffer = "";
    unsigned long headerTimeout = millis();
    int reportedLen = -1;
    bool gotHeader = false;

    while (millis() - headerTimeout < 5000) {
      if (Serial2.available() > 0) {
        char c = Serial2.read();
        headerBuffer += c;
        int hrIdx = headerBuffer.indexOf("+HTTPREAD:");
        if (hrIdx != -1 && c == '\n') {
          int colon = headerBuffer.indexOf(":", hrIdx);
          String lenStr = headerBuffer.substring(colon + 1);
          lenStr.trim();
          int rl = lenStr.toInt();
          // Skip the trailing "+HTTPREAD: 0" terminator line if it shows up
          // as the header; only accept a positive data length here.
          if (rl > 0) {
            reportedLen = rl;
            gotHeader = true;
            break;
          }
        }
      }
      yield();
    }

    if (!gotHeader || reportedLen <= 0) {
      Serial.printf("Failed to get HTTPREAD data header at %ld. Aborting.\n", currentPosition);
      Update.abort();
      sendAT("AT+HTTPTERM", TO_CELL);
      return;
    }

    // Read exactly reportedLen bytes — whatever the modem actually gives.
    uint8_t* buf = (uint8_t*)malloc(reportedLen);
    if (buf == nullptr) {
      Serial.println("malloc failed. Aborting.");
      Update.abort();
      sendAT("AT+HTTPTERM", TO_CELL);
      return;
    }

    int bytesRead = 0;
    unsigned long readDeadline = millis() + 10000;
    while (bytesRead < reportedLen && millis() < readDeadline) {
      if (Serial2.available() > 0) {
        buf[bytesRead++] = Serial2.read();
      }
      yield();
    }

    if (bytesRead != reportedLen) {
      Serial.printf("Short read: got %d of %d at %ld. Aborting.\n",
                    bytesRead, reportedLen, currentPosition);
      free(buf);
      Update.abort();
      sendAT("AT+HTTPTERM", TO_CELL);
      return;
    }

    size_t written = Update.write(buf, reportedLen);
    free(buf);

    if (written != (size_t)reportedLen) {
      Serial.printf("Update.write wrote %d of %d. Aborting.\n", (int)written, reportedLen);
      Update.abort();
      sendAT("AT+HTTPTERM", TO_CELL);
      return;
    }

    currentPosition += reportedLen;

    // Consume the trailing OK after the data block.
    unsigned long okTimeout = millis();
    String tail = "";
    while (millis() - okTimeout < 2000) {
      if (Serial2.available()) {
        char c = Serial2.read();
        tail += c;
        if (tail.indexOf("OK") != -1) break;
      }
      yield();
    }

    if (currentPosition % 20480 < 1024) {
      Serial.printf("Progress: %.1f%% (%ld / %d)\n",
                    ((float)currentPosition / firmwareSize) * 100, currentPosition, firmwareSize);
    }
  }

  Serial.printf("Final position: %ld / %d\n", currentPosition, firmwareSize);

  if (Update.end() && Update.isFinished()) {
    Serial.println("\n[SUCCESS] Firmware verified! Rebooting...");
    sendAT("AT+HTTPTERM", TO_CELL);
    delay(500);
    ESP.restart();
  } else {
    Serial.printf("\n[FAILURE] Update halted. Error: %s\n", Update.errorString());
    sendAT("AT+HTTPTERM", TO_CELL);
  }
}

void doOTA() {
  int remoteVersion = checkVersion();
  
  if (remoteVersion <= 0) {
    return;
  }
  
  Serial.printf("Current Version: %d | Server Version: %d\n", LOCAL_VERSION, remoteVersion);
  
  if (remoteVersion > LOCAL_VERSION) {
    downloadNewVersion();
  }
  
  sendAT("AT+HTTPTERM", TO_CELL);
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

  sendAT("AT+CGSOCKCONT=1,\"IP\",\"" + APN + "\"", TO_CELL);
  
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
    nmeaSentence = "$GNRMC,,V,,,,,,,,,,N,V"; 
  }
  else {
    String lat = ddToDegMin(gpsFields[5], true);
    String lon = ddToDegMin(gpsFields[7], false);
    nmeaSentence = "$GPRMC," + gpsFields[10] + ",A," + lat + "," + gpsFields[6] + "," + 
                  lon + "," + gpsFields[8] + "," + gpsFields[12] + "," + 
                  gpsFields[13] + "," + gpsFields[9] + ",,,A,V";
  }
  Serial.println(nmeaSentence);
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