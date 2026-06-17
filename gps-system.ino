//import libraries
#include  <Arduino.h>
#include <Update.h>
#include <Preferences.h>

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
String DEVICE_ID = "000";

//CHANGE THIS EVERY UPDATE!!
const int LOCAL_VERSION = 0;

//https://dl.dropboxusercontent.com/scl/fi/.../<filename>?rlkey=...&st=...&dl=1
String version_url = "https://dl.dropboxusercontent.com/scl/fi/r0rhfi73h7iciwrlumrls/version.txt?rlkey=ddjovrejh5wg06vr26z63uvmu&st=9zyuv7eo&dl=1";
String firmware_url = "https://dl.dropboxusercontent.com/scl/fi/1x22iaxccj0rqc2e2tuvf/test-gps-code.ino.bin?rlkey=eh8112jyw16821s8k884o9d76&st=u3ulhjbe&dl=1";

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

const String UPDATE_TIME_UTC = "000000.00"; //hhmmss.ss, set to 2am COT

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
bool otaExecutedToday = false; // Prevents re-trigger loop
String dateLastUpdated = "000000"; // ddmmyy, change this when you update
int serverVersion = 0;
int firmwareSize = 0; // Will be parsed dynamically from version.txt
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

  // // Open namespace "storage" in Read/Write mode (false)
  // preferences.begin("storage", false);

  // // Try to read the key "dev_id". If it doesn't exist, fall back to "189", or whatever
  // // device ID you need
  // DEVICE_ID = preferences.getString("dev_id", "189");
  
  // // If it was empty/new, save it explicitly so it's permanently there
  // if (DEVICE_ID == "189" && !preferences.isKey("dev_id")) {
  //   preferences.putString("dev_id", "189");
  // }
  
  // Serial.print("\n[SYSTEM] Persistent Hardware Device ID Loaded: ");
  // Serial.println(DEVICE_ID);
  
  // // Close the preferences instance until needed again
  // preferences.end();

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
  delay(10000);
  String curTime = getTime();
  Serial.println("curTime"+curTime);
  if (curTime >= UPDATE_TIME_UTC) {
    sendAT("AT+CIPCLOSE=0", TO_LOCAL);
    String curDate = getDate();
    Serial.println("lastUpdate:"+dateLastUpdated);
    Serial.println("curDate:"+curDate);
    if (dateLastUpdated != curDate) {
      downloadNewVersion();
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

    // 3. Configure internal SSL Engine for Dropbox (Context 0)
    sendAT("AT+CSSLCFG=\"sslversion\",0,4", TO_LOCAL); // Force TLS 1.2 explicitly
    sendAT("AT+CSSLCFG=\"authmode\",0,0", TO_LOCAL);   // Bypass strict certificate authority checks
    sendAT("AT+CSSLCFG=\"enableSNI\",0,1", TO_LOCAL);  // Enable Server Name Indication
    
    // NEW CRITICAL LINE FOR DROPBOX SECURE HANDSHAKES:
    sendAT("AT+CSSLCFG=\"ciphersuites\",0,0xFFFF", TO_LOCAL); // Enable ALL cipher suites supported by the modem
    
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
  Serial.println("\nChecking for available firmware updates...");
  
  // 1. Fetch the exact metadata file size dynamically
  long metadataFileSize = httpGET(version_url); 
  if (metadataFileSize <= 0) {
    addError(ERR_VERSION_FILE_SIZE);
    Serial.println("[ERROR] Failed to obtain valid version.txt size from server.");
    return -1;
  }
  
  int tries = 0;
  while (tries < 3) {
    // 2. Command the modem to pull down the ENTIRE metadata block payload safely
    String response = sendATHTTPREAD(String(metadataFileSize), TO_CELL);
    Serial.println("READRESPONSE"+response);

    // 4. Split the text payload cleanly by its newline delimiter
    int newlineIdx = response.indexOf("\n");
    if (newlineIdx != -1) {
      String verStr = response.substring(0, newlineIdx);
      String sizeStr = response.substring(newlineIdx + 1);
      verStr.trim();
      sizeStr.trim();
      
      serverVersion = verStr.toInt();
      firmwareSize = sizeStr.toInt();
      
      Serial.printf("[SUCCESS] Metadata Sync -> Target Version: %d | Expected Size: %ld bytes\n", serverVersion, firmwareSize);
      return serverVersion;
    } else {
      Serial.println("[ERROR] Failed to parse version information.");
      stop();
    }
  }
  return -1;
}

void downloadNewVersion() {
  checkVersion();
  if (firmwareSize <= 0) {
    Serial.println("Error: Valid firmware size not found. Aborting.");
    return;
  }

  if (!Update.begin(firmwareSize)) {
    Serial.printf("CRITICAL ERROR: File size (%ld bytes) exceeds available space!\n", firmwareSize);
    return;
  }

  Serial.printf("Initializing Zero-Drift Stream Parser for %ld bytes...\n", firmwareSize);
  long currentPosition = 0;
  const int chunkSize = 1024; 
  bool downloadComplete = false;

  while (currentPosition < firmwareSize && !downloadComplete) {
    sendAT("AT+HTTPTERM", TO_CELL);
    delay(100);
    sendAT("AT+HTTPINIT", TO_CELL);
    delay(100);

    sendAT("AT+CSSLCFG=\"sslversion\",0,4", TO_LOCAL); 
    sendAT("AT+CSSLCFG=\"authmode\",0,0", TO_LOCAL);   
    sendAT("AT+CSSLCFG=\"enableSNI\",0,1", TO_LOCAL);  
    sendAT("AT+CSSLCFG=\"ciphersuites\",0,0xFFFF", TO_LOCAL);
    sendAT("AT+HTTPPARA=\"URL\",\"" + firmware_url + "\"", TO_CELL);

    long endPosition = currentPosition + chunkSize - 1;
    if (endPosition >= firmwareSize) {
      endPosition = firmwareSize - 1;
    }
    long targetBytesThisChunk = (endPosition - currentPosition) + 1;
    
    String rangeHeader = "Range: bytes=" + String(currentPosition) + "-" + String(endPosition);
    sendAT("AT+HTTPPARA=\"USERDATA\",\"" + rangeHeader + "\"", TO_CELL);

    Serial2.println("AT+HTTPACTION=0");
    
    String actionResponse = "";
    unsigned long actionStart = millis();
    int receivedBytes = 0;
    while (millis() - actionStart < 20000) {
      while (Serial2.available() > 0) {
        char c = Serial2.read();
        Serial.write(c);
        actionResponse += c;
        if (actionResponse.indexOf("+HTTPACTION:") != -1 && actionResponse.endsWith("\n")) break;
      }
      if (actionResponse.indexOf("+HTTPACTION:") != -1) break;
      yield();
    }

    int actionIdx = actionResponse.indexOf("+HTTPACTION:");
    if (actionIdx != -1) {
      int firstComma = actionResponse.indexOf(",", actionIdx);
      int secondComma = actionResponse.indexOf(",", firstComma + 1);
      String status = actionResponse.substring(firstComma + 1, secondComma);
      status.trim();

      if (status == "200" || status == "206") {
        String sizeStr = actionResponse.substring(secondComma + 1);
        sizeStr.trim();
        receivedBytes = sizeStr.toInt();
      } else {
        Serial.printf("\nNetwork error at position %ld. Status: %s\n", currentPosition, status.c_str());
        Update.abort();
        return;
      }
    } else {
      Serial.println("No HTTPACTION response received.");
      Update.abort();
      return;
    }

    if (receivedBytes <= 0) {
      downloadComplete = true;
      break;
    }

    // Use sendATHTTPREAD to fetch the payload
    String payload = sendATHTTPREAD(String(targetBytesThisChunk), TO_CELL);
    if (payload.length() == 0) {
      Serial.println("Error: HTTPREAD returned no data.");
      Update.abort();
      return;
    }

    // If module returned fewer bytes than requested, accept actual length
    int bytesToWrite = payload.length();
    if (bytesToWrite > (firmwareSize - currentPosition)) {
      bytesToWrite = firmwareSize - currentPosition;
    }

    // Write payload into flash (safely, byte-by-byte)
    int bytesWritten = 0;
    for (int i = 0; i < bytesToWrite; i++) {
      uint8_t b = (uint8_t)payload.charAt(i);
      Update.write(&b, 1);
      bytesWritten++;
      currentPosition++;
    }

    Serial.printf("Verified Download Progress: %.2f%% (%ld / %ld bytes flashed)\n",
                  ((float)currentPosition / firmwareSize) * 100, currentPosition, firmwareSize);
    
    if (currentPosition >= firmwareSize) {
      downloadComplete = true;
    }
  }

  // Finalize update
  if (Update.end() && Update.isFinished()) {
    Serial.println("\n[SUCCESS] Binary matches signature constraints! Rebooting...");
    sendAT("AT+HTTPTERM", TO_CELL);
    delay(500);
    ESP.restart();
  } else {
    Serial.printf("\n[FAILURE] Update verification failed. Core reason: %s\n", Update.errorString());
    sendAT("AT+HTTPTERM", TO_CELL);
  }
}


//helper functions

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
  return "00";
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
  bool negative = dd < 0.0;
  if (negative) dd = -dd;

  int deg = (int)dd;
  double frac = dd - (double)deg;
  double minutes = frac * 60.0;

  // handle rounding that would push minutes to 60.0000
  long minFrac = (long)round((minutes - floor(minutes)) * 10000.0);
  int minWhole = (int)floor(minutes);
  if (minFrac >= 10000) {
    minFrac = 0;
    minWhole += 1;
    if (minWhole >= 60) {
      minWhole = 0;
      deg += 1;
    }
  }

  char buf[20];
  if (isLatitude) {
    // degrees: 2 digits, minutes: 2 digits + '.' + 4 decimals
    sprintf(buf, "%02d%02d.%04ld", deg, minWhole, minFrac);
  } else {
    // degrees: 3 digits
    sprintf(buf, "%03d%02d.%04ld", deg, minWhole, minFrac);
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

// Read the response for an AT+HTTPREAD command from Serial2 and return the raw data payload.
// Returns an empty string on timeout or error.
String sendATHTTPREAD(String numBytesToRead, unsigned long timeout) {
  while (Serial2.available() > 0) {
      Serial.write(Serial2.read());
  }
  // send the AT message
  Serial2.println("AT+HTTPREAD=0,"+numBytesToRead);
  
  // read the response
  unsigned long start = millis();
  String curLine = "";
  bool headerFound = false;
  int dataLen = -1;
  String payload = "";
  int bytesCollected = 0;

  while (millis() - start < timeout) {
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      Serial.write(c); // mirror for debug / logging

      if (!headerFound) {
        // Accumulate a line until newline to look for the +HTTPREAD: header or ERROR
        if (c == '\n') {
          curLine.trim();
          if (curLine.length() > 0) {
            if (curLine.indexOf("ERROR") != -1) {
              return ""; // read failed
            }
            int idx = curLine.indexOf("+HTTPREAD:");
            if (idx != -1) {
              // parse length
              String lenStr = curLine.substring(idx + 10);
              lenStr.trim();
              dataLen = lenStr.toInt();
              headerFound = true;
              // if there's no data to read, return empty payload
              if (dataLen <= 0) return "";
            }
          }
          curLine = "";
        } else if (c != '\r') {
          curLine += c;
        }
      } else {
        // After header found, read exactly dataLen bytes (count raw bytes)
        payload += c;
        bytesCollected++;
        if (bytesCollected >= dataLen) {
          // Return exactly the data block (first dataLen bytes)
          if (payload.length() > dataLen) {
            return payload.substring(0, dataLen);
          }
          return payload;
        }
      }
    }
    yield();
  }

  // timeout
  return "";
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
  //may not be needed
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