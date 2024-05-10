#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
HardwareSerial serialCom(1); // define a Serial for UART1
const int SerialRX = 4;
const int SerialTX = 5;
typedef struct struct_message {
    char data[250];
} struct_message;
typedef struct struct_message1 {
    uint8_t data[250];
} struct_message1;
// Create a struct_message called BME280Readings to hold sensor readings
struct_message SendData;
struct_message RecData;
//struct_message1 RecData;
int intr =0;

char StartChar='!',StopChar='%';
char ReceivedData[250];
int indexRec=0;
bool isComm = false,PreferedData=false;
// Global copy of slave
#define NUMSLAVES 2
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int SlaveCnt = 0;

#define CHANNEL 1
#define PRINTSCANRESULTS 0

void Serialler(){
  int len = serialCom.available();
  while (len--) {
    char inByte = (char)serialCom.read();
    //isComm = true;
    if(inByte=='!')
    {
      isComm = true;
    }else if(inByte == '%'){
      ReceivedData[indexRec]=inByte;
      isComm = false;
      indexRec = 0;
      strcpy(SendData.data,ReceivedData);
      Serial.println(ReceivedData);
      PreferedData=true;
    }    
    if(isComm){
      ReceivedData[indexRec]=inByte;
      ReceivedData[indexRec+1]=' ';
      indexRec++;
      //Serial.println(ReceivedData);
    }    
    /*if(indexRec==250){
      indexRec = 0;
      PreferedData = true;
      strcpy(SendData.data,ReceivedData);
      Serial.println(ReceivedData);
      //serialCom.flush();
      //serialCom.end();
    }*/
  }
}
// Init ESP Now with fallback

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset slaves
  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("SENDATA") == 0) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        /*if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slaves[SlaveCnt].peer_addr[ii] = (uint8_t) mac[ii];
          }
        }*/
        memcpy(slaves[SlaveCnt].peer_addr, broadcastAddress, 6);
        slaves[SlaveCnt].channel = CHANNEL; // pick a channel
        slaves[SlaveCnt].encrypt = 0; // no encryption
        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) {
    Serial.print(SlaveCnt); Serial.println(" Slave(s) found, processing..");
  } else {
    Serial.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      bool exists = esp_now_is_peer_exist(slaves[i].peer_addr);
      if (!exists) {
        char macStr[18];
        uint8_t*mac_addr = slaves[i].peer_addr;
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        Serial.print("MAC ADDR CON: "); Serial.println(macStr);
        esp_err_t addStatus = esp_now_add_peer(&slaves[i]);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
}


// send data
void sendData() {
  intr++;
  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = slaves[i].peer_addr;
    if (i == 0) { // print only for first slave
      Serial.print("Sending: ");
      Serial.println(SendData.data);
    }
    //esp_err_t result = esp_now_send(peer_addr, (uint8_t*)&SendData, sizeof(SendData));
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&SendData, sizeof(SendData));
    
    Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }
    delay(100);
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * info, const uint8_t *data, int data_len) {
  char macStr[18];
  memcpy(&RecData, data, sizeof(RecData));
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           info[0], info[1], info[2],
           info[3], info[4], info[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println((char*)RecData.data);
  for(int i=0;i<data_len;i++)Serial.println(RecData.data[i],HEX);
  Serial.println("");
}
void setup() {
  Serial.begin(115200);
  serialCom.begin(115200,SERIAL_8N1,SerialRX,SerialTX);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  WiFi.begin("SENDATA:","123456789");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  ScanForSlave();
  pinMode(35,OUTPUT);
}

void loop() {
  //digitalWrite(35,HIGH);
  //delay(1000);
  //digitalWrite(35,LOW);
  //delay(1000);
  
  Serialler();
  //
  //serialCom.println(intr);
  if (SlaveCnt > 0) {
    manageSlave();
    //SendData.data = ReceivedData;
    //String char16t = String(intr)+"!gfdgdfgdgdfgdfgfdggdgfdgdfgdfgdgdfgdfgdgfdfggfdgdfgdgdfgdfgfdgfdgdgfdgdfgdfgdgdfgdfgdgfdfggfdgdfgdgdfgdfgfdgfdgdgfd5gdfgdfg45454d445454545454gdfgdfgdgfdfggfdgdfgdgdfgdfgfdgfdgdgfdgdfgdfgdgdfgdfgdgfdfggfdgdfgdgdfgdfgfdgfdgdgfdgdfgdfgdgdfgdfgdgfdf#";
    intr++;
    //strcpy(SendData.data,char16t.c_str());
    if(PreferedData){
      sendData();
      PreferedData=false;
    }
  } else {
  }
}
