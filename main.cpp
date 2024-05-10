#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#define CHANNEL 1
#define DATA_MAX_LEN 250
char StartChar='!',StopChar='%';
char ReceivedData[DATA_MAX_LEN];
uint8_t ReceivedData8[DATA_MAX_LEN];
int indexRec=0,crnt_indx=0;
bool isComm = false,PreferedData=false;
#include <HardwareSerial.h>
HardwareSerial serialCom(1); // define a Serial for UART1
const int SerialRX = 4;
const int SerialTX = 5;
// Init ESP Now with fallback
void InitESPNow() {
  //WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char* Prefix = "SENDATA:";
  String Mac = WiFi.macAddress();
  const char* SSID = Prefix;
  const char* Password = "123456789";
  bool result = WiFi.softAP(SSID, Password);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}
void manageSlave() {    
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = CHANNEL;  // use the current channel
  peerInfo.encrypt = false;
  esp_err_t addPeerResult;
  // Check if peer is already added
  if (!esp_now_is_peer_exist(broadcastAddress)) {
    addPeerResult= esp_now_add_peer(&peerInfo);
  }   
  if (addPeerResult == ESP_OK) {
    Serial.println("Peer added successfully");
  } 
}
typedef struct struct_message {
    char data[DATA_MAX_LEN];
} struct_message;
struct_message SendData;
void sendData() {
    //for(int o=0;o<8;o++)Serial.println(SendData.data[o],HEX);
    Serial.println(SendData.data);
    //esp_err_t result = esp_now_send(peer_addr, (uint8_t*)&SendData, sizeof(SendData));
    //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&ReceivedData, sizeof(ReceivedData));
    //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&SendData, sizeof(SendData));
    //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&ReceivedData8, sizeof(ReceivedData8));
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&ReceivedData8, crnt_indx);
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

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
void Serialler(){
  int len = serialCom.available();
  int ind =0;
  crnt_indx=len;
  while (len--) {
    char inByte = (char)serialCom.read();
    ReceivedData8[ind]= (uint8_t)inByte;    
    ind++;
    //isComm = true;
    if(isComm){
      ReceivedData[indexRec]=inByte;
      indexRec++;
      //Serial.print(inByte,HEX);
    } 
    if(inByte==0x7e)
    {
      isComm = true;
      ReceivedData[indexRec]=inByte;
      indexRec++;
      Serial.print("Start Symbol: ");
      Serial.println(inByte,HEX);
    }else if(inByte == 0xe7){
      //ReceivedData[indexRec]=inByte;                 
      for(int i=0; i<indexRec;i++)SendData.data[i]=ReceivedData[i];
      //strcpy(SendData.data,ReceivedData);
      for(int i=0; i<indexRec;i++){
        Serial.println(SendData.data[i],HEX);
      }
      Serial.print("End Symbol: ");
      Serial.println(inByte,HEX);
      Serial.println((char*)ReceivedData8);
      PreferedData=true;
      isComm = false; 
      indexRec = 0;
    }    
    
    //Serial.println((char*)ReceivedData8);
  }  
}
// Create a struct_message called BME280Readings to hold sensor readings
struct_message RecData;
void OnDataRecv(const uint8_t * info, const uint8_t *data, int data_len) {
  char macStr[18];
  memcpy(&RecData, data, sizeof(RecData));
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           info[0], info[1], info[2],
           info[3], info[4], info[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(RecData.data);
  Serial.println("");
}
void setup() {
  Serial.begin(115200);
  serialCom.begin(115200,SERIAL_8N1,SerialRX,SerialTX);
  WiFi.mode(WIFI_MODE_APSTA);
  configDeviceAP();
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  InitESPNow();
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
 // manageSlave();
}


int cnt=0;
void loop() {
  Serialler();
  cnt++;
  //serialCom.println(cnt);
  if(PreferedData){  
    manageSlave();  
    sendData();
    PreferedData=false;
  }
}
