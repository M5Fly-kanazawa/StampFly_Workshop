//AtomFly2 Telemetry
#include <Arduino.h>
#include <M5Atom.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define CHANNEL 14

uint8_t Channel = CHANNEL;
uint8_t data[1];
esp_now_peer_info_t peerInfo;

void dataRecv(const uint8_t *addr, const uint8_t *data, int datasize) {
  //データ受信時に実行したい内容をここに書く。
  float a;
  uint8_t *dummy;
  uint8_t offset = 2;
  dummy=(uint8_t*)&a;
  
  dummy[0]=data[0];
  dummy[1]=data[1];
  if (dummy[0]==0xF4)return;
  if ((dummy[0]==99)&&(dummy[1]==99))Serial.printf("#PID Gain P Ti Td Eta ");
  for (uint8_t i=0; i<((datasize-offset)/4); i++)
  {
    dummy[0]=data[i*4 + 0 + offset];
    dummy[1]=data[i*4 + 1 + offset];
    dummy[2]=data[i*4 + 2 + offset];
    dummy[3]=data[i*4 + 3 + offset];
    Serial.printf(">data:%9.4f ", a);
  }
  Serial.printf("\n");
}

void change_channel(uint8_t ch)
{
  peerInfo.channel = ch;
  if (esp_now_mod_peer(&peerInfo)!=ESP_OK)
  {
        Serial.println("Failed to modify peer");
        return;
  }
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  WiFi.mode(WIFI_STA); //Wi-Fi機能をステーションモードで起動
  WiFi.disconnect(); //初期化前にWi-Fi接続を切断
  Serial.printf("MAC ADDRESS: %s\r\n", (WiFi.macAddress()).c_str());
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  //ペアリング
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
        Serial.println("Failed to add peer");
        return;
  }
  esp_now_register_recv_cb(dataRecv);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

}

void loop() {
  // put your main code here, to run repeatedly:
  M5.update();
  if (M5.Btn.wasPressed())
  {
    Channel++;
    if (Channel==15)Channel=1;
    change_channel(Channel);
    Serial.printf("Channel %02d\n\r", Channel);
  }
}