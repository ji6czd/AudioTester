#include <Arduino.h>
#include <Wire.h>
#include <cstdint>
#include <ios>
#include <variant>

#define SPK_PIN D10
#define INA226_ADDR 0x40

#define INA226REG_SHUNT_V 0x01
#define INA226REG_BUS_V 0x02
#define SHUNT_R 25 // 0.025Orm

bool INA226_isExist() {
	Wire.beginTransmission(INA226_ADDR);
	return (Wire.endTransmission() == 0);
}

void INA226_write(uint8_t reg, uint16_t val) {
	Wire.beginTransmission(INA226_ADDR);
	Wire.write(reg);
	Wire.write(val >> 8);
	Wire.write(val & 0x00ff);
	Wire.endTransmission();
}

uint16_t INA226_read(uint8_t reg) {
	uint16_t ret = 0;
	// リクエストするレジスタをコール
	Wire.beginTransmission(INA226_ADDR);
	Wire.write(reg);
	Wire.endTransmission();
	// 2バイトリクエスト
	Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2);
	// 2バイト取り込み
	while (Wire.available()) {
		//	初回は0なので下位ビットに埋まる
		//	2回目は下位ビットを上位にずらして、新しく来たものを下位ビットに埋める
		ret = (ret << 8) | Wire.read();
	}
	return ret;
}

bool isExistINA226 = false;

void setup() {
  pinMode(SPK_PIN, OUTPUT);
  Serial.begin(115200);
  delay(1000); // for RP2040 serial
  Wire.setSCL(SCL);
  Wire.setSDA(SDA);
  Wire.setClock(400*1000);
  Wire.begin();
  delay(100);
  if (INA226_isExist()) {
    Serial.println("INA226 Ammeter");
    isExistINA226 = true;
    tone(SPK_PIN, 2000, 300);
  } else {
    Serial.println("INA226 not found");
    while(1) {
      tone(SPK_PIN, 1000, 300);
      delay(1000);
    }
  }
  INA226_write(0x00, 0b0100010000000111);
  delay(500);
}

#define DELAY 3 // 測定間隔
#define TH 0.2 // 測定限界

void printBar(int val) {
  constexpr uint8_t displayMultiprex=5;
  static uint8_t lastLen=0;
  val = val / displayMultiprex; // 点字ディスプレイ１セルで２目盛りにしたいので*2
  String bar;
  for(int i = 0; i < val/2; i++) {
    bar.concat("=");
  }
  if (val%2) {
    bar.concat("l");
  }
  uint8_t curLen = bar.length();
  for (int i=0; i < lastLen - curLen; i++) {
    bar.concat(" ");
  }
  lastLen = curLen;
  Serial.printf("%s\r", bar.c_str());
}

void loop() {
  unsigned long t = millis();

  float amp = (int16_t)INA226_read(INA226REG_SHUNT_V) * 2.5f / SHUNT_R;
  float volt = INA226_read(INA226REG_BUS_V) * 1.25f / 1000.f;
  static float pamp = 0.0f;
  static float pvolt = 0.0f;
  static uint ampHoldTime = 0;
  static uint voltHoldTime = 0;
  if (amp > TH && amp > pamp) {
    pamp = amp;
    ampHoldTime = millis();
  }
  if(ampHoldTime && millis() > ampHoldTime+1000) {
    pamp = 0;
    ampHoldTime = 0;
  }
  if (volt > pvolt) {
    pvolt = volt;
    voltHoldTime = millis();
  }
  if(voltHoldTime && millis() > voltHoldTime+1000) {
    pvolt = 0;
    voltHoldTime = 0;
  }
    
//		float amp = (float)random(1000) / 10.f;
//		float volt = (float)random(10) / 2.f;
  uint16_t pFreq = 0;
  uint16_t freq = 500+(amp*100.0);
  Serial.printf("%.2fmA %.2fV\r", pamp, pvolt);
  // printBar(int(pamp));
  if (amp > TH && pFreq != freq) {
    // tone(SPK_PIN, freq);
    pFreq = freq;
  } else {
    noTone(SPK_PIN);
  }
  unsigned long elapse = millis() - t;
  if (elapse < DELAY) {
    // Serial.printf("%05d\r", elapse);
    delay(DELAY - elapse);
  }
}

