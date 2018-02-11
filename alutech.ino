#include <EEPROM.h>
#include <avr/wdt.h>

//#define DEBUG

#define PIN_SDA 2
#define PIN_SCL 3
#define PIN_LEARN 4
#define PIN_LEFT 5
#define PIN_RIGHT 6

#define PIN_RELAY_INDOOR 9
#define PIN_RELAY_LEFT 10
#define PIN_RELAY_OUTDOOR 11
#define PIN_RELAY_RIGHT 12
#define PIN_LED 13

#define CNT_SHORT 0
#define CNT_LONG 6
#define CNT_IGNORE 100

#define MAX_IDS 10

#define CMD_TOP 0x24
#define CMD_LEFT 0x44
#define CMD_BOTTOM 0x84
#define CMD_RIGHT 0xC4

#define DEBOUNCE_TIME 100
#define DOUBLE_CLICK_TIME 1000

volatile byte buf[10];
volatile byte data = 0;
volatile byte cnt = 0;
volatile byte idx = 0;
volatile byte state = 0; //0 = undefined, 1 = start, 2 = stop, 3,4,5 = error

uint16_t *ids = NULL;
byte ids_cnt;

byte repeat;
byte repeat_cnt = 0;

byte press_left = 0;
byte press_right = 0;

unsigned long time_indoor = -1;
unsigned long time_left = -1;
unsigned long time_right = -1;

void setup() {
  pinMode(PIN_RELAY_INDOOR, OUTPUT);
  pinMode(PIN_RELAY_LEFT, OUTPUT);
  pinMode(PIN_RELAY_OUTDOOR, OUTPUT);
  pinMode(PIN_RELAY_RIGHT, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  pinMode(PIN_LEARN, INPUT_PULLUP);
  pinMode(PIN_LEFT, INPUT_PULLUP);
  pinMode(PIN_RIGHT, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("alutech advanced control");

  if(!digitalRead(PIN_LEARN)){
    Serial.println("reset all ids");
    EEPROM.update(0, 0);
  }

  if(readIds()){
    Serial.println("error reading ids");
  }
  
  attachInterrupt(digitalPinToInterrupt(PIN_SDA), on_sda, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SCL), on_scl, RISING);

  wdt_enable(WDTO_8S);
}

void on_sda() {
  if(digitalRead(PIN_SCL)){
    if(digitalRead(PIN_SDA)){
      state = state == 1 ? 2 : 4;
    }else{
      state = state ? 3 : 1;
    }
  }
}

void on_scl() {
  if(state == 1){
    if(idx > 7){
      if(digitalRead(PIN_SDA)){
        state = 5;
      }else{
        if(cnt < 10){
          buf[cnt] = data;
          data = 0;
          idx = 0;
          cnt++;
        }
      }
    }else{
      data<<= 1;
      if(digitalRead(PIN_SDA)){
        data++;
      }
      idx++;
    }
  }
}

int readIds(){
  ids_cnt = EEPROM.read(0);
  if(ids_cnt > MAX_IDS){
    Serial.println("warning reading ids, max count");
    ids_cnt = MAX_IDS;
  }
  
  ids = malloc(ids_cnt * sizeof(uint16_t));
  if(!ids){
    return -1;
  }

  for(byte i=0,addr; i < ids_cnt && i < MAX_IDS; i++){
    addr = i*sizeof(uint16_t);
    ((uint8_t*)ids)[addr] = EEPROM.read(addr+1);
    ((uint8_t*)ids)[addr+1] = EEPROM.read(addr+2);
    Serial.print("read id #");
    Serial.print(i+1);
    Serial.print(": 0x");
    Serial.println(ids[i], HEX);
  }

  return 0;
}

int writeId(uint16_t id){
  if(ids_cnt > MAX_IDS){
    Serial.println("error writing id, max count");
    return -1;
  }

  Serial.print("writing id: 0x");
  Serial.println(id, HEX);
  
  byte addr = ids_cnt*sizeof(uint16_t);
  EEPROM.update(addr+2, id >> 8);
  EEPROM.update(addr+1, id & 0xFF);
  EEPROM.update(0, ids_cnt+1);

  free(ids);
  ids = NULL;
  if(readIds()){
    Serial.println("error reading ids after write");
  }

  return 0;
}

int checkId(uint16_t id){
  for(byte i=0; i < ids_cnt; i++)
    if(ids[i] == id)
      return 0;

  return -1;
}

int remoteShort(byte cmd){
  if(cmd == CMD_LEFT){
    togglePin(PIN_LED);
    togglePin(PIN_RELAY_LEFT);
    Serial.println("cmd left");
    return 0;
  }
  if(cmd == CMD_RIGHT){
    togglePin(PIN_LED);
    togglePin(PIN_RELAY_RIGHT);
    Serial.println("cmd right");
    return 0;
  }
  if(cmd == CMD_BOTTOM){
    if(time_indoor == -1){
      time_indoor = millis();
    }else if(millis() - time_indoor < DOUBLE_CLICK_TIME){
      time_indoor = -1;
      togglePin(PIN_LED);
      togglePin(PIN_RELAY_INDOOR);
      Serial.println("cmd bottom");
      return 0;
    }else{
      time_indoor = millis();
    }
  }
  return -1;
}

int remoteLong(byte cmd){
  if(cmd == CMD_TOP){
    togglePin(PIN_LED);
    PORTB&= ~((1 << (PIN_RELAY_INDOOR-8)) | (1 << (PIN_RELAY_LEFT-8)) | (1 << (PIN_RELAY_OUTDOOR-8)) | (1 << (PIN_RELAY_RIGHT-8)));
    Serial.println("cmd top long");
    return 0;
  }
  if(cmd == CMD_BOTTOM){
    togglePin(PIN_LED);
    togglePin(PIN_RELAY_OUTDOOR);
    Serial.println("cmd bottom long");
    return 0;
  }
  return -1;
}

void loop() {
  if(state > 1){

#ifdef DEBUG
    Serial.print(state);
    Serial.print(": data =");
    for(byte i = 0; i < cnt; i++){
      Serial.print(" 0x");
      Serial.print(buf[i], HEX);
    }
    Serial.println(";");
#endif

    if(state == 2 && buf[0] == 0x02
    && buf[1] == 0x04 && buf[2] == buf[8]
    && buf[3] == buf[4] && buf[7] == 0x77)
      processPacket(*((uint16_t*)&buf[5]), buf[2], buf[3]);

    data = 0;
    idx = 0;
    cnt = 0;
    state = 0;
  }

  if(!digitalRead(PIN_LEFT)){
    if(time_left == -1){
      time_left = millis();
      press_left = 1;
    }else if(press_left && millis() - time_left > DEBOUNCE_TIME){
      press_left = 0;
      remoteShort(CMD_LEFT);
    }
  }else if(time_left != -1 && millis() - time_left > DEBOUNCE_TIME){
    time_left = -1;
  }

  if(!digitalRead(PIN_RIGHT)){
    if(time_right == -1){
      time_right = millis();
      press_right = 1;
    }else if(press_right && millis() - time_right > DEBOUNCE_TIME){
      press_right = 0;
      remoteShort(CMD_RIGHT);
    }
  }else if(time_right != -1 && millis() - time_right > DEBOUNCE_TIME){
    time_right = -1;
  }

  wdt_reset();
}

void processPacket(uint16_t id, byte cmd, byte rep){
  if(checkId(id)){
    if(!digitalRead(PIN_LEARN)){
      writeId(id);
    }
    return;
  }

  if(repeat != rep){
    repeat = rep;
    repeat_cnt = 0;
    return;
  }
  
  if(repeat_cnt == CNT_IGNORE){
    return;
  }

  if((repeat_cnt == CNT_SHORT && !remoteShort(cmd))
  || (repeat_cnt == CNT_LONG && !remoteLong(cmd))){
    repeat_cnt = CNT_IGNORE;
  }else{
    repeat_cnt++;
  }
}

void togglePin(byte pin){
  PORTB^= 1 << (pin-8);
}

