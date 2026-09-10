#ifndef CAR_RUNTIME_CONFIG_H
#define CAR_RUNTIME_CONFIG_H
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#ifdef ARDUINO
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define PSTR(x) x
#define pgm_read_word(p) (*(p))
#define snprintf_P snprintf
#endif

namespace CarConfig {
enum {
#define CAR_PARAMETER(name, value, minimum, maximum, scale) name##_ID,
#include "runtime_parameters.h"
#undef CAR_PARAMETER
COUNT
};
static const uint16_t defaults[][3] PROGMEM = {
#define CAR_PARAMETER(name, value, minimum, maximum, scale) {value, minimum, maximum},
#include "runtime_parameters.h"
#undef CAR_PARAMETER
};
static uint16_t active[COUNT], staged[COUNT];
static bool pending = false;
static uint32_t owner = 0;
static unsigned long touched = 0;
static uint16_t crcByte(uint16_t crc, uint8_t byte) {
  crc ^= (uint16_t)byte << 8;
  for (uint8_t i=0;i<8;i++) crc = (crc & 0x8000) ? (uint16_t)((crc<<1)^0x1021) : (uint16_t)(crc<<1);
  return crc;
}
static uint16_t textCrc(const char *p, uint8_t length) {
  uint16_t crc=0xFFFF;
  while (length--) crc=crcByte(crc, (uint8_t)*p++);
  return crc;
}
static uint16_t valuesCrc(const uint16_t *v) {
  uint16_t crc=0xFFFF;
  for(uint8_t i=0;i<COUNT;i++) {
    crc=crcByte(crc,(uint8_t)v[i]); crc=crcByte(crc,(uint8_t)(v[i]>>8));
  }
  return crc;
}
static void init() {
  for(uint8_t i=0;i<COUNT;i++) active[i]=pgm_read_word(&defaults[i][0]);
  pending=false;
}
static bool editing() {
  if(pending && (unsigned long)(millis()-touched)>30000UL) pending=false;
  return pending;
}
static bool valid(const uint16_t *v) {
  for(uint8_t i=0;i<COUNT;i++)
    if(v[i]<pgm_read_word(&defaults[i][1]) || v[i]>pgm_read_word(&defaults[i][2])) return false;
  for(uint8_t p=0;p<4;p++) {
    uint8_t b=p*11;
    if(v[b]<v[b+1] || v[b+1]<v[b+2] || v[b+2]<v[b+6] || v[b+9]>v[b+10]) return false;
    for(uint8_t k=3;k<=7;k++) {
      if(k==6) continue;
      if(v[b+k]<v[b+9] || v[b+k]>v[b+10]) return false;
    }
  }
  for(uint8_t p=0;p<8;p++) {
    uint8_t b=44+p*4;
    if(v[b]>=v[b+1] || (uint32_t)v[b+2]*2>=v[b] || (uint32_t)v[b+3]*2>=v[b+1]) return false;
  }
  return v[SLOW_ZONE_PERCENT_ID]<v[MID_ZONE_PERCENT_ID]
    && v[MAX_RMEM_CORRECTION_ID]<=v[MAX_R_CORRECTION_ID];
}
static bool hexValue(const char *s, uint8_t digits, uint32_t &v) {
  if(strlen(s)!=digits) return false;
  v=0;
  while(*s) {
    char c=*s++; uint8_t n;
    if(c>='0' && c<='9') n=c-'0';
    else if(c>='A' && c<='F') n=c-'A'+10;
    else return false;
    v=(v<<4)|n;
  }
  return true;
}
static bool number(const char *s, uint16_t &v) {
  if(!*s) return false;
  uint32_t n=0;
  while(*s) {
    if(*s<'0' || *s>'9') return false;
    n=n*10+(*s++-'0');
    if(n>65535) return false;
  }
  v=(uint16_t)n; return true;
}
// Replies are protected independently, including the nonce, operation and values.
static void reply(char *body) {
  uint16_t crc=textCrc(body,(uint8_t)strlen(body));
  Serial.print(body);
  char tail[8]; snprintf_P(tail,sizeof(tail),PSTR(",%04X"),(unsigned int)crc);
  Serial.println(tail);
}
static void answer(char op, const char *tag, uint16_t value) {
  char body[40];
  snprintf_P(body,sizeof(body),PSTR("@CFG,%c,%s,%u"),op,tag,(unsigned int)value);
  reply(body);
}
static void error(const char *tag, uint16_t code) {answer('E',tag,code);}
static bool process(char *line, bool running) {
  if(strncmp(line,"@CFG,",5)!=0) return false;
  char *last=strrchr(line,',');
  uint32_t check;
  if(!last || !hexValue(last+1,4,check) || textCrc(line,(uint8_t)(last-line))!=check) {
    Serial.println(F("@ERR,BAD_CMD")); return true;
  }
  *last=0;
  char *fields[6]; uint8_t count=0;
  fields[count++]=line;
  for(char *p=line;*p;p++) if(*p==',') {
    *p=0;
    if(count>=6) {Serial.println(F("@ERR,BAD_CMD")); return true;}
    fields[count++]=p+1;
  }
  uint32_t tag;
  if(count<3 || strlen(fields[1])!=1 || !hexValue(fields[2],8,tag)) {
    Serial.println(F("@ERR,BAD_CMD")); return true;
  }
  char op=fields[1][0]; const char *nonce=fields[2];
  if(running) {error(nonce,2); return true;}
  bool open=editing();
  uint16_t arg=0,value=0;
  if(op=='I' && count==3) {
    char body[48];
    snprintf_P(body,sizeof(body),PSTR("@CFG,I,%s,1,%u,%u"),nonce,(unsigned int)COUNT,(unsigned int)valuesCrc(active));
    reply(body); return true;
  }
  if(op=='G' && count==4 && number(fields[3],arg) && arg<COUNT) {
    char body[96];
    uint8_t used=(uint8_t)snprintf_P(body,sizeof(body),PSTR("@CFG,G,%s,%u"),nonce,(unsigned int)arg);
    for(uint8_t n=0;n<8 && arg+n<COUNT;n++)
      used+=(uint8_t)snprintf_P(body+used,sizeof(body)-used,PSTR(",%u"),(unsigned int)active[arg+n]);
    reply(body); return true;
  }
  if(op=='B' && count==3) {
    if(open && owner!=tag) {error(nonce,2);return true;}
    if(!open) memcpy(staged,active,sizeof(active));
    owner=tag;pending=true;touched=millis();
    answer('B',nonce,valuesCrc(active));return true;
  }
  if(!open || owner!=tag) {error(nonce,4);return true;}
  if(op=='A' && count==3) {
    pending=false;answer('A',nonce,valuesCrc(active));return true;
  }
  if(op=='S' && count==5 && number(fields[3],arg) && arg<COUNT && number(fields[4],value)) {
    if(value<pgm_read_word(&defaults[arg][1]) || value>pgm_read_word(&defaults[arg][2])) {error(nonce,3);return true;}
    staged[arg]=value;touched=millis();
    char body[48];
    snprintf_P(body,sizeof(body),PSTR("@CFG,S,%s,%u,%u"),nonce,(unsigned int)arg,(unsigned int)value);
    reply(body);return true;
  }
  if(op=='C' && count==4 && number(fields[3],arg)) {
    if(valuesCrc(staged)!=arg) {error(nonce,7);return true;}
    if(!valid(staged)) {error(nonce,6);return true;}
    memcpy(active,staged,sizeof(active));pending=false;
    answer('C',nonce,valuesCrc(active));return true;
  }
  error(nonce,1);return true;
}
static long brakeLead(char mode,long target) {
  const char *modes="WSADQEZC";
  const char *found=strchr(modes,mode);
  if(!found || !mode) return 0;
  uint8_t b=44+(uint8_t)(found-modes)*4;
  long lo=active[b],hi=active[b+1],a=active[b+2],z=active[b+3];
  if(target<lo || target>hi) return 0;
  long span=hi-lo;
  return (a*(hi-target)+z*(target-lo)+span/2)/span;
}
}
#endif


