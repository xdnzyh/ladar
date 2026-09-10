#ifndef CAR_MOTION_RESULT_H
#define CAR_MOTION_RESULT_H
#include <Print.h>

// One immutable terminal record; queries never invoke motor-control functions.
namespace CarResult {
struct Snapshot {
  uint32_t tag;
  long request;
  long wheel[4];
  float brake, enc;
  char mode;
  uint8_t reason;
  bool mm, valid;
};
static Snapshot last;
static uint32_t armedTag, activeTag;
static bool armed = false, inFlight = false;
static void reset() { armed=false; inFlight=false; last.valid=false; }
static void arm(uint32_t tag) { armedTag=tag; armed=true; }
static void hex(Print &out, uint32_t value, uint8_t digits) {
  while(digits) {
    uint8_t nibble=(value >> ((--digits)*4)) & 15;
    out.write((uint8_t)(nibble<10 ? '0'+nibble : 'A'+nibble-10));
  }
}
class CheckedPrint : public Print {
public:
  uint16_t crc;
  CheckedPrint(): crc(0xFFFF) {}
  size_t write(uint8_t value) {
    crc=CarConfig::crcByte(crc,value);
    return Serial.write(value);
  }
  void finish() {
    Serial.write(','); hex(Serial,crc,4); Serial.println();
  }
};
static void reply(uint32_t tag, char status=0) {
  CheckedPrint out;
  out.print(F("@RESULT,")); hex(out,tag,8); out.write(',');
  if(status) out.write(status);
  else {
    out.write(last.mode); out.write(','); out.print(last.reason);
    out.write(','); out.print(last.request);
    out.print(last.mm ? F(",MM,") : F(",CNT,"));
    out.print(last.brake,2); out.write(','); out.print(last.enc,2);
    for(uint8_t i=0;i<4;i++) {out.write(',');out.print(last.wheel[i]);}
  }
  out.finish();
}
// Validate the entire command before stripping the checksum and final tag.
static bool checked(char *line, uint32_t &tag, bool move) {
  char *check=strrchr(line,',');
  uint32_t expected;
  if(!check || !CarConfig::hexValue(check+1,4,expected)
      || CarConfig::textCrc(line,(uint8_t)(check-line))!=expected) return false;
  *check=0;
  char *id=strrchr(line,',');
  if(!id || !CarConfig::hexValue(id+1,8,tag)) return false;
  if(!move && id!=line+7) return false;
  if(move) *id=0;
  return true;
}
static bool query(char *line) {
  if(strncmp(line,"@RESULT,",8)!=0) return false;
  uint32_t tag;
  if(!checked(line,tag,false)) { Serial.println(F("@ERR,BAD_CMD")); return true; }
  if(inFlight && tag==activeTag) reply(tag,'B');
  else if(last.valid && tag==last.tag) reply(tag);
  else reply(tag,'N');
  return true;
}
}
#endif
