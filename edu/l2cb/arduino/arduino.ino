#include <EEPROM.h>


/*
 * COBS implementation
 * 
 * https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */

#include <stdint.h>
#include <stddef.h>

typedef int16_t tBufLen;
typedef uint8_t tBufByte;

/*

 */
#define MAX_PROG_BUF_SIZE 255
#define MAX_SERIAL_BUF_SIZE (MAX_PROG_BUF_SIZE + 8)

#define FACTORY_PROGRAM \
  "N0W1N1W1N2W1N3W1N4W1N5W1" \
  "O0W1O1W1O2W1O3W1O4W1O5W1" \
  "N0N1N2N3N4N5W1O0O1O2O3O4O5W1" \
  "N0N1N2N3N4N5W1O0O1O2O3O4O5W1"

/*
 * LED does not blink because of failure to add a wait before loop
 */


  
const uint8_t gEEPROMSig[] = {'E', 'C', '7', 0};
uint8_t gMsg[16] = {0};
uint8_t gProgram[MAX_PROG_BUF_SIZE];
uint8_t gSerialBuf[MAX_SERIAL_BUF_SIZE];
uint16_t gMaxProgramLen, gProgramLen, gSerialLen;

uint8_t getHwPin(uint8_t pin) {
  return pin + 8;
}

uint8_t getHwPin2(uint8_t pin) {
  return getHwPin(pin - '0');
}

uint16_t writeEEPROMSig() {
  uint16_t i;
  
  for (i = 0; gEEPROMSig[i]; i += 1) {
    EEPROM.write(i, gEEPROMSig[i]);
  }
  return (i + 1);
}

bool saveProgram(void) {
  uint8_t offset = writeEEPROMSig();
  for (int i = 0; i < gMaxProgramLen; i += 1) {
    EEPROM.write(i + offset, (i < gProgramLen) ? gProgram[i] : 0);
  }
  gMsg[0] = 'D';
  return true;
}

#define MAX_LED 6
bool setProgramA(const uint8_t *aProgram, uint16_t offset, uint16_t len, bool save) {
  if (len >= gMaxProgramLen) {
    return false;
  }
  memcpy(gProgram, aProgram + offset, len);
  gProgramLen = len;
  for (int i = 0; i < MAX_LED; i += 1) {
    digitalWrite(getHwPin(i), LOW);
  }  
  if (save) {
    return saveProgram();
  }
  return true;
}

bool loadProgram(void) {
  uint16_t offset;
  gMsg[0] = 'A';
  for (offset = 0; gEEPROMSig[offset]; offset += 1) {
    if (EEPROM.read(offset) != gEEPROMSig[offset]) {
      gMsg[0] = 'B';
      return false;
    }
  }
  uint16_t offset2;
  for (offset2 = 0; offset2 < gMaxProgramLen; offset2 += 1) {
    gProgram[offset2] = EEPROM.read(offset + offset2);
    if (!gProgram[offset2]) {
      break;
    }
  }
  gProgramLen = offset2;
  gMsg[0] = 'C';
  return true;
}



void execInstruction(uint8_t instruction, uint8_t param) {
  char msg[16] = {0};
  switch (instruction) {
    case 'n': 
    case 'N': {
      
        uint16_t pin = getHwPin2(param);
        digitalWrite(pin, HIGH);
        sprintf(msg, "N %d", pin);
      }
      break;
      
    case 'o':
    case 'O': {
        uint16_t pin = getHwPin2(param);
        digitalWrite(pin, LOW);
        sprintf(msg, "O %d", pin);
      }
      break;
      
    case 'w': 
    case 'W': {
        uint16_t d = (param - '0') * 1000;
        delay(d);
        sprintf(msg, "W %d", d);
      }
      break;
  }
  if (msg[0]) {
    sendSerialMsg(msg);
  }
}

void sendSerialMsg(const char *msg) {
  for (uint16_t i = 0; msg[i]; i += 1) {
    Serial.write(msg[i]);
  }
  Serial.write('\n');
}

void sendSerialMsg2(uint8_t *buf, uint16_t offset, uint16_t len) {
  uint16_t endOffset = offset + len;
  for (uint16_t i = offset; i < endOffset; i += 1) {
    Serial.write(buf[i]);
  }
  Serial.write('\n');
}

bool sendId() {
  char temp[16];
  sprintf(temp, "%s,1,%u", gEEPROMSig, gMaxProgramLen);
  sendSerialMsg(temp);
  return 1;
}


bool handleNewProgram(uint16_t offset) {
  uint16_t lenOffset = offset + 1;
  if (lenOffset >= gSerialLen) {
    return 0;
  }
  //uint16_t programLen = gSerialBuf[lenOffset];
  uint16_t programLen = gSerialBuf[lenOffset] - '0';
  uint16_t programEndOffset = programLen + lenOffset;
  if (programEndOffset >= gSerialLen) {
    return 0;
  }
  if (!setProgramA(gSerialBuf, lenOffset + 1, programLen, true)) {
    return 0;
  }
  char temp[16];
  sprintf(temp, "POK %d", programLen);
  sendSerialMsg(temp);
  return programEndOffset + 1;
}

void flushSysCmds() {
  uint16_t consumed = 0;
  
  while (gSerialLen > 0) {
    switch (gSerialBuf[0]) {
      case 'P':
        consumed = handleNewProgram(0);
        break;
      default:
        consumed = 1;
        break;
    }
    if (consumed < 1) {
      break;
    }
    uint16_t newLen = gSerialLen - consumed;
    if (newLen > 0) {
      memmove(gSerialBuf, gSerialBuf + consumed, newLen);
    }
    gSerialLen = newLen;
  }
}

void checkForSysCmds() {
  char temp[32];
  sprintf(temp, "I,ec7,1.0,%s", gMsg);
  sendSerialMsg(temp);
  
  while (Serial.available() && gSerialLen < MAX_SERIAL_BUF_SIZE) {
    gSerialBuf[gSerialLen] = Serial.read();
    gSerialLen += 1;
  }
  if (gSerialLen > 0) {
    flushSysCmds();
  }
}





template<tBufLen X> class MsgBuf {

  private:

    tBufByte buf[X];
    tBufLen  len;
    
  public:

    MsgBuf() {
      reset();
    }

    void reset() {
      this->len = 0;
    }

    void appendByte(tBufByte c) {
      this->buf[this->len++] = c;
    }

    tBufByte *getBufPtr() {
      return this->buf;
    }

    bool hasSpace() {
      return this->len < X;
    }

    tBufLen getLen() {
      return this->len;
    }

    tBufByte getByteAt(tBufLen index) {
      return this->buf[index];
    }

    void setByteAt(tBufLen index, tBufByte ch) {
      this->buf[index] = ch;
    }

    void splice(tBufLen to, tBufLen from, tBufLen len) {
      if (len < 0) {
        len = this->len - from;
      }
      memmove(buf + to, buf + from, len);
      this->len = to + len;
    }

    
};

#define MAX_UINT8_BUF_SIZE (256)

typedef MsgBuf<MAX_UINT8_BUF_SIZE> MsgBufBig;
typedef MsgBuf<32> MsgBufResp;

class CobsDecoderCallback {

  public:

    virtual void onPacket(MsgBufBig *pData) = 0;
    
};

class CobsDecoder {

  private:

    MsgBufBig *input, output;
    CobsDecoderCallback *callback;

  public:

    CobsDecoder(MsgBufBig *pInput, CobsDecoderCallback *pCallback) {
      this->callback = pCallback;
      this->input = pInput;
    }

    /*
     * UnStuffData decodes "length" bytes of data at
     * the location pointed to by "ptr", writing the
     * output to the location pointed to by "dst".
     *
     * Returns the length of the decoded data
     * (which is guaranteed to be <= length).
     */
     
    tBufLen UnStuffData(tBufLen start, tBufLen len, tBufLen *zeroAt) {
      tBufByte code = 0xFF, copy = 0;
      tBufLen end = start + len;
      
      *zeroAt = -1;
      
      for (tBufLen curr = start; curr < end; copy--) {
        if (copy != 0) {
          this->output.appendByte(this->input->getByteAt(curr++));
        } else {
          if (code != 0xFF) {
            this->output.appendByte(0);
          }
          copy = code = this->input->getByteAt(curr++);
          if (code == 0) {
            *zeroAt = curr;
            break; /* Source length too long */
          }
        }
      }
      return output.getLen();
    }
    

    bool feed() {
      tBufLen zeroAt;

      output.reset();
     
      tBufLen len = this->UnStuffData(0, this->input->getLen(), &zeroAt);
      if (-1 != zeroAt) {
        /*
         * Feed this packet.
         */
        callback->onPacket(&output);
        this->input->splice(0, zeroAt + 1, -1);
        return true;
      }
      if (len >= MAX_UINT8_BUF_SIZE) {
        /*
         * Too big of a packet. Discard.
         */
        this->input->reset();
      }
      return false;
    }

};

#define MAX_LED 6

class LedMgr {

  private:

    uint8_t getHwPin(uint8_t pin) {
      return pin + 8;
    }

  public:

    
    LedMgr() {
      for (int i = 0; i < MAX_LED; i += 1) {
        uint8_t pin = getHwPin(i);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
      }
    }

    void allTo(const uint8_t state) {
      for (int i = 0; i < MAX_LED; i += 1) {
        uint8_t pin = getHwPin(i);
        digitalWrite(pin, state);
      }
    }

    void pinTo(uint8_t pin, const uint8_t state) {
      uint8_t hwPin = getHwPin(pin);
      digitalWrite(hwPin, state);
    }

    uint8_t getNumLights() {
      return MAX_LED;
    }

};

class SerialMgr {

  private:

    MsgBufBig *input;

  public:

    SerialMgr(MsgBufBig *pInput) {
      this->input = pInput;
    }

    void waitForReady() {
      Serial.begin(115200);      
      while (!Serial) {
        delay(100);
      }
    }

    tBufLen fromSerial() {
      while (Serial.available() && this->input->hasSpace()) {
        this->input->appendByte(Serial.read());
      }
      return this->input->getLen();
    }

    void send(MsgBufResp *pOutput) {
      tBufLen len = pOutput->getLen();
      for (tBufLen i = 0; i < len; i += 1) {
        Serial.write(pOutput->getByteAt(i));
      }
    }
};

class CobsEncoder {

  private:

    SerialMgr  *serialMgr;
    MsgBufResp  tempBuf, respBuf;

    inline void StartBlock(tBufLen *code_ptr, tBufByte *code) {
      *code_ptr = this->tempBuf.getLen();
      this->tempBuf.appendByte(0);
      *code = 1;
    }

    inline void FinishBlock(tBufLen code_ptr, tBufByte code) {
      this->tempBuf.setByteAt(code_ptr, code);
    }
    
    void StuffData() {
      tBufByte code;
      tBufLen  code_ptr, ptr = 0, end = this->respBuf.getLen();
      tBufLen  dst = 0;

      tempBuf.reset();
      StartBlock(&code_ptr, &code);
      while (ptr < end) {
        if (code != 0xFF) {
          tBufByte c = this->respBuf.getByteAt(ptr++);
          if (c != 0) {
            this->tempBuf.appendByte(c);
            code++;
            continue;
          }
        }
        FinishBlock(code_ptr, code);
        StartBlock(&code_ptr, &code);
      }
      FinishBlock(code_ptr, code);
    }

  public:

    CobsEncoder(SerialMgr *pSerialMgr) {
      this->serialMgr = pSerialMgr;
    }

    MsgBufResp *getRespBuf() {
      return &this->respBuf;
    }
    
    void send() {
      this->StuffData();
      this->serialMgr->send(&this->tempBuf);
    }

};

#define PROTOCOL_VERSION (1)

class CmdProcessor : public CobsDecoderCallback {

  private:

    typedef enum _tCmds {
      Program = 80,
      Query
    } tCmds;
    
    SerialMgr *serialMgr;
    LedMgr *ledMgr;
    CobsEncoder *encoder;
    MsgBufResp *respBuf;
    
  public:

    CmdProcessor(SerialMgr *pSerialMgr, LedMgr *pLedMgr, CobsEncoder *pEncoder) {
      this->serialMgr = pSerialMgr;
      this->ledMgr = pLedMgr;
      this->encoder = pEncoder;
      this->respBuf = this->encoder->getRespBuf();
    }

    void onPacket(MsgBufBig *pData) {
      tBufLen len = pData->getLen();
      
      if (len < 1) {
        return;
      }
      
      this->respBuf->reset();
      tBufByte cmd = pData->getByteAt(0);
      switch (cmd) {
        case tCmds::Query:
          
          this->respBuf->appendByte(tCmds::Query);
          this->respBuf->appendByte(pData->getByteAt(1)); /* Req <-> Resp match */
          this->respBuf->appendByte(PROTOCOL_VERSION);
          this->respBuf->appendByte(this->ledMgr->getNumLights());
          this->encoder->send();
          this->ledMgr->pinTo(len, HIGH);
          break;
      }
    }

};

MsgBufBig input;
LedMgr ledMgr;

SerialMgr serialMgr(&input);
CobsEncoder cobsEncoder(&serialMgr);
CmdProcessor cmdProcessor(&serialMgr, &ledMgr, &cobsEncoder);
CobsDecoder cobsDecoder(&input, &cmdProcessor);


/*
 * n = Turn On
 * o = Turn Off
 * w = Wait
 */

uint8_t state = LOW;
   
void loop() {
  if (serialMgr.fromSerial() > 0) {
    cobsDecoder.feed();
  }
  delay(100);
  
  /*
  checkForSysCmds();
  if (gProgramLen < 1) {
    delay(1000);
  } else {

    char instruction = 0;
    char temp[16];
    
    for (int i = 0; i < gProgramLen; i += 1) {
      checkForSysCmds();
      switch (gProgram[i]) {
        case 'n':
        case 'N':
        case 'o':
        case 'O':
        case 'w':
        case 'W':
          instruction = gProgram[i];
          break;
        default:
          if (instruction) {
            execInstruction(instruction, gProgram[i]);
            instruction = 0;
          }
          break;
      }
    }
  }
  */
    
}

void setup() {

  serialMgr.waitForReady();
  /*
  gSerialLen = 0;
  gMaxProgramLen = min(MAX_PROG_BUF_SIZE, EEPROM.length() - 8);
  
  if (!loadProgram()) {
    setProgramA((uint8_t *)FACTORY_PROGRAM, 0, strlen(FACTORY_PROGRAM), true);
  }
  */
}
