#include <EEPROM.h>


/*
 * COBS implementation
 * 
 * https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */

#include <stdint.h>
#include <stddef.h>

typedef int16_t bufLen;

#define StartBlock()  (code_ptr = dst++, code = 1)
#define FinishBlock() (*code_ptr = code)

size_t StuffData(const uint8_t *ptr, bufLen length, uint8_t *dst) {
  const uint8_t *start = dst, *end = ptr + length;
  uint8_t code, *code_ptr; /* Where to insert the leading count */

  StartBlock();
  while (ptr < end) {
    if (code != 0xFF) {
      uint8_t c = *ptr++;
      if (c != 0) {
        *dst++ = c;
        code++;
        continue;
      }
    }
    FinishBlock();
    StartBlock();
  }
  FinishBlock();
  return dst - start;
}



/*
 * UnStuffData decodes "length" bytes of data at
 * the location pointed to by "ptr", writing the
 * output to the location pointed to by "dst".
 *
 * Returns the length of the decoded data
 * (which is guaranteed to be <= length).
 */
 
bufLen UnStuffData(const uint8_t *ptri, bufLen length, uint8_t *dst, bufLen *zeroAt) {
  const uint8_t *start = dst, *ptr = ptri, *end = ptr + length;
  uint8_t code = 0xFF, copy = 0;

  *zeroAt = -1;
  
  for (; ptr < end; copy--) {
    if (copy != 0) {
      *dst++ = *ptr++;
    } else {
      if (code != 0xFF)
        *dst++ = 0;
      copy = code = *ptr++;
      if (code == 0) {
        *zeroAt = ptr - ptri;
        break; /* Source length too long */
      }
    }
  }
  return dst - start;
}

#define MAX_UINT8_BUF_SIZE (256)

class CobsDecoderCallback {

  public:

    virtual void onPacket(uint8_t *pBuf, bufLen len) = 0;
    
};

class CobsDecoder {

  public:

    typedef struct _tBuf {
      uint8_t buf[MAX_UINT8_BUF_SIZE];
      uint8_t len;
    } tBuf;

  private:

    tBuf input, output;
    CobsDecoderCallback *callback;

    void initBuf(tBuf &buf) {
      buf.len = 0;
      //memset(buf.buf, 0, sizeof(buf.buf));    
    }

  public:

    CobsDecoder(CobsDecoderCallback *pCallback) {
      initBuf(input);
      callback = pCallback;
    }

    bool append(uint8_t *pBuf, uint8_t len) {
      if (input.len + len > MAX_UINT8_BUF_SIZE) {
        return false;
      }
      memcpy(input.buf + input.len, pBuf, len);
      input.len += len;
      return true;
    }

    bool feed() {
      bufLen zeroAt;

      initBuf(output);
      bufLen len = UnStuffData(input.buf, input.len, output.buf, &zeroAt);
      if (-1 != zeroAt) {
        /*
         * Feed this packet.
         */
        callback->onPacket(output.buf, len);
        bufLen nextByte = zeroAt + 1;
        bufLen newLen = input.len - nextByte;
        memmove(input.buf, input.buf + nextByte, newLen);
        input.len = newLen;
        return true;
      }
      if (len >= MAX_UINT8_BUF_SIZE) {
        /*
         * Too big of a packet. Discard.
         */
        initBuf(input);
      }
      return false;
    }

};


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

#define MAX_LED 6

class LedManager {

  private:

    uint8_t getHwPin(uint8_t pin) {
      return pin + 8;
    }

  public:

    
    LedManager() {
      for (int i = 0; i < MAX_LED; i += 1) {
        uint8_t pin = getHwPin(i);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
      }
    }
};

//#define MAX_UINT8_BUF_SIZE (256)

class MsgBuf {

  private:

    uint8_t buf[MAX_UINT8_BUF_SIZE];
    int16_t len;
    
  public:

    MsgBuf() {
      reset();
    }

    void reset() {
      this->len = 0;
    }

    void appendByte(uint8_t c) {
      this->buf[this->len++] = c;
    }

    bool hasSpace() {
      return this->len < MAX_UINT8_BUF_SIZE;
    }
    
};

class SerialManager {

  private:

    MsgBuf msgBuf;

  public:

    SerialManager() {
      Serial.begin(115200);
    }

    void WaitForReady() {
      while (!Serial) {
        delay(100);
      }
    }

    void fromSerial() {
      while (Serial.available() && msgBuf.hasSpace()) {
        msgBuf.appendByte(Serial.read());
      }
    }
  
};

SerialManager serialMgr;
LedManager ledMgr;

/*
 * n = Turn On
 * o = Turn Off
 * w = Wait
 */
void loop() {

  delay(1000);
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

  serialMgr.WaitForReady();
  /*
  gSerialLen = 0;
  gMaxProgramLen = min(MAX_PROG_BUF_SIZE, EEPROM.length() - 8);
  
  if (!loadProgram()) {
    setProgramA((uint8_t *)FACTORY_PROGRAM, 0, strlen(FACTORY_PROGRAM), true);
  }
  */
}
