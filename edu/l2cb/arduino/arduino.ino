/*
 * COBS implementation
 * 
 * https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */

#include <stdint.h>
#include <stddef.h>

typedef int16_t tBufLen;
typedef uint8_t tBufByte;

#define MAX_PROG_BUF_SIZE 255
#define MAX_SERIAL_BUF_SIZE (MAX_PROG_BUF_SIZE + 8)

#define FACTORY_PROGRAM \
  "N0W1N1W1N2W1N3W1N4W1N5W1" \
  "O0W1O1W1O2W1O3W1O4W1O5W1" \
  "N0N1N2N3N4N5W1O0O1O2O3O4O5W1" \
  "N0N1N2N3N4N5W1O0O1O2O3O4O5W1"

#define PC_TEST

#ifdef PC_TEST

/*
 * Compile with "g++ -g -x c++ arduino.ino" for testing on PC
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define OUTPUT (0)
#define INPUT  (1)
#define LOW    (0)
#define HIGH   (1)

typedef struct _PinInfo {
  uint8_t mode;
  uint8_t state;
} PinInfo;

PinInfo Pins[13];

void pinMode(uint8_t pin, uint8_t mode) {
  Pins[pin].mode = mode;
}

void digitalWrite(uint8_t pin, uint8_t state) {
  Pins[pin].state = state;
}

void delay(uint32_t d) {
  fprintf(stdout, "Delay %d\n", d);
  usleep(d * 1000);
}

class SerialCls {

  private:

    uint8_t  index, aLen;
    uint8_t *pStr;
    
  public:

    void set(uint8_t *str, uint8_t al) {
      index = 0;
      pStr = str;
      aLen = al;
    }

    explicit operator bool() const {
        return true;
    }  

    bool available() {
      return index < aLen;
    }

    uint8_t read() {
      uint8_t result = -1;
      if (available()) {
        result = pStr[index++];
      }
      fprintf(stdout, "Returning %x\n", result);
      return result;
    }

    void begin(uint32_t baudRate) {
      fprintf(stdout, "Baud rate set to %d\n", baudRate);
    }

    void write(uint8_t data) {
      fprintf(stdout, "data %d\n", data);
    }
};

SerialCls Serial;

#endif

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
      if (from >= this->len) {
        this->len = to;
        return;
      }
      if (len < 0) {
        len = this->len - from;
      }
#ifdef PC_TEST
      fprintf(stdout, "Memmove to: %d from: %d len: %d this.len %d\n", to, from, len, this->len);
#endif
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

    virtual void onPacket(MsgBufBig *pData) override {
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


#ifdef PC_TEST
int main(int argc, const char *argv[]) {
  uint8_t data[] = {3, 80, 1, 0, 3, 80, 1, 0};
  setup();
  Serial.set(data, sizeof(data));
  while (true) {
    loop();
  }
}
#endif
