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

#include "debug.h"

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
      memmove(buf + to, buf + from, len);
      this->len = to + len;
#ifdef PC_DEBUG
      fprintf(stdout, "Memmove to: %d from: %d len: %d this.len %d\n", to, from, len, this->len);
#endif
      
    }

    void copyFrom(MsgBuf *other, tBufLen offset, tBufLen len) {
      memcpy(this->buf, other->buf + offset, len);
      this->len = len;
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
      if (-1 != zeroAt || len < MAX_UINT8_BUF_SIZE) {
        /*
         * Feed this packet.
         */
        callback->onPacket(&output);
        this->input->reset();
        return true; 
        
        if (-1 != zeroAt) {
          this->input->splice(0, zeroAt, -1);
        } else {
          this->input->reset();
        }
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
      Serial.flush();
    }
};

class ProgramMgr {

  private:

    MsgBufBig program;
    tBufLen ip;
    LedMgr *ledMgr;

    void resetIP(void) {
      this->ip = 0;
    }
    
  public:

    ProgramMgr(LedMgr *pLedMgr) {
      this->ledMgr = pLedMgr;
      this->resetIP();
      this->program.reset();
    }

    
    void loadProgramFrom(MsgBufBig *other) {
      this->program.copyFrom(other, 2, other->getLen() - 2);
      this->resetIP();
    }

    uint16_t advance() {
      tBufLen len = this->program.getLen();
      if (len < 1 || 0 != (len % 2)) {
        return 0;
      }
      uint16_t d = 0;
      tBufByte instruction = this->program.getByteAt(this->ip);
      tBufByte arg = this->program.getByteAt(this->ip + 1);
      uint8_t numLights = this->program.getLen();
      tBufByte index = 0;
      switch (instruction) {
        case 78: /* N = ON */ {
          index = arg - 48; /* "9" - "0" */
          if (index >= 0 && index < numLights) {
            this->ledMgr->pinTo(index, HIGH); 
          }
        }
        break;
      case 79: /* O = OFF */ {
          index = arg - 48; /* "9" - "0" */
          if (index >= 0 && index < numLights) {
            this->ledMgr->pinTo(index, LOW);
          }
        }
        break;
      case 87: /* W = WAIT */ {
          uint8_t interval = arg - 48;
          if (interval >= 0 && index < 10) {
            d = (interval * 1000);
          }
        }
        break;
      }
      this->ip += 2;
      if (this->ip >= len) {
        this->ip = 0;
      }
      return d;
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
    ProgramMgr *progMgr;
    CobsEncoder *encoder;
    MsgBufResp *respBuf;
    
  public:

    CmdProcessor(SerialMgr *pSerialMgr, LedMgr *pLedMgr, ProgramMgr *pProgMgr, CobsEncoder *pEncoder) {
      this->progMgr = pProgMgr;
      this->serialMgr = pSerialMgr;
      this->ledMgr = pLedMgr;
      this->encoder = pEncoder;
      this->respBuf = this->encoder->getRespBuf();
    }

    virtual void onPacket(MsgBufBig *pData) override {
      tBufLen len = pData->getLen();
#ifdef PC_DEBUG
      fprintf(stdout, "Buf len %d\n", len);
      for (tBufLen i = 0; i < len; i += 1) {
        fprintf(stdout, "Index %d data %d\n", i, pData->getByteAt(i));
      }
#endif          
      
      if (len < 1) {
        return;
      }
      
      this->respBuf->reset();
      tBufByte cmd = pData->getByteAt(0);
      switch (cmd) {
        case tCmds::Query:
#ifdef PC_DEBUG
          fprintf(stdout, "Query\n");
#endif          
          this->respBuf->appendByte(tCmds::Query);
          this->respBuf->appendByte(pData->getByteAt(1)); /* Req <-> Resp match */
          this->respBuf->appendByte(PROTOCOL_VERSION);
          this->respBuf->appendByte(this->ledMgr->getNumLights());
          this->encoder->send();
          break;

        case tCmds::Program:
          this->progMgr->loadProgramFrom(pData);
          break;
      }
    }

};

LedMgr ledMgr;

MsgBufBig input;

ProgramMgr progMgr(&ledMgr);

SerialMgr serialMgr(&input);
CobsEncoder cobsEncoder(&serialMgr);
CmdProcessor cmdProcessor(&serialMgr, &ledMgr, &progMgr, &cobsEncoder);
CobsDecoder cobsDecoder(&input, &cmdProcessor);


uint8_t state = LOW;
uint16_t count = 0;

   
void loop() {
  tBufLen fs = serialMgr.fromSerial();
  if (fs > 0) {
    if (cobsDecoder.feed()) {
      ledMgr.pinTo(fs, HIGH);
    }
  }
  uint16_t d = progMgr.advance();
  if (d > 1) {
    delay(d);
  }
    
}

void setup() {
  serialMgr.waitForReady();
}


#ifdef PC_DEBUG
int main(int argc, const char *argv[]) {
  uint8_t data[] = {3, 81, 1};
  //uint8_t data[] = {03, 11, 22, 02, 33, 00};
  setup();
  Serial.set(data, sizeof(data));
  uint32_t count = 0;
  while (true) {
    count += 1;
    if (count > 9999999) {
      fprintf(stdout, "%d\n", count);
      count = 0;
    }
    loop();
  }
}
#endif
