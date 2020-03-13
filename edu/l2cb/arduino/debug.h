//#define PC_DEBUG

#ifdef PC_DEBUG

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

/* Totally 14 pins, 0 to 13 */

#define MAX_PINS 14

PinInfo Pins[MAX_PINS];

void pinMode(uint8_t pin, uint8_t mode) {
  if (pin < 0 || pin >= MAX_PINS) {
    fprintf(stdout, "PIN mode overflow pin: %d mode: %d\n", pin, mode);
  } else {
    Pins[pin].mode = mode;
    fprintf(stdout, "PIN %d mode set to %d\n", pin, mode); 
  }
}

void digitalWrite(uint8_t pin, uint8_t state) {
  if (pin < 0 || pin >= MAX_PINS) {
    fprintf(stdout, "digitalWrite overflow pin: %d state: %d\n", pin, state);
  } else {
    Pins[pin].state = state;
    fprintf(stdout, "PIN value %d set to %d\n", pin, state);
  }
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
      fprintf(stdout, "[From Serial] %d\n", result);
      return result;
    }
    
    void flush() {
      fprintf(stdout, "[To serial] FLUSH\n");
      fflush(stdout);
    }

    void begin(uint32_t baudRate) {
      fprintf(stdout, "Baud rate set to %d\n", baudRate);
    }

    void write(uint8_t data) {
      fprintf(stdout, "[To serial] data %d\n", data);
    }
};

SerialCls Serial;

const char *EEPROMFile = "eeprom.dat";

class EEPROMCls {

  private:

    FILE *fp;
    uint8_t buf[512];
    
  public:

    uint16_t length() {
      return sizeof(buf);
    }

    EEPROMCls() {
      FILE *fr = fopen(EEPROMFile, "rb");
      if (NULL != fr) {
        fread(buf, sizeof(uint8_t), sizeof(buf), fr);
        fclose(fr);
      }
      fp = fopen(EEPROMFile, "wb");
    }
    
    void write(uint16_t offset, uint8_t value) {
      fprintf(stdout, "[EEPROM] Write offset %d Value %x\n", offset, value);
      if (offset >= sizeof(buf)) {
        fprintf(stdout, "Write offset is invalid %d\n", offset);
        return;
      }
      buf[offset] = value;
      fseek(fp, 0, SEEK_SET); 
      fwrite(buf, sizeof(uint8_t), sizeof(buf), fp);
      fflush(fp);
    }

    ~EEPROMCls() {
      fclose(fp);
    }

    uint8_t read(uint16_t offset) {
      if (offset >= sizeof(buf)) {
        fprintf(stdout, "Invalid read offset %d\n", offset);
        return 0;
      }
      fprintf(stdout, "[EEPROM] Read %x val %x\n", offset, buf[offset]);
      return buf[offset];
    }
};

EEPROMCls EEPROM;

//g++ -g -Wall -Wpedantic -fstack-protector -fsanitize=address -O0 -DPC_DEBUG -x c++ arduino.ino

#endif
