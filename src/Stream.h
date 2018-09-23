#ifndef STREAM_H
#define STREAM_H

#ifndef F_CPU
  #define F_CPU 16000000
#endif

#ifndef BAUD
  #define BAUD 9600
#endif

#ifndef X86_64
  #include <util/setbaud.h>
  #include <inttypes.h>
#else
  #include <cinttypes>
#endif

/*! TODO
* Add ringbuffer
* Optimize Code
*/

class Stream {
public:
  Stream();

  //Data is sent as a byte to the serial port
  void write(int8_t data);
  //Data is sent as a series of bytes to the serial port
  void write(const int8_t* data, uint8_t size);

  //Data is converted into ASCII and then is sent to the serial port
  void print(char data);
  void print(const char* data);
  void print(int data);
  void print(unsigned int data);
  void print(float data);

  //calls sufficient print method and adds a carriage return
  template<typename T>
  void println(T data) {
    this->print(data);
    this->write('\n');
  }
};

template<typename T>
Stream& operator<<(Stream& stream, const T data) {
  stream.print(data);
  return stream;
}

namespace avr {
  //buffered
  extern Stream cout;
  //not buffered
  extern Stream err;
}

#endif //STREAM_H
