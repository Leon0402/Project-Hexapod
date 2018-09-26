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
  #include <stdlib.h>
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
  void write(char data);
  //Data is sent as a series of bytes to the serial port
  void write(const char* data);

  //Converts T into a char array and calls write for every char
  template<typename T>
  void print(T data) {
    char buffer[11];
    #ifndef X86_64
      itoa(data, buffer, 10);
    #endif
    this->write(buffer);
  }

  //calls sufficient print method and adds a carriage return
  template<typename T>
  void println(T data) {
    this->print(data);
    this->write('\n');
  }
};

template<>
inline void Stream::print<char>(char data) {
  this->write(data);
}

template<>
inline void Stream::print<const char*>(const char* data) {
  this->write(data);
}

template<>
inline void Stream::print<double>(double data) {
  char buffer[7];
  #ifndef X86_64
    dtostrf(data, 6, 2, buffer);
  #endif
  this->write(buffer);
}

template<>
inline void Stream::print<float>(float data) {
  char buffer[9];
  #ifndef X86_64
    dtostrf(data, 8, 3, buffer);
  #endif
  this->write(buffer);
}

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
