#ifndef __DEBUG_H__
#define __DEBUG_H__

#define DEBUG 1

#if DEBUG
  #define DEBUG_begin(...)   Serial.begin(__VA_ARGS__)
  #define DEBUG_print(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_println(...) Serial.println(__VA_ARGS__)
  #define DEBUG_delay(...)   delay(__VA_ARGS__)
#else
  #define DEBUG_begin(...)
  #define DEBUG_print(...)
  #define DEBUG_println(...)
  #define DEBUG_delay(...)
#endif

#endif /* __DEBUG_H__ */