#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef SERIALDEBUG
    #define DEBUGKV(key,val) {Serial.print(F(key)); Serial.println(val);}
    #define DEBUG_PRINT(val)  { Serial.print(val);}
    #define DEBUG_PRINTLN(val)  { Serial.println(val);}
#else
    #define DEBUGKV(key,val) {}
    #define DEBUG_PRINT(val)  {}
    #define DEBUG_PRINTLN(val)  {}
#endif

#endif