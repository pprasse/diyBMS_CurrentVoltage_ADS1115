#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef SERIALDEBUG
    #define DEBUG_PRINT(val)  { Serial.print(val);}
    #define DEBUG_PRINTLN(val)  { Serial.println(val);}
#else
    #define DEBUG_PRINT(val)  {}
    #define DEBUG_PRINTLN(val)  {}
#endif

#endif