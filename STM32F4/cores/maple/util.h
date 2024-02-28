#ifndef UTIL_H
#define UTIL_H


// #ifndef word
//  #define word(a, b) ( (uint16_t)((a)<<8) | (b) )
// #endif

#ifndef htons
 #define htons(x) __builtin_bswap16(x)
//  #define ntohs(x) htons(x)
#endif

#ifndef htonl
 #define htonl(x) __builtin_bswap32(x)
 #define ntohl(x) htonl(x)
#endif


#ifdef __cplusplus
#include <wirish.h>
extern "C" {
#endif

extern uint8_t dbg;
#define DBG_BUF_SIZE 4096
#if 0
extern char buf[DBG_BUF_SIZE];
#define PRINTF(...) { snprintf(buf, sizeof(buf), __VA_ARGS__); Serial.println(buf); }
#else
void PRINTF(const char * format, ...); // should be declared in each module in part
#endif

char * printIP(const uint8_t * address);
char * printMAC(const uint8_t * address);

#ifdef __cplusplus
}
#endif

#endif
