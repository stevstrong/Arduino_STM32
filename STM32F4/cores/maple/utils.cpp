#include <stdio.h>
#include <stdarg.h>
// #include <stdint.h>
#include <Arduino.h>

extern "C" {

char buf[DBG_BUF_SIZE];
char ip_buf[16];
char mac_buf[20];
const char dummy[] = "DUMMY";

void DBG_PRINTF(uint8_t dbg, const char * format, ...)
{
    if (!dbg) return;
    va_list ap;
    va_start(ap, format);
    vsprintf(buf, format, ap);
    va_end(ap);
    Serial.println(buf);
    // delay(10);
}

#if 1
void PRINTF(const char * format, ...)
{
    va_list ap;
    va_start(ap, format);
    vsprintf(buf, format, ap);
    va_end(ap);
    Serial.println(buf);
    // delay(10);
}
#endif

char * printIP(const uint8_t * address)
{
    sprintf(ip_buf,"%u.%u.%u.%u",address[0],address[1],address[2],address[3]);
    return ip_buf;
}

char * printMAC(const uint8_t * address)
{
    sprintf(mac_buf,"%2x:%2x:%2x:%2x:%2x:%2x",address[0],address[1],address[2],address[3],address[4],address[5]);
    return mac_buf;
}


}
