/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:43:01
* @Last Modified by:   ksvbka
* @Last Modified time: 2016-12-12 00:39:47
*/

#include "utility.h"

/* Helper function */

static char buf[10];

/* Convert Integer to String*/
char* to_string(int i)
{
        char sign = '+';
        short len = 0;
        char *p = buf;
        char* pbuf = buf;
        if (i < 0) {
                sign = '-';
                i = -i;
        }
        do {
                *pbuf++ = (i % 10) + '0';
                len++;
                i /= 10;
        } while (i != 0);

        if (sign == '-') {
                *pbuf++ = '-';
                len++;
        }
        for (i = 0; i < len / 2; i++) {
                p[len] = p[i];
                p[i] = p[len - 1 - i];
                p[len - 1 - i] = p[len];
        }
        p[len] = 0;
        return buf;
}

/* Convert String to Integer*/
int to_int(char *s)
{
        int i = 0;
        char sign = '+';
        if (*s == '-') {
                sign = '-';
                s++;
        }
        while (*s != 0) {
                i = i * 10 + (*s - '0');
                s++;
        }
        if (sign == '-')
                i = -i;
        return i;
}


/* Convert an unsigned int to signed int.*/
int16_t unsignedToSigned(uint16_t val)
{
        if (val > 0x8000)
                val = -((65535 - val) + 1);
        return val;
}

/* Convert a 16-bit number into little endian */
uint16_t u16ToLittleEndian(uint16_t u16input )
{
        return ( (u16input >> 8) ^ (u16input << 8) );
}

/* Convert a 32-bit number into little endian. */
uint32_t u32ToLittleEndian(uint32_t u32input )
{
        return ( (u32input >> 24)
                 ^ ( (u32input >> 8) & 0x000FF00 )
                 ^ ( (u32input << 8) & 0x00FF0000 )
                 ^ ( (u32input << 24) & 0xFF000000 )
               );
}
