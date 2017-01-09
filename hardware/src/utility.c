/*
* @Author: Trung Kien
* @Date:   2016-11-29 11:43:01
* @Last Modified by:   Kienltb
* @Last Modified time: 2017-01-09 16:31:58
*/

#include "utility.h"

/* Helper function */

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

/* Convert a float into 4byte and store to buffer */
void float_to_buff(float data, char* buffer)
{
        buffer = (char*)(&data);
}

/* Convert a 4byte in buffer to float */
float buffer_to_float(char* buffer)
{
        return *(float*)buffer;
}
