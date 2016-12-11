#ifndef __TYPE_DEF_H__
#define __TYPE_DEF_H__

typedef void (*callback) (void* param);

#define ARRAY_LENGTH(array) (sizeof((array))/sizeof((array)[0]))

/* Cat void* to ptr type* */
#define CAST_PTR(type, ptr) ((type* )ptr)

/* Cat void* to value of pointer type*/
#define CAST_VAL(type, ptr) (* ((type* )ptr))

typedef unsigned char  bool;

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef NULL
#define NULL 0
#endif

#define BIT0  0x0001
#define BIT1  0x0002
#define BIT2  0x0004
#define BIT3  0x0008
#define BIT4  0x0010
#define BIT5  0x0020
#define BIT6  0x0040
#define BIT7  0x0080
#define BIT8  0x0100
#define BIT9  0x0200
#define BIT10 0x0400
#define BIT11 0x0800
#define BIT12 0x1000
#define BIT13 0x2000
#define BIT14 0x4000
#define BIT15 0x8000

#endif // __TYPE_H__
