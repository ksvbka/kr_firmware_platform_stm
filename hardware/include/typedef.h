#ifndef __TYPE_DEF_H__
#define __TYPE_DEF_H__

typedef void (*callback) (void* param);

#define ARRAY_LENGTH(array) (sizeof((array))/sizeof((array)[0]))

/* Cat void* to ptr type* */
#define CAST_PTR(ptr, type) ((type* )ptr)

/* Cat void* to value of pointer type*/
#define CAST_VAL(ptr, type) (* ((type* )ptr))

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

#endif // __TYPE_H__
