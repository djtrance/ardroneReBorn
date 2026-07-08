#ifndef STD_H
#define STD_H

#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>

/* Basic XDC types */
typedef void            Void;
typedef void *          Ptr;
typedef int             Int;
typedef unsigned int    Uns;
typedef unsigned int    UInt;
typedef char            Char;
typedef char *          String;
typedef int             Bool;

/* Sized integer types */
typedef signed char     Int8;
typedef unsigned char   UInt8;
typedef short           Int16;
typedef unsigned short  UInt16;
typedef int             Int32;
typedef unsigned int    UInt32;
typedef long long       Int64;
typedef unsigned long long UInt64;

/* Lowercase 'u' aliases (XDAS naming) */
typedef UInt8   Uint8;
typedef UInt16  Uint16;
typedef UInt32  Uint32;
typedef UInt64  Uint64;

/* XDAS aliases */
typedef UInt8   XDAS_UInt8;
typedef Int8    XDAS_Int8;
typedef UInt16  XDAS_UInt16;
typedef Int16   XDAS_Int16;
typedef UInt32  XDAS_UInt32;
typedef Int32   XDAS_Int32;

/* Argument type (pointer-sized int) */
typedef unsigned long   UArg;

/* Floating-point types */
typedef float           Float;
typedef double          Double;

#endif
