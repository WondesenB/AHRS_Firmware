/* Copyright (C) 1997,1998,1999,2000,2001,2006 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

/*
 *        ISO C99: 7.18 Integer types <stdint.h>
 */

#ifndef _STDINT_H
#define _STDINT_H        1

//#include <features.h>
//#include <bits/wchar.h>
//#include <bits/wordsize.h>

/* Exact integral types.  */

/* Signed.  */

/* There is some amount of overlap with <sys/types.h> as known by inet code */
#ifndef __int8_t_defined
# define __int8_t_defined
typedef signed char                Int8_t;
//typedef short int                int16_t;
//typedef int                        int32_t;
# if __WORDSIZE == 64
typedef long int                int64_t;
# else
__extension__
//typedef long long int                int64_t;
# endif
#endif

/* Unsigned.  */
typedef unsigned char                Uint8_t;
//typedef unsigned short int        uint16_t;
#ifndef __uint32_t_defined
//typedef unsigned int                uint32_t;
# define __uint32_t_defined
#endif
#if __WORDSIZE == 64
typedef unsigned long int        uint64_t;
#else
__extension__
//typedef unsigned long long int        uint64_t;
#endif

#endif /* stdint.h */
