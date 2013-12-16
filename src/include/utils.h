/*******************************************************************************
 Copyright (C) 2012  Trey Marc ( a t ) gmail.com

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ****************************************************************************/

#ifndef MWI_UTILS_H
#define MWI_UTILS_H

/*
 * version
 */
#define MWGC_VERSION "SNAPSHOT-2013.12.14"

/*
 * log level
 */
#if !defined(_LOGLEVL)
#define _LOGLEVL 0
#endif

/*
 * build specific
 */
#if defined( _WINDOZ )
#include <Windows.h>
#else
typedef unsigned short HANDLE;
#endif


/*
 * logic
 */
#define NOK -1
#define OK 1

/*
 * math
 */
#define PI 3.1415926535897932384626433832795
#define deg2radian(X) (PI * X) / 180

/*
 * log
 */
#define MW_ERROR(x) printf(x);

#if (_LOGLEVL>2)
#define MW_INFO(x) printf(x);
#define MW_TRACE(x) printf(x);
#else
#define MW_INFO(x);
#define MW_TRACE(x);

#endif

#endif
