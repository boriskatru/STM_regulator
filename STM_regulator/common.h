/*
 * $Id: common.h 238 2015-12-28 22:10:50Z yeti-dn $
 *
 * Common includes for tests.
 *
 * I, the copyright holder of this work, release this work into the public
 * domain.  This applies worldwide.  In some countries this may not be legally
 * possible; if so: I grant anyone the right to use this work for any purpose,
 * without any conditions, unless such conditions are required by law.
 */
#define _USE_MATH_DEFINES
#define _ISOC99_SOURCE 1
#define _GNU_SOURCE 1
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "gwyfile.h"

#ifdef _WIN32
#include <io.h>
#include <process.h>
#define unlink _unlink
#else
#include <unistd.h>
#endif

#ifdef _MSC_VER
#define getpid _getpid
#endif

