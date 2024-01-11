// Copyright [2016] Zurich Instruments AG
//
// Note: The utility functions included in this header file are a preliminary
// version.  Function calls and parameters may change without notice.

#ifndef __ETC_EXAMPLES_ZIUTILS_H__
#define __ETC_EXAMPLES_ZIUTILS_H__

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#define PRsize_t "I"
#define PRptrdiff_t "I"
#define PRIu64 "llu"
#define PRId64 "lld"
#ifndef snprintf
#define snprintf sprintf_s
#endif
#define strtok_r strtok_s
#else
#include <inttypes.h>
#define PRsize_t "z"
#define PRptrdiff_t "t"
#include <string.h>
#include <unistd.h>
#endif
#include <stdexcept>

void sleep(int ms)
{
#ifdef _WIN32
  Sleep(ms);
#else
  usleep(ms * 1000);
#endif
}

static inline bool isError(ZIResult_enum resultCode)
{
  if (resultCode != ZI_INFO_SUCCESS)
  {
    char* message;
    ziAPIGetError(resultCode, &message, NULL);
    fprintf(stderr, "Error: %s\n", message);
    return true;
  }
  return false;
}

static inline void checkError(ZIResult_enum resultCode)
{
  if (resultCode != ZI_INFO_SUCCESS)
  {
    char* message;
    ziAPIGetError(resultCode, &message, NULL);
    throw(std::runtime_error(message));
  }
}

static inline void checkLastError(ZIConnection conn)
{
  char message[1000];
  message[0] = 0;
  ziAPIGetLastError(conn, message, 1000);
  if (strlen(message) > 0)
  {
    throw(std::runtime_error(message));
  }
  else
  {
    printf("No error!\n");
  }
}

/// Query an environment variable with a default value. If the
/// environment variable is not set the defaultValue will be returned.
const char* ziUtilsGetEnv(const char* env, const char* defaultValue)
{
  const char* value = std::getenv(env);
  if (value == NULL)
  {
    value = defaultValue;
  }
  return value;
}

/// Check the versions of the API and Data Server are the same.
/** Issue a warning and return 0 if the release version of the API used in the
   session (daq) does not have the same release version as the Data Server (that
   the API is connected to). If the versions match return 1.

    @param[in]   conn     The initialised ::ZIConnection representing an API
   session.
 */
static inline uint8_t ziApiServerVersionCheck(ZIConnection conn)
{
  unsigned int apiRevision = 0;
  const char* apiVersion;
  ZIIntegerData serverRevision = 0;
  char serverVersion[1024];

  checkError(ziAPIGetRevision(&apiRevision));
  checkError(ziAPIGetVersion(&apiVersion));
  checkError(ziAPIGetValueI(conn, "/zi/about/revision", &serverRevision));
  const char path[] = "/zi/about/version";
  unsigned int length = 0;
  checkError(ziAPIGetValueString(conn, path, serverVersion, &length, 1024));

  if (strcmp(apiVersion, serverVersion) != 0)
  {
    // clang-format off
    printf("*******************************************************************************************************\n");
    printf("Warning: There is a mismatch between the versions of the API and Data Server. The API reports version `%s' (revision: "
           "%d) and Data Server `%s', (revision: %"  PRIu64 "). See the ``Compatibility'' Section in the LabOne Programming "
           "Manual for more information.\n", apiVersion, apiRevision, serverVersion, serverRevision);
    printf("*******************************************************************************************************\n");
    // clang-format on
    return 0;
  }

  return 1;
}

#endif  // __ETC_EXAMPLES_ZIUTILS_H__
