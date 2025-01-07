#ifndef NAOSOCCER_POS_ACTION_CPP__VISIBILITY_CONTROL_H_
#define NAOSOCCER_POS_ACTION_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define NAOSOCCER_POS_ACTION_CPP_EXPORT __attribute__ ((dllexport))
    #define NAOSOCCER_POS_ACTION_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define NAOSOCCER_POS_ACTION_CPP_EXPORT __declspec(dllexport)
    #define NAOSOCCER_POS_ACTION_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAOSOCCER_POS_ACTION_CPP_BUILDING_DLL
    #define NAOSOCCER_POS_ACTION_CPP_PUBLIC NAOSOCCER_POS_ACTION_CPP_EXPORT
  #else
    #define NAOSOCCER_POS_ACTION_CPP_PUBLIC NAOSOCCER_POS_ACTION_CPP_IMPORT
  #endif
  #define NAOSOCCER_POS_ACTION_CPP_PUBLIC_TYPE NAOSOCCER_POS_ACTION_CPP_PUBLIC
  #define NAOSOCCER_POS_ACTION_CPP_LOCAL
#else
#define NAOSOCCER_POS_ACTION_CPP_EXPORT __attribute__ ((visibility("default")))
#define NAOSOCCER_POS_ACTION_CPP_IMPORT
#if __GNUC__ >= 4
#define NAOSOCCER_POS_ACTION_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define NAOSOCCER_POS_ACTION_CPP_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define NAOSOCCER_POS_ACTION_CPP_PUBLIC
#define NAOSOCCER_POS_ACTION_CPP_LOCAL
#endif
#define NAOSOCCER_POS_ACTION_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // NAOSOCCER_POS_ACTION_CPP__VISIBILITY_CONTROL_H_