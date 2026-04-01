#ifndef TELEOP_TWIST_JOY_EXPORT_H
#define TELEOP_TWIST_JOY_EXPORT_H

#if defined _WIN32 || defined __CYGWIN__
  #ifdef TELEOP_TWIST_JOY_BUILDING_DLL
    #define TELEOP_TWIST_JOY_EXPORT __declspec(dllexport)
  #else
    #define TELEOP_TWIST_JOY_EXPORT __declspec(dllimport)
  #endif
#else
  #define TELEOP_TWIST_JOY_EXPORT __attribute__((visibility("default")))
#endif

#endif  // TELEOP_TWIST_JOY_EXPORT_H
