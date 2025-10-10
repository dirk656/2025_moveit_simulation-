#pragma once

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RH_MOVEIT_PLANNER_EXPORT __attribute__ ((dllexport))
    #define RH_MOVEIT_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define RH_MOVEIT_PLANNER_EXPORT __declspec(dllexport)
    #define RH_MOVEIT_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef RH_MOVEIT_PLANNER_BUILDING_LIBRARY
    #define RH_MOVEIT_PLANNER_PUBLIC RH_MOVEIT_PLANNER_EXPORT
  #else
    #define RH_MOVEIT_PLANNER_PUBLIC RH_MOVEIT_PLANNER_IMPORT
  #endif
  #define RH_MOVEIT_PLANNER_PUBLIC_TYPE RH_MOVEIT_PLANNER_PUBLIC
  #define RH_MOVEIT_PLANNER_LOCAL
#else
  #define RH_MOVEIT_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define RH_MOVEIT_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define RH_MOVEIT_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define RH_MOVEIT_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RH_MOVEIT_PLANNER_PUBLIC
    #define RH_MOVEIT_PLANNER_LOCAL
  #endif
  #define RH_MOVEIT_PLANNER_PUBLIC_TYPE
#endif