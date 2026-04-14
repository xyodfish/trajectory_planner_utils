/**
 * @file velocity_planning.h
 * @brief Velocity Planning Library - Main Include Header
 * 
 * Include this header to access all velocity planning functionality.
 * 
 * @example
 * #include <vp/velocity_planning.h>
 */

#ifndef VP_VELOCITY_PLANNING_H
#define VP_VELOCITY_PLANNING_H

// Core interfaces
#include "vp/velocity_planner_interface.h"
#include "vp/planner_exception.h"

// Planners
#include "vp/trapezoidal_planner.h"
#include "vp/double_s_planner.h"
#include "vp/multi_velocity_planner.h"

#endif  // VP_VELOCITY_PLANNING_H
