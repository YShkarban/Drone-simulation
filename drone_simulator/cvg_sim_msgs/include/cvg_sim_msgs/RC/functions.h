
#ifndef HECTOR_UAV_MSGS_RC_FUNCTIONS_H
#define HECTOR_UAV_MSGS_RC_FUNCTIONS_H

#include <hector_uav_msgs/RC.h>
#include <algorithm>

namespace hector_uav_msgs {

  static inline const char *getFunctionString(uint8_t function)
  {
    switch(function) {
      case RC::ROLL:   return "ROLL";
      case RC::PITCH:  return "PITCH";
      case RC::YAW:    return "YAW";
      case RC::STEER:  return "STEER";
      case RC::HEIGHT: return "HEIGHT";
      case RC::THRUST: return "THRUST";
      case RC::BRAKE:  return "BRAKE";
    }
    return 0;
  }

  static inline bool hasAxis(const RC& rc, RC::_axis_function_type::value_type function)
  {
    return std::find(rc.axis_function.begin(), rc.axis_function.end(), function) != rc.axis_function.end();
  }

  static inline bool getAxis(const RC& rc, RC::_axis_function_type::value_type function, RC::_axis_type::value_type& value)
  {
    if (!rc.valid) return false;
    const RC::_axis_function_type::const_iterator it = std::find(rc.axis_function.begin(), rc.axis_function.end(), function);
    if (it == rc.axis_function.end()) return false;
    value = rc.axis.at(it - rc.axis_function.begin());
    return true;
  }

  static inline void setAxis(RC& rc, RC::_axis_function_type::value_type function, RC::_axis_type::value_type value)
  {
    const RC::_axis_function_type::iterator it = std::find(rc.axis_function.begin(), rc.axis_function.end(), function);
    if (it == rc.axis_function.end()) {
      rc.axis_function.push_back(function);
      rc.axis.push_back(value);
    } else {
      rc.axis.at(it - rc.axis_function.begin()) = value;
    }
  }

  static inline bool hasSwitch(const RC& rc, RC::_swit_function_type::value_type function)
  {
    return std::find(rc.swit_function.begin(), rc.swit_function.end(), function) != rc.swit_function.end();
  }

  static inline bool getSwitch(const RC& rc, RC::_swit_function_type::value_type function, RC::_swit_type::value_type& value)
  {
    if (!rc.valid) return false;
    const RC::_swit_function_type::const_iterator it = std::find(rc.swit_function.begin(), rc.swit_function.end(), function);
    if (it == rc.swit_function.end()) return false;
    value = rc.swit.at(it - rc.swit_function.begin());
    return true;
  }

  static inline void setSwitch(RC& rc, RC::_swit_function_type::value_type function, RC::_swit_type::value_type value)
  {
    const RC::_swit_function_type::iterator it = std::find(rc.swit_function.begin(), rc.swit_function.end(), function);
    if (it == rc.swit_function.end()) {
      rc.swit_function.push_back(function);
      rc.swit.push_back(value);
    } else {
      rc.swit.at(it - rc.swit_function.begin()) = value;
    }
  }

} 

#endif 
