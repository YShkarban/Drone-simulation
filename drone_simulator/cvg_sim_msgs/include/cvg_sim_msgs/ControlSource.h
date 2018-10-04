
#ifndef HECTOR_UAV_MSGS_CONTROLSOURCE_H
#define HECTOR_UAV_MSGS_CONTROLSOURCE_H

namespace hector_uav_msgs
{
  typedef uint8_t ControlSource;

  enum {
    CONTROL_AUTONOMOUS = 0,
    CONTROL_REMOTE = 1,
    CONTROL_JOYSTICK = 2
  };

  template <typename InStream>
  static inline InStream& operator>>(InStream& in, ControlSource& value) {
    int temp;
    in >> temp;
    value = static_cast<ControlSource>(temp);
    return in;
  }

  template <typename OutStream>
  static inline OutStream& operator<<(OutStream& out, const ControlSource& value) {
    return out << static_cast<int>(value);
  }
}

#endif 
