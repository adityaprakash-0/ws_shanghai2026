# skill23_ws local_sysroot — Orbbec camera + studica drivers (not global apt).
# Source after: source /opt/ros/humble/setup.bash
#               source ~/skill23_ws/install/setup.bash

_SKILL23_LOCAL_SYSROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SKILL23_LOCAL_SYSROOT="$_SKILL23_LOCAL_SYSROOT"

_ORBBEC_PREFIX="$_SKILL23_LOCAL_SYSROOT/opt/ros/humble"
if [ -d "$_ORBBEC_PREFIX/lib" ]; then
  export AMENT_PREFIX_PATH="$_ORBBEC_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export CMAKE_PREFIX_PATH="$_ORBBEC_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$_ORBBEC_PREFIX/lib:$_SKILL23_LOCAL_SYSROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export LIBRARY_PATH="$_ORBBEC_PREFIX/lib:$_SKILL23_LOCAL_SYSROOT/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
  if [ -d "$_ORBBEC_PREFIX/bin" ]; then
    export PATH="$_ORBBEC_PREFIX/bin:$PATH"
  fi
fi

unset _ORBBEC_PREFIX _SKILL23_LOCAL_SYSROOT
