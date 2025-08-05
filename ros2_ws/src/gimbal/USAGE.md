# Gimbal Package Usage Guide

## Available Target Libraries

When using `find_package(gimbal REQUIRED)` in your CMakeLists.txt, the following target libraries are available:

### Core Libraries (No Qt6 required)
- `gimbal::gimbal_helpers` - Core helper functions and utilities
- `gimbal::usv_gimbal_controller` - USV-side gimbal controller
- `gimbal::gimbal_action` - Gimbal action classes (LockOn, Scan, etc.)

### GCS Libraries (Require Qt6)
- `gimbal::gcs_gimbal_controller` - GCS-side gimbal controller (needs Qt6::Core)
- `gimbal::gcs_gimbal_streamer` - Video streaming (needs Qt6::Core + FFmpeg)
- `gimbal::gimbal_image_provider` - QML image provider (needs Qt6::Quick)

## Example Usage in CMakeLists.txt

```cmake
find_package(gimbal REQUIRED)

# For USV applications (no Qt6 needed)
target_link_libraries(my_usv_app
    gimbal::gimbal_helpers
    gimbal::usv_gimbal_controller
    gimbal::gimbal_action
)

# For GCS applications (Qt6 required)
target_link_libraries(my_gcs_app
    gimbal::gimbal_helpers
    gimbal::gcs_gimbal_controller
    gimbal::gcs_gimbal_streamer
    gimbal::gimbal_image_provider
)
```

## Example Usage in C++

```cpp
// For USV applications
#include "gimbal/helpers.hpp"
#include "gimbal/usv/gimbal_controller.hpp"
#include "gimbal_action/lock_on.hpp"

// For GCS applications  
#include "gimbal/gcs/gimbal_controller.hpp"
#include "gimbal/gcs/gimbal_streamer.hpp"
```

## Common Mistakes

❌ **Wrong**: `gimbal::gcs::gimbal_controller`
✅ **Correct**: `gimbal::gcs_gimbal_controller`

❌ **Wrong**: `gimbal::gimbal_controller`
✅ **Correct**: `gimbal::gcs_gimbal_controller` or `gimbal::usv_gimbal_controller`

## Dependencies

The gimbal package automatically provides these transitive dependencies:
- rclcpp
- sensor_msgs
- custom_msgs
- OpenCV
- cv_bridge
- Qt6 (optional, only for GCS components)
