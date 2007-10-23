
# Create the extension library
ADD_LIBRARY(Extensions_FixedTimeStepPhysics
  Physics/FixedTimeStepPhysics.cpp
  Physics/RigidBox.cpp
)

TARGET_LINK_LIBRARIES(Extensions_FixedTimeStepPhysics
  Extensions_AccelerationStructures
)