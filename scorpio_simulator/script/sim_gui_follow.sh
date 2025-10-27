#!/bin/zsh

# Set user camera follow the robot in ignition-gazebo

x=-1.0
y=0.0
z=1.6

ign service -s /gui/follow \
  -r "data: 'scorpio'" \
  --reqtype ignition.msgs.StringMsg \
  --reptype ignition.msgs.Boolean \
  --timeout 1000

ign service -s /gui/follow/offset \
  -r "x: $x, y: $y, z: $z" \
  --reqtype ignition.msgs.Vector3d \
  --reptype ignition.msgs.Boolean \
  --timeout 1000
