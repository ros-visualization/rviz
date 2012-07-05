# The interpolation mode to use during this step
uint8 interpolation_mode
uint8 LINEAR    = 0 # Positions will be linearly interpolated
uint8 SPHERICAL = 1 # Position and orientation will be interpolated in a spherical sense.

# When should this pose be reached?
duration time_from_start

# The frame-relative position for the camera.
geometry_msgs/PointStamped camera

# The frame-relative position for the target (or pivot for an Orbit controller).
# Note that a '0' time stamp will lock the target to the frame provided.
geometry_msgs/PointStamped target

# The frame-relative vector that maps to "up" in the view plane.
# The zero-vector will default to +Z in Fixed Frame
# Note that a '0' time stamp will lock the up vector to the frame provided.
geometry_msgs/Vector3Stamped up

