# Standard Header
std_msgs/Header header

# Mesh of the place
shape_msgs/Mesh mesh

# Representation of a plane, using the plane equation ax + by + cz + d = 0
shape_msgs/Plane coefficients

# Normal vector
geometry_msgs/Point32 normal

# Center point 
geometry_msgs/Point32 center

# Plane pose relative to the global reference frame
geometry_msgs/Transform pose

# Width and height of the bounding rectangle around the plane contours
float32[2] extents

# The polygon bounds of the plane
geometry_msgs/Polygon bounds