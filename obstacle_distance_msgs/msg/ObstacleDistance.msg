
std_msgs/Header header

# metadata for the map
nav_msgs/MapMetaData info

# The data (distances and indexes)is in row-major order, starting with (0,0).
# Cell (1, 0) will be listed second, representing the next cell in the x direction.
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).

# Distances array contains the real distances in meters
# of the closest obstacle.
float32[] distances

# Array indexes of the closest obstacle in the array.
# Cell to index:
# index = row * width + col
# Index to cell:
# row = floor(index/width)
# col = index % width
int32[] indexes