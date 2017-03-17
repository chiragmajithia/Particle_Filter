import math

# ------------------------------------------------------------------------------
# Convert world coordinate to grid coordinate
# ------------------------------------------------------------------------------
def to_grid(x, y, origin_x, origin_y, size_x, size_y, resolution):
  x_grid = (x - origin_x)/resolution;
  y_grid = (y - origin_y)/resolution;
  if (x - origin_x) >= size_x*resolution or x_grid < 0:
  	return None;
  elif (y - origin_y) >= size_y*resolution or y_grid < 0:
  	return None;
  else:
  	pos = (int(x_grid),int(y_grid));
  	return pos;
  return None
    
# ------------------------------------------------------------------------------
# Convert grid coordinate to world coordinate
# ------------------------------------------------------------------------------
def to_world(gx, gy, origin_x, origin_y, size_x, size_y, resolution):
  x_world = gx * resolution + origin_x;
  y_world = gy * resolution + origin_y;
  x_world = x_world + resolution/2;
  y_world = y_world + resolution/2;
  return (x_world,y_world)
  return None
  
# ------------------------------------------------------------------------------
# Convert grid coordinate to map index
# ------------------------------------------------------------------------------
def to_index(gx, gy, size_x):
  return gy * size_x + gx

# ------------------------------------------------------------------------------
# Given two integer coordinates return a list of coordinates of a line between 
# the two points.
# ------------------------------------------------------------------------------
def bresenham(x0, y0, x1, y1):

    # Setup initial conditions
    dx = x1 - x0
    dy = y1 - y0
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
        swapped = True
 
    # Recalculate differentials
    dx = x1 - x0
    dy = y1 - y0
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y0 < y1 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y0
    points = []
    for x in range(x0, x1 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points