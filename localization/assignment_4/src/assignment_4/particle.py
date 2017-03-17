from assignment_3.geometry import *
from math import pi
import random
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches

#-------------------------------------------------------------------------------
# Generates a random pose in the map (in real world coordinates)
def random_particle(the_map):
  origin_x=the_map.info.origin.position.x
  origin_y=the_map.info.origin.position.y
  size_x = the_map.info.width
  size_y = the_map.info.height
  while True:
    x_map = random.randrange(0,size_x)
    y_map = random.randrange(0,size_y)
    theta = random.random()*2*pi
    if the_map.data[to_index(x_map,y_map,size_x)] == 0:
      break;
  (x_world,y_world) = to_world(x_map,y_map,origin_x,origin_y,size_x,size_y,the_map.info.resolution)
  return (x_world, y_world, theta)

#-------------------------------------------------------------------------------
# Generates a new particle from an old one by adding noise to it
def new_particle(particle, spatial_var, angle_var):
  x     = float(np.random.normal(particle[0],spatial_var,1))
  y     = float(np.random.normal(particle[1],spatial_var,1))
  theta = float(np.random.normal(particle[2],angle_var,1))
  theta = theta % pi - pi
  return (x,y,theta)
    
#-------------------------------------------------------------------------------
# Resamples the particles.
# NOTE: particle weights are not normalized i.e. it is not guaranteed that the 
# sum of all particle weights is 1

def resample(particles_weighted, n_particles):
  scores = [p_w[0] for p_w in particles_weighted]
  particles = []
  max_index = scores.index(max(scores))
  #print('length of particles_weighted'+str(len(scores)) + 'number of particles' + str(n_particles))
  scores = np.divide(scores,float(sum(scores))) #Normalized weights
  
  #print('scores = '+str(scores))
  #raw_input()

  #generate cummulative distribution function
  cdf = [0]
  for i in scores[0:len(scores)]:
    cdf.append(cdf[len(cdf)-1]+i)
  #print('cdf = '+str(cdf))
  print('cdf length = '+str(len(cdf)))
  #raw_input()
  s_var= 1.0 # percent
  a_var= 1.0 # fraction of pi
  spatial_var=math.sqrt(10*10*s_var/100*s_var/100)
  angle_var=math.pi/a_var

  #generate randon numbers to create a particle with uniform distribution
  U = np.random.uniform(0,1,n_particles)
  #find where it falls in cdf and generate a :particle in it!
  indx = 0
  for u in U:
    for indx in range(len(cdf)):
      #print('u = ' +str(u))
      if u >= cdf[indx] and u <= cdf[indx+1]:
        p = new_particle(particles_weighted[indx][1],spatial_var,angle_var)
        particles.append(p)
        break
  #particles.append(particles_weighted[max_index][1])
  #print('particles =  ' + str(len(particles)))
  return particles  

# ----------------------------------------------------------------------------
# Draw an occupancy grid
def draw_occupancy_grid(the_map, ax):

  for cellId in range(len(the_map.data)):

    # Get cell grid coordinates
    x = cellId // the_map.info.width
    y = cellId %  the_map.info.width

    # Get cell world coordinates
    (x, y) = to_world ( x, y,
                    the_map.info.origin.position.x,
                    the_map.info.origin.position.y,
                    the_map.info.width, the_map.info.height,
                    the_map.info.resolution)


    # Add patch
    res = the_map.info.resolution
    if the_map.data[cellId] == 100:
      patch = patches.Rectangle ( (x-res/2, y-res/2), res, res, color='k', alpha=0.5)
      ax.add_patch(patch)
    elif the_map.data[cellId] == 0:
      patch = patches.Rectangle ( (x-res/2, y-res/2), res, res, color='b', alpha=0.5)
      ax.add_patch(patch)

  None

# ----------------------------------------------------------------------------
# Draw scored particles. If no scores were assigned to the particles (i.e. all
# particles have score 0) then particles are drawn with score of 0.1
def draw_particles_scored(particles_weighted):

  # Check if scores are unassigned
  scoresAssigned = False
  for particle in particles_weighted:
    if particle[0] != 0.0:
      scoresAssigned  = True

  # Draw particles
  for ptclId in range(len(particles_weighted)):
    (x, y, theta) = particles_weighted[ptclId][1]

    mSize = 0.1
    if scoresAssigned:
      mSize = particles_weighted[ptclId][0] * 20

    plt.plot( [x,  x - math.sin(theta)*0.5],
          [y,  y + math.cos(theta)*0.5],
          'g', linewidth=mSize / 5)
    plt.plot(x, y, 'ro', markersize=mSize, markeredgecolor='r')
  None

#-------------------------------------------------------------------------------
# This function is called each interation after calculating the scores of the
# particles. Use it for debugging
def debug_call(particles_weighted, the_map):

  debug = False

  if not debug:
    return 

  # Initialize figure
  my_dpi = 96
  plt.figure(1, figsize=(800/my_dpi, 800/my_dpi), dpi=my_dpi)
  plt.cla()
  plt.xlim ( the_map.info.origin.position.x, the_map.info.origin.position.x + the_map.info.width )
  plt.ylim ( the_map.info.origin.position.y, the_map.info.origin.position.y + the_map.info.height )
  plt.gca().set_aspect('equal', adjustable='box')
  plt.xlabel('X world')
  plt.xlabel('Y world')
  ax = plt.axes()

  # Draw map
  draw_occupancy_grid(the_map, ax)

  # Draw particles
  draw_particles_scored(particles_weighted)

  # Show plot
  plt.draw()

  pause = True
  if pause:
    k = plt.waitforbuttonpress(1)
    while not k:
      k = plt.waitforbuttonpress(1)
  else:
    plt.waitforbuttonpress(1e-6)

None  