from queue import PriorityQueue
import numpy as np
import cv2
import time
import os
import math


# Function to create the grid space
def createGrid(height, width, bounding_location, padding = 0, wall_padding = 0, scale = 1):
  image = np.full((height, width, 3), 255, dtype=np.uint8)

  if wall_padding > 0:
    image = setWallPadding(image, wall_padding, 125)

  if padding > 0:
    image = setObstaclesAndTruePadding(image, bounding_location, padding, scale, 125)


  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  grid = np.full((height, width), 5 * height * width)
  grid[gray == 125] = -12
  grid[gray == 180] = -15
  grid[gray == 0] = -11
  grid = grid.flatten()
  return grid, image


# Function to add padding for walls
def setWallPadding(image, padding, value):
  height, width, _ = image.shape
  points = [
      (0, 0), (padding, height),
      (0, 0), (width, padding),
      (0, height - padding), (width, height),
      (width - padding, 0), (width, height),
  ]
  for i in range(0, len(points), 2):
    cv2.rectangle(image, points[i], points[i+1], (value, value, value), -1)
  return image


# Function to add obstacles and their padding
def setObstaclesAndTruePadding(image, bounding_location, padding, scale, value):
  if padding > 0:
    doPadding = True

  for obstacle in bounding_location:
    obstacle = np.array(obstacle, dtype=np.int32) * scale
    obstacle = obstacle.astype(np.int32)
    if len(obstacle) == 1: # Check again later
      cv2.circle(image, (obstacle[0][0], obstacle[0][1]), obstacle[0][2] + padding, (value, value, value), -1)
      cv2.circle(image, (obstacle[0][0], obstacle[0][1]), obstacle[0][2], (0, 0, 0), -1)
    else:
      if doPadding:
        x_small = x_big = y_small = y_big = -1
        for point in obstacle:
          if x_small > point[0] or x_small == -1:
            x_small = point[0]
          if x_big < point[0] or x_big == -1:
            x_big = point[0]
          if y_small > point[1] or y_small == -1:
            y_small = point[1]
          if y_big < point[1] or y_big == -1:
            y_big = point[1]
          cv2.circle(image, tuple((np.array(point))), padding, (value, value, value), -1)

        if x_small - padding > 0:
          x_small -= padding
        else:
          x_small = 0
        if x_big + padding < width:
          x_big += padding
        else:
          x_big = width
        if y_small - padding > 0:
          y_small -= padding
        else:
          y_small = 0
        if y_big + padding < height:
          y_big += padding
        else:
          y_big = height
        new_points = np.array([[x_small, y_small], [x_small, y_big], [x_big, y_big], [x_big, y_small]])
        cv2.fillPoly(image, pts=[new_points], color=(value, value, value))

        for point in obstacle:
          cv2.circle(image, tuple((np.array(point))), padding, (value, value, value), -1)
      cv2.fillPoly(image, pts=[obstacle], color=(0, 0, 0))

  return image


# Function to calculate Euclidean distance
def heuristic(node, goal):
    return ((node[0] - goal[0])**2 + (node[1] - goal[1])**2)**0.5
    # return distance.euclidean(node, goal) #try


# Set initial variables
start = time.time()

unscaled_clearance = int(input("\nEnter the obstacle clearance:"))
unscaled_robot_radius = 220
unscaled_robot_width = 287
unscaled_height = 2000
unscaled_width = 6000
unscaled_effective_padding = unscaled_robot_radius + unscaled_clearance

wheel_radius = 33
wheel_dist = 287

scale = 1/5 # needs to be fn of rpms

height = int(unscaled_height * scale) # y size
width = int(unscaled_width * scale) # x size
robot_radius = unscaled_robot_radius * scale
robot_width = unscaled_robot_width * scale

non_round_pad = unscaled_effective_padding * scale
effective_padding = math.ceil(non_round_pad)

rpm1 = int(int(input("\nEnter first RPM:")))
rpm2 = int(int(input("\nEnter second RPM:")))
dt = 0.1
tt = 1


if rpm1 > rpm2:
  bigger = rpm1
  smaller = rpm2
else:
  bigger = rpm2
  smaller = rpm1

goal_threshold = robot_radius / 2

# (left, right)
actions = [
    (0, bigger),
    (bigger, 0),
    (bigger, bigger),
    (0, smaller),
    (smaller, 0),
    (smaller, smaller),
    (bigger, smaller),
    (smaller, bigger),
]

timestep = 0

last_explored = last_explored_speed = -1

obstacle_file_path = ""

obstacle_bounding_boxes = [
    [(1500, 2000), (1500, 1000), (1750, 1000),  (1750, 2000)],
    [(2500, 1000), (2500, 0), (2750, 0), (2750, 1000)],
    [(4200, 1200, 600)],
    ]

circle_radius = 600 * scale
circle_x = 4200 * scale
circle_y = 1200 * scale

rect1_x_low = (1500 * scale) - non_round_pad
rect1_x_high = (1750 * scale) + non_round_pad
rect1_y_low = (1000 * scale) - non_round_pad
rect1_y_high = 2000 * scale

rect2_x_low = (2500 * scale) - non_round_pad
rect2_x_high = (2750 * scale) + non_round_pad
rect2_y_low = 0
rect2_y_high = (1000 * scale) + non_round_pad

open = PriorityQueue()
visited = []
visited_steps = []

if os.path.exists(obstacle_file_path) and os.path.isfile(obstacle_file_path):
  pass
else:
  grid, useless = createGrid(height, width, obstacle_bounding_boxes, effective_padding, effective_padding, scale)
  # grid = np.full((height*width), 5*height*width)
  backtrack_grid = np.full((height*width), -1)
  backtrack_action = np.full((height*width), -1)
  backtrack_path = np.full((height*width), -1)

end = time.time()
print(end - start)

precision = False

# Get starting position
valid = False
while not valid:
  starting_x = int(int(input("\nEnter starting x position:")) * scale)
  starting_y = int(int(input("\nEnter starting y position:")) * scale)
  starting_theta = int(input("\nEnter starting theta position:")) % 360

  current_pos = starting_x + (width * starting_y)
  try:
    if grid[current_pos] == 5 * height * width:
      grid[current_pos] = 0
      valid = True
    else:
      print("\nStarting position invalid, obstacle exists, Enter again\n")
  except:
    print("\nStarting position invalid, obstacle exists, Enter again\n")


# Get goal position
valid = False
while not valid:
  goal_x = int(int(input("\nEnter goal x position:")) * scale)
  goal_y = int(int(input("\nEnter goal y position:")) * scale)
  goal_index = goal_x + (width * goal_y)

  try:
    if grid[goal_index] == 5 * height * width:
      valid = True
    else:
      print("\nGoal position invalid, obstacle exists, Enter again\n")

  except:
    print("\nGoal position invalid, obstacle exists, Enter again\n")


# Setup costs of each action
costs = [0 for i in range(len(actions))]

for i, action in enumerate(actions):
  old_x = 0
  old_y = 0
  new_theta = starting_theta
  for j in range(int(tt/dt)):
    new_theta += ((action[1] - action[0]) * 2 * math.pi * dt * wheel_radius / (unscaled_robot_width * 60))
    new_x = old_x + ((action[0] + action[1]) * math.cos(new_theta)) * math.pi * dt * wheel_radius * scale / (60)
    new_y = old_y + ((action[0] + action[1]) * math.sin(new_theta)) *math.pi * dt * wheel_radius * scale / (60)

    costs[i] += heuristic((old_x, old_y), (new_x, new_y))

    old_x = new_x
    old_y = new_y



stop_condition = 2

visit_count = 0

recording = False

# Start A*
open.put(( heuristic((starting_x, starting_y), (goal_x, goal_y)), -1, current_pos, starting_theta))

goal_distance = goal_threshold + 1

goal_found = False
start = time.time()
while not open.empty():
    current_cost, current_action, current_pos, current_theta = open.get()

    if not grid[current_pos] == -13:
        timestep += 1
        grid[current_pos] = -13

        x_pos = int(current_pos % width)
        y_pos = int((current_pos - (current_pos % width))/width)
        current_distance = heuristic((x_pos, y_pos), (goal_x, goal_y))

        if current_distance <= goal_threshold:
          if goal_distance > current_distance:
            print(current_distance)
            goal_distance = current_distance
            last_explored = current_pos
            print("Goal path found")
            goal_found = True

        if goal_found:
          continue

        for i, action in enumerate(actions):
          new_x = x_pos
          new_y = y_pos
          new_theta = current_theta
          steps = []
          steps.append((new_x, new_y))
          skip = False
          for j in range(int(tt/dt)):
            new_theta += ((action[1] - action[0]) * 2 * math.pi * dt * wheel_radius / (unscaled_robot_width * 60))
            new_x += ((action[0] + action[1]) * math.cos(new_theta)) * math.pi * dt * wheel_radius * scale / (60)
            new_y += ((action[0] + action[1]) * math.sin(new_theta)) *math.pi * dt * wheel_radius * scale / (60)
            int_x = round(new_x)
            int_y = round(new_y)
            new_pos = int_x + (width * int_y)

            if new_x < 0 or new_y < 0 or new_x >= width or new_y >= height or new_pos >= height*width:
                skip = True
                break

            ### Obstacle Check
            if grid[new_pos] < -10:
                skip = True
                break

            steps.append((int_x, int_y))
          if not skip:

            new_distance = heuristic((new_x, new_y), (goal_x, goal_y))
            cost = current_cost + costs[i] - current_distance + new_distance

            if grid[new_pos] > cost:
                grid[new_pos] = cost
                backtrack_grid[new_pos] = current_pos
                backtrack_action[new_pos] = i
                backtrack_path[new_pos] = visit_count
                open.put((cost, i, new_pos, new_theta))

                visited_steps.append(steps)

                visit_count += 1

    # if timestep == stop_condition:
    #   break

print(f"Execution time: {time.time() - start} seconds")


start = time.time()

# #Print action sets
# action_sets = []
# index = last_explored if last_explored != -1 else last_explored_speed
# while backtrack_grid[index] > 0:
#     action_index = backtrack_action[index]
#     action_sets.append(actions[action_index])
#     index = backtrack_grid[index]

# action_sets.reverse()



# Display or record video
if recording:
    size = (width, height)
    fps = 90
    record = cv2.VideoWriter('final_video.avi', cv2.VideoWriter_fourcc(*'MJPG'), fps, size)

    path = []
    path_action = []
    path_steps = []
    if not last_explored == -1:
        index = last_explored
    else:
        index = last_explored_speed
    while backtrack_grid[index] > 0:
        x_pos = int(index % width)
        y_pos = int((index - (index % width))/width)

        path.append((x_pos, y_pos))
        path_action.append(actions[backtrack_action[index]])
        path_steps.append(visited_steps[backtrack_path[index]])

        index = backtrack_grid[index]
    path.append((starting_x, starting_y))
    path.reverse()
    path_action.reverse()

    # Grid generation
    grid, image = createGrid(height, width, obstacle_bounding_boxes, int(math.ceil(unscaled_clearance * scale)), int(math.ceil(unscaled_clearance * scale)), scale)


    visited_length = len(visited_steps)
    step_size = int(visited_length / (fps * 2 * 6)) + 1

    # Show exploration
    for i, steps in enumerate(visited_steps):
        for j in range(0, 10):
            cv2.line(image, steps[j], steps[j+1], (125, 255, 125), 2)

        if i % step_size == 0 or i < 4 * fps:
            cv2.circle(image, (goal_x, goal_y), int(goal_threshold), (255, 0, 0), 2)
            image = cv2.flip(image, 0)
            image = np.uint8(image)
            record.write(image)
            image = cv2.flip(image, 0)
    
    # Show final path
    for steps in path_steps:
        for i in range(0, 10):
            cv2.line(image, steps[i], steps[i+1], (0, 0, 255), 2)

    cv2.circle(image, (goal_x, goal_y), int(goal_threshold), (255, 0, 0), 2)
    
    image = cv2.flip(image, 0)
    image = np.uint8(image)
    for i in range(90):
        record.write(image)
    
    record.release()

    print(f"Video Saving: {time.time() - start} seconds")

    # Display video of full algorithm
    cap = cv2.VideoCapture("final_video.avi")

    if (cap.isOpened()== False):
        print("Error opening video stream or file")

    while cap.isOpened():
        ret, frame = cap.read()

        if ret == True:
            cv2.imshow("A*", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        else:
            break

    cap.release()
else:
    path = []
    path_action = []
    path_steps = []
    if not last_explored == -1:
        index = last_explored
    else:
        index = last_explored_speed
    while backtrack_grid[index] > 0:
        x_pos = int(index % width)
        y_pos = int((index - (index % width))/width)

        path.append((x_pos, y_pos))
        path_action.append(actions[backtrack_action[index]])
        path_steps.append(visited_steps[backtrack_path[index]])

        index = backtrack_grid[index]
    path.append((starting_x, starting_y))
    path.reverse()
    path_action.reverse()

    grid_i, gray = createGrid(height, width, obstacle_bounding_boxes, int(math.ceil(unscaled_clearance * scale)), int(math.ceil(unscaled_clearance * scale)), scale)

    image = gray.copy()

    ### Visited Show
    for steps in visited_steps:
        for i in range(0, 10):
            cv2.line(image, steps[i], steps[i+1], (125, 255, 125), 2)


    ### Final Path Show
    for steps in path_steps:
        for i in range(0, 10):
            cv2.line(image, steps[i], steps[i+1], (0, 0, 255), 2)


    cv2.circle(image, (goal_x, goal_y), int(goal_threshold), (255, 0, 0), 2)
    image = cv2.flip(image, 0)
    image = np.uint8(image)

    cv2.imshow("Final", cv2.resize(image, (1200, 400)))
    cv2.waitKey(0)

cv2.destroyAllWindows()

print("Action Sets (RPM settings for each step):")
for action in path_action:
    print(action)