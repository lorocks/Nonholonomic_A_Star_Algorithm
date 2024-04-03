from queue import PriorityQueue
import numpy as np
import cv2
import time
import os
import math
import cv2 

# Fucntion to create the grid space
def createGrid(height, width, bounding_location, padding = 0, wall_padding = 0, travel_padding = 0, scale = 1):
  image = np.full((height, width, 3), 255, dtype=np.uint8)

  if travel_padding > 0:
    image = setTravelPadding(image, travel_padding, bounding_location, scale, 180)

  if wall_padding > 0:
    image = setWallPadding(image, wall_padding, 125)

  if padding > 0:
    image = setObstaclesAndTruePadding(image, bounding_location, padding, scale, 125)


  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  grid = np.full((height, width), 5 * height * width)
  grid[gray == 125] = -12
  grid[gray == 180] = -15
  grid[gray == 0] = -11
  grid = grid.reshape(-1)
  return grid, gray

# Function to add padding for step size
def setTravelPadding(image, padding, obstacles, scale, value):
  for obstacle in obstacles:
    obstacle = np.array(obstacle, dtype=np.int32) * scale
    for point in obstacle:
      cv2.circle(image, point, padding, (value, value, value), -1)
  
  return image

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
    paddings = [
        [padding, padding],
                [-padding, -padding],
     [padding, -padding],
      [-padding, padding],
                [0, padding],
                [0, -padding],
                [padding, 0],
                [-padding, 0]
                ]


  for obstacle in bounding_location:
    obstacle = np.array(obstacle, dtype=np.int32) * scale
    if len(obstacle) == 2:
      if doPadding:
        for pad in paddings:
          cv2.rectangle(image, (obstacle[0][0] - pad[0], obstacle[0][1] - pad[1]), (obstacle[1][0] + pad[0], obstacle[1][1] + pad[1]), (value, value, value), -1)
        points = [(obstacle[0][0], obstacle[0][1]), (obstacle[1][0], obstacle[1][1]), ( obstacle[0][0], obstacle[1][1]), ( obstacle[1][0],  obstacle[0][1])]
        for point in points:
          cv2.circle(image, point, padding, (value, value, value), -1)
      cv2.rectangle(image, obstacle[0], obstacle[1], (0, 0, 0), -1)
    else:
      if doPadding:
        for pad in paddings:
          length = len(obstacle)
          arrr = np.full((length, 2), pad)
          cv2.fillPoly(image, pts=[np.subtract(obstacle, arrr)], color=(value, value, value))
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

unscaled_robot_radius = int(input("\nEnter the robot radius:"))
unscaled_clearance = int(input("\nEnter the obstacle clearance:"))
unscaled_height = 500
unscaled_width = 1200
unscaled_effective_padding = unscaled_robot_radius + unscaled_clearance

scale = 1 # test scale to see speed of iteration, (9 is like 0.0966 and 0.088)

height = unscaled_height * scale # y size
width = unscaled_width * scale # x size
effective_padding = unscaled_effective_padding * scale
travel_dist = int(input("\nEnter step size:"))
goal_threshold = 1.5
padding = int(((travel_dist * (3 - (3)**0.5)/4) + (unscaled_effective_padding)) * scale)
angles = [ 60, 30, 0, -30, -60 ]
angle_indices = {
  0 : 0,
  30 : 1,
  60 : 2,
  90 : 3,
  120 : 4,
  150 : 5,
  180 : 6,
  210 : 7,
  240 : 8,
  270 : 9,
  300 : 10,
  330 : 11,
}
timestep = 0

last_explored = last_explored_speed = -1

recording = True

obstacle_file_path = ""

obstacle_bounding_boxes = [
    # [[175, 100], [100, 500] ], [[275, 400], [350, 0]],
    [(100, 100), (100, 500), (175, 500), (175, 100)],
    [(275, 0), (275, 400), (350, 400), (350, 0)],
    [[650 - (75*(3**0.5)), 325], [650 - (75*(3**0.5)), 175], [650, 100], [650 + (75*(3**0.5)), 175], [650 + (75*(3**0.5)), 325], [650, 400]],
    [[900, 450], [1100, 450], [1100, 50], [900, 50], [900, 125], [1020, 125], [1020, 375], [900, 375]],
    ]


open = PriorityQueue()
visited = []

if os.path.exists(obstacle_file_path) and os.path.isfile(obstacle_file_path):
  pass
else:
  grid, useless = createGrid(height, width, obstacle_bounding_boxes, effective_padding, effective_padding, padding, scale)
  backtrack_grid = np.full((height*width), -1)

if travel_dist < 2:
   travel_dist = 2
elif travel_dist > 30:
   travel_dist = 30

end = time.time()
print(end - start)

precision = False

# Get starting position
valid = False
while not valid:
  starting_x = int(input("\nEnter starting x position:")) * scale
  starting_y = int(input("\nEnter starting y position:")) * scale
  starting_theta = int(input("\nEnter starting theta position:")) % 360

  current_pos = starting_x + (width * starting_y)
  try:
    if grid[current_pos] == 5 * height * width:
      if not starting_theta % 30 == 0:
        print("\nInvalid angle input")
        continue
      grid[current_pos] = 0
      valid = True
    else:
      print("\nStarting position invalid, obstacle exists, Enter again\n")
  except:
    print("\nStarting position invalid, obstacle exists, Enter again\n")


# Get goal position
valid = False
while not valid:
  goal_x = int(input("\nEnter goal x position:")) * scale
  goal_y = int(input("\nEnter goal y position:")) * scale
  goal_index = goal_x + (width * goal_y)
  goal_theta = int(input("\nEnter goal theta position:")) % 360

  try:
    if grid[goal_index] == 5 * height * width:
      if not goal_theta % 30 == 0:
        print("\nInvalid goal angle")
        continue
      valid = True
    else:
      print("\nGoal position invalid, obstacle exists, Enter again\n")
  except:
    print("\nGoal position invalid, obstacle exists, Enter again\n")


# Start A*
open.put(( heuristic((starting_x, starting_y), (goal_x, goal_y)), current_pos, starting_theta))

goal_found = False
start = time.time()
while not open.empty() and not goal_found:
    current_cost, current_pos, current_theta = open.get()

    # Condition to perform optimal path finding with angle
    if precision:
        if not grid[angle_indices[current_theta], current_pos] == -13:
            timestep += 1
            grid[angle_indices[current_theta], current_pos] = -13

            x_pos = int(current_pos % width)
            y_pos = int((current_pos - (current_pos % width))/width)
            current_distance = heuristic((x_pos, y_pos), (goal_x, goal_y))

            if current_distance <= goal_threshold and last_explored_speed == -1: # and current_theta == goal_theta
                last_explored_speed = current_pos
                angle_speed = current_theta

            for angle in angles:
                new_theta = (current_theta + angle) % 360
                new_x = x_pos + int(((travel_dist * math.cos((new_theta)*math.pi / 180) * scale) ))
                new_y = y_pos + int(((travel_dist * math.sin((new_theta)*math.pi / 180) * scale) ))
                new_pos = new_x + (width * new_y)

                if new_x < 0 or new_y < 0 or new_x >= width or new_y >= height or new_pos >= height*width:
                    continue
                
                if grid[angle_indices[new_theta], new_pos] < -10:
                    continue
                
                new_distance = heuristic((new_x, new_y), (goal_x, goal_y))      
                cost = current_cost + (travel_dist * scale) - current_distance + new_distance
            #  orgrid[new_pos] > cost
                if grid[angle_indices[new_theta], new_pos] > cost:
                    grid[angle_indices[new_theta], new_pos] = cost
                    backtrack_grid[angle_indices[new_theta], new_pos] = (current_pos * 100) + angle_indices[current_theta]
                    open.put((cost, new_pos, new_theta))

                    visited.append((new_x, new_y))
                    visited.append((x_pos, y_pos))

                    if new_distance <= goal_threshold and new_theta == goal_theta:
                        print(new_distance)
                        last_explored = new_pos
                        print("Goal path found")
                        goal_found = True
    # Condition to perform optimal path finding
    else:
        if not grid[current_pos] == -13:
            timestep += 1
            grid[current_pos] = -13

            x_pos = int(current_pos % width)
            y_pos = int((current_pos - (current_pos % width))/width)
            current_distance = heuristic((x_pos, y_pos), (goal_x, goal_y))

            for angle in angles:
                new_theta = (current_theta + angle) % 360
                new_x = x_pos + int(((travel_dist * math.cos((new_theta)*math.pi / 180) * scale) ))
                new_y = y_pos + int(((travel_dist * math.sin((new_theta)*math.pi / 180) * scale) ))
                new_pos = new_x + (width * new_y)

                if new_x < 0 or new_y < 0 or new_x >= width or new_y >= height or new_pos >= height*width:
                    continue
                
                if grid[new_pos] < -10:
                    continue
                
                new_distance = heuristic((new_x, new_y), (goal_x, goal_y))      
                cost = current_cost + (travel_dist * scale) - current_distance + new_distance

                if grid[new_pos] > cost:
                    grid[new_pos] = cost
                    backtrack_grid[new_pos] = (current_pos * 100) + angle_indices[current_theta]
                    open.put((cost, new_pos, new_theta))

                    visited.append((new_x, new_y))
                    visited.append((x_pos, y_pos))
            # Switch to optimal path finding with angle
            if current_distance <= 5 * travel_dist:
                precision = True
                grid = np.array([grid for i in range(12)]) # changes
                backtrack_grid = np.array([backtrack_grid for i in range(12)]) # changes
               
      


print(f"Execution time: {time.time() - start} seconds")


start = time.time()
# Display or record video
if recording:
    size = (width, height)
    fps = 90
    record = cv2.VideoWriter('final_video.avi', cv2.VideoWriter_fourcc(*'MJPG'), fps, size)

    path = []
    if not last_explored == -1:
      index = last_explored
      index_angle = angle_indices[goal_theta]
    else:
      index = last_explored_speed
      index_angle = angle_speed
    last = 0
    while backtrack_grid[index_angle, index] > 0:
        x_pos = int(index % width)
        y_pos = int((index - (index % width))/width)

        # path.append((x_pos / scale, y_pos / scale))
        path.append((x_pos, y_pos))
        # print(x_pos, y_pos)
        # print(((x_pos - goal_x)**2 + (y_pos - goal_y)**2)**0.5)
        value = backtrack_grid[index_angle, index]
        index_angle = value % 100
        index = int(value / 100)
    path.append((starting_x, starting_y))
    path.reverse()

    grid, gray = createGrid(height, width, obstacle_bounding_boxes, unscaled_clearance * scale, unscaled_clearance * scale, 0, scale)

    image = np.full((height, width, 3), (224, 224, 224))
    image[gray == 125] = (125, 125, 125)
    image[gray == 0] = (0, 0, 0)
    image = np.ascontiguousarray(image, dtype=np.uint8)

    visited_length = len(visited)
    step_size = int(visited_length / (fps * 2 * 6))
    print(visited_length, step_size)

    for i in range(0, visited_length, 2):
      cv2.line(image, visited[i], visited[i+1], (125, 255, 125), 2)

      if i % step_size == 0 or i < 4 * fps:
        cv2.circle(image, (goal_x, goal_y), int(goal_threshold), (255, 0, 0), scale)
        image = cv2.flip(image, 0)
        image = np.uint8(image)
        record.write(image)
        image = cv2.flip(image, 0)
    
    last = -1
    for point in path:
        if last == -1:
            last = point
        else:
            cv2.line(image, last, point, (0, 0, 255), 2)
            last = point
    cv2.circle(image, (goal_x, goal_y), int(travel_dist/2), (255, 0, 0), scale)
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
    if not last_explored == -1:
      index = last_explored
      index_angle = angle_indices[goal_theta]
    else:
      index = last_explored_speed
      index_angle = angle_indices[angle_speed]
    last = 0
    while backtrack_grid[index_angle, index] > 0:
        x_pos = int(index % width)
        y_pos = int((index - (index % width))/width)

        # path.append((x_pos / scale, y_pos / scale))
        path.append((x_pos, y_pos))
        # print(x_pos, y_pos)
        # print(((x_pos - goal_x)**2 + (y_pos - goal_y)**2)**0.5)
        value = backtrack_grid[index_angle, index]
        index_angle = value % 100
        if index_angle > 11:
          index = value
          index_angle = 0
        else:
          index = int(value / 100)
    path.append((starting_x, starting_y))
    path.reverse()

    grid, gray = createGrid(height, width, obstacle_bounding_boxes, unscaled_clearance * scale, unscaled_clearance * scale, 0, scale)

    image = np.full((height, width, 3), (224, 224, 224))
    image[gray == 125] = (125, 125, 125)
    image[gray == 0] = (0, 0, 0)
    image = np.ascontiguousarray(image, dtype=np.uint8)

    last = -1
    for point in path:
        if last == -1:
            last = point
        else:
            cv2.line(image, last, point, (0, 0, 255), 2)
            last = point
    cv2.circle(image, (goal_x, goal_y), int(travel_dist/2), (255, 0, 0), scale)
    image = cv2.flip(image, 0)
    image = np.uint8(image)

    cv2.imshow("Final", cv2.resize(image, (1200, 500)))
    cv2.waitKey(0)

cv2.destroyAllWindows()