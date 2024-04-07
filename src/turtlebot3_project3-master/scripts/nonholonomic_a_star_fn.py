from queue import PriorityQueue
import numpy as np
import cv2
import time
import os
import math


# Function to create the grid space
def createGrid(height, width, bounding_location, padding = 0, wall_padding = 0, scale = 1):
  image = np.full((height, width, 3), 255, dtype=np.uint8)

  # Add wall padding
  if wall_padding > 0:
    image = setWallPadding(image, wall_padding, 125)

  # Add padding and obstacles using half planes and semi algebraic equations
  for obstacle in bounding_location:
    obstacle = np.array(obstacle, dtype=np.int32) * scale
    obstacle = obstacle.astype(np.int32)
    if len(obstacle) == 1:
        center_x = obstacle[0][0]
        center_y = obstacle[0][1]
        radius = obstacle[0][2]

        x_small = center_x - radius
        x_big = center_x + radius
        y_small = center_y - radius
        y_big = center_y + radius

        x_small_pad = x_small
        x_big_pad = x_big
        y_small_pad = y_small
        y_big_pad = y_big

        if x_small - padding > 0:
            x_small_pad -= padding
        else:
            x_small_pad = 0
        if x_big + padding < width:
            x_big_pad += padding
        else:
            x_big_pad = width
        if y_small - padding > 0:
            y_small_pad -= padding
        else:
            y_small_pad = 0
        if y_big + padding < height:
            y_big_pad += padding
        else:
            y_big_pad = height
        
        for x in range(x_small_pad, x_big_pad):
            for y in range(y_small_pad, y_big_pad):
                if heuristic((x, y), (center_x, center_y)) <= radius + padding + 1:
                    image[y, x] = (125, 125, 125)

        for x in range(x_small, x_big):
            for y in range(y_small, y_big):
                if heuristic((x, y), (center_x, center_y)) <= radius:
                    image[y, x] = (0, 0, 0)

    else:
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

            x_small_pad = x_small
            x_big_pad = x_big
            y_small_pad = y_small
            y_big_pad = y_big

            if x_small - padding > 0:
                x_small_pad -= padding
            else:
                x_small_pad = 0
            if x_big + padding < width:
                x_big_pad += padding
            else:
                x_big_pad = width
            if y_small - padding > 0:
                y_small_pad -= padding
            else:
                y_small_pad = 0
            if y_big + padding < height:
                y_big_pad += padding
            else:
                y_big_pad = height

            image[y_small_pad:y_big_pad, x_small_pad:x_big_pad] = (125, 125, 125)
            image[y_small:y_big, x_small:x_big] = (0, 0, 0)


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
    image[points[i][1]:points[i+1][1], points[i][0]:points[i+1][0]] = (value, value, value)
  return image


# Function to calculate Euclidean distance
def heuristic(node, goal):
    return ((node[0] - goal[0])**2 + (node[1] - goal[1])**2)**0.5
    # return distance.euclidean(node, goal) #try


# A* algorithm optimal path generation
def runAStar(unscaled_height, unscaled_width, unscaled_robot_radius, unscaled_clearance, unscaled_robot_width, wheel_radius, starting_x, starting_y, starting_theta, goal_x, goal_y, rpm1, rpm2, scale):
    # Set initial variables
    start = time.time()

    unscaled_effective_padding = unscaled_robot_radius + unscaled_clearance

    starting_x = int(starting_x * scale)
    starting_y = int(starting_y * scale)

    goal_x = int(goal_x * scale)
    goal_y = int(goal_y * scale)

    height = int(unscaled_height * scale) # y size
    width = int(unscaled_width * scale) # x size
    robot_radius = unscaled_robot_radius * scale
    robot_width = unscaled_robot_width * scale

    non_round_pad = unscaled_effective_padding * scale
    effective_padding = math.ceil(non_round_pad)


    dt = 0.1
    tt = 1

    if rpm1 > rpm2:
        bigger = rpm1
        smaller = rpm2
    else:
        bigger = rpm2
        smaller = rpm1

    goal_threshold = robot_radius / 2

    bigger = bigger * math.pi * wheel_radius / 60
    smaller = smaller * math.pi * wheel_radius / 60

    # (left, right) actions
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

    last_explored = -1

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
    visited_steps = []

    print("Building Obstacle Space")

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


    # Setup costs of each action
    costs = [0 for i in range(len(actions))]

    for i, action in enumerate(actions):
        old_x = 0
        old_y = 0
        new_theta = starting_theta
        for j in range(int(tt/dt)):
            new_theta += ((action[1] - action[0]) * 2 * dt / (unscaled_robot_width))
            new_x = old_x + ((action[0] + action[1]) * math.cos(new_theta)) * dt * scale
            new_y = old_y + ((action[0] + action[1]) * math.sin(new_theta)) * dt * scale

            costs[i] += heuristic((old_x, old_y), (new_x, new_y))

            old_x = new_x
            old_y = new_y


    visit_count = 0


    # Start A*
    current_pos = starting_x + (width * starting_y)
    final_goal_pos = goal_x + (width * goal_y)

    if grid[current_pos] < 0 or grid[final_goal_pos] < 0:
        return False, False

    initial_distance = heuristic((starting_x, starting_y), (goal_x, goal_y))

    open.put(( initial_distance, -1, current_pos, starting_theta))

    goal_distance = goal_threshold + 1

    progress = 0
    print("Exploration Progress: 0%")

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
                if progress == 3:
                    print("Exploration Progress: 99%")
                    progress = 4
                if goal_distance > current_distance:
                    print(current_distance)
                    goal_distance = current_distance
                    last_explored = current_pos
                    print("Goal path found")
                    goal_found = True
            elif progress == 0 and current_distance/initial_distance < 0.75:
                print("Exploration Progress: 25%")
                progress = 1
            elif progress == 1 and current_distance/initial_distance < 0.5:
                print("Exploration Progress: 50%")
                progress = 2
            elif progress == 2 and current_distance/initial_distance < 0.25:
                print("Exploration Progress: 75%")
                progress = 3

            if goal_found:
                continue

            # Generate new nodes
            for i, action in enumerate(actions):
                new_x = x_pos
                new_y = y_pos
                new_theta = current_theta
                steps = []
                steps.append((new_x, new_y))
                skip = False
                for j in range(int(tt/dt)):
                    new_theta += ((action[1] - action[0]) * 2 * dt / (unscaled_robot_width))
                    new_x += ((action[0] + action[1]) * math.cos(new_theta)) * dt * scale
                    new_y += ((action[0] + action[1]) * math.sin(new_theta)) * dt * scale
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

    print(f"Execution time: {time.time() - start} seconds")

    path_action = []
    path_steps = []
    index = last_explored
    while backtrack_grid[index] > 0:
        x_pos = int(index % width)
        y_pos = int((index - (index % width))/width)

        path_action.append(actions[backtrack_action[index]])
        path_steps.append(visited_steps[backtrack_path[index]])

        index = backtrack_grid[index]
    path_action.reverse()

    grid_i, gray = createGrid(height, width, obstacle_bounding_boxes, int(math.ceil(unscaled_clearance * scale)), int(math.ceil(unscaled_clearance * scale)), scale)

    image = gray.copy()

    ### Final Path Show
    for steps in path_steps:
        for i in range(0, 10):
            cv2.line(image, steps[i], steps[i+1], (0, 0, 255), 2)


    cv2.circle(image, (goal_x, goal_y), int(goal_threshold), (255, 0, 0), 2)
    image = cv2.flip(image, 0)
    image = np.uint8(image)

    print("Image of final path will be displayed after 5 seconds, enter any key to exit image and continue application")
    time.sleep(5)

    cv2.imshow("Final", cv2.resize(image, (1200, 400)))
    cv2.waitKey(0)

    cv2.destroyAllWindows()

    return path_action, path_steps


# Create a more optimized path for ROS
def optimizePath(path_actions):
    optimized_path = []
    count = -1
    old_action = None

    # Combine same actions into one
    for action in path_actions:
        if action == old_action:
            optimized_path[count][2] += 1
        else:
            optimized_path.append([action[0], action[1], 1])
            count += 1
            if old_action == None:
                optimized_path[0][2] = 1.25
            old_action = action
    
    return optimized_path