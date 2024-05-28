#!/usr/bin/env python3
import numpy as np
import math
from traffic_light_detector import TrafficLightState
from utils import transform_world_to_ego_frame, rotate_z, rotate_x,to_rot
# State machine states
FOLLOW_LANE = 0
DECELERATE_TO_STOP = 1
STOP_FOR_OBSTACLES = 2

#constants
DIST_TO_TL          = 15 # distance (m) for evaluating nearest traffic light to stop at 
DIST_STOP_TL        = 4 # minimum distance (m) from the traffic light to which we want to stop for a RED light
TL_Y_POS            = 3.5 # distance (m) on the Y axis for the traffic light check.

# Stop speed threshold
STOP_THRESHOLD = 0.02
# Number of cycles before moving from stop sign.
STOP_COUNTS = 10
#traffic light states 
GREEN = 0
YELLOW = 1
RED = 2

class BehaviouralPlanner:
    def __init__(self, lookahead, lead_vehicle_lookahead):
        self._lookahead                     = lookahead
        #self._traffic_light_fences          = traffic_lights_fences
        #self._traffic_light_visited         = [False]*len(self._traffic_light_fences)
        #self._stop_for_traffic_light        = False
        self._light_state                   = TrafficLightState.NO_TL
        self._follow_lead_vehicle_lookahead = lead_vehicle_lookahead
        self._state                         = FOLLOW_LANE
        self._follow_lead_vehicle           = True
        self._obstacle_on_lane              = False
        self._goal_state                    = [0.0, 0.0, 0.0]
        self._goal_index                    = 0
        #self._stop_count                    = 0
        self._lookahead_collision_index     = 0
        #self._traffic_light_state           = {}
        #self._tl_id                         = None
        #self._pedestrian_found              = False

        self._depth_images                  = None
        self._current_box                   = None # current detected traffic light box -> format: {camera_name:box}
        self._cameras_params                = {} # parameters of depth camera used in camera projection geometry -> format: {name:params}
        self._inv_intrinsic_matrices        = {"CameraDEPTH_TL":None, "CameraDEPTH_FRONT":None} # inverse of intrinsic matrices used in camera projection geometry

        # Rotation matrix to align image frame to camera frame
        rotation_image_camera_frame = np.dot(rotate_z(-90 * math.pi /180),rotate_x(-90 * math.pi /180))

        image_camera_frame = np.zeros((4,4))
        image_camera_frame[:3,:3] = rotation_image_camera_frame
        image_camera_frame[:, -1] = [0, 0, 0, 1]

        # Lambda Function for transformation of image frame in camera frame 
        self._image_to_camera_frame = lambda object_camera_frame: np.dot(image_camera_frame, object_camera_frame)

    def set_lookahead(self, lookahead):
        self._lookahead = lookahead

    def set_lightstate(self, state):
        self._light_state=state

    def set_depth_imgs(self, depth_imgs):
        self._depth_images = depth_imgs
        
    def set_current_box(self, box):
        if box!=[]: self._current_box = box
    
    def set_cameras_params(self, depth_info):
        self._cameras_params = depth_info

        for k in list(depth_info.keys()):
            # Calculate Inverse Intrinsic Matrix for both depth cameras
            f = self._cameras_params[k]["w"] /(2 * math.tan(self._cameras_params[k]["fov"] * math.pi / 360))
            Center_X = self._cameras_params[k]["w"] / 2.0
            Center_Y = self._cameras_params[k]["h"] / 2.0

            intrinsic_matrix = np.array([[f, 0, Center_X],
                                        [0, f, Center_Y],
                                        [0, 0, 1]])

            self._inv_intrinsic_matrices[k]=np.linalg.inv(intrinsic_matrix)
    
    def get_tl_stop_goal(self, waypoints, ego_state, closest_index, goal_index):
        """
        Checks whether the vehicle is near a traffic light and returns a waypoint index near to it or 
        None if no waypoints are found. The check is done 15 meters from the traffic light. 
        If so, the one that is located at most 5 meters from the intersection is chosen as the target waypoint
        so it is possible to stop at an acceptable distance.
        Args:
            waypoints: list of the waypoints on the path
            ego_state: (x, y, yaw, current_speed) of the vehicle
            closest_index: index of the nearest waypoint
            goal_index: index of the current goal waypoint
        Returns:
            [int]: index of the waypoint target
        """
        
        # recover estimated TL location
        tl_pos = self.get_tl_pos()

        if tl_pos is not None:
            tl_pos = tl_pos[0]
            # For each waypoint from the closest to the goal we want to check if there is a traffic light that we have to manage
            
            for i in range(closest_index, goal_index):
                waypoint_loc_relative = transform_world_to_ego_frame(
                    [waypoints[i][0], waypoints[i][1], 0],
                    [ego_state[0], ego_state[1], 0.0],
                    [0.0, 0.0, ego_state[2]]
                )
                # We calculate the distance between the current waypoint and the current traffic light
                dist_spot = np.linalg.norm(np.array([waypoint_loc_relative[0] - tl_pos[0], waypoint_loc_relative[1] - tl_pos[1]]))
                # If this distance is smaller than DIST_TO_TL, we spot the traffic light
                if dist_spot < DIST_TO_TL:
                    # But we also check if it is ahead of ego or behind. If ahead, we choose a stop waypoint.
                    if tl_pos[0] > 0 and tl_pos[1] <= TL_Y_POS:
                        print(f"TL ahead. Position: {tl_pos}")
                        for j in range(i, len(waypoints)):
                            waypoint_loc_relative = transform_world_to_ego_frame(
                                [waypoints[j][0], waypoints[j][1], waypoints[j][2]],
                                [ego_state[0], ego_state[1], 0.0],
                                [0.0, 0.0, ego_state[2]]
                            )
                            if np.linalg.norm(np.array([waypoint_loc_relative[0] - tl_pos[0], waypoint_loc_relative[1] - tl_pos[1]])) < DIST_STOP_TL:
                                print(f"TL Stop Waypoint: {j} {waypoints[j]}")
                                return j
                    # Otherwise we stop checking.
                    else:
                        return None
        return None

    def get_tl_pos(self):
        """This function estimates the position of the nearest traffic light, 
           combining informations from detector e depth cameras.
        Returns:
            position in the vehicle frame or None if no boxes are detected
        """
        if self._current_box is None: return None # NO_TL

        cam_name = list(self._current_box.keys())[0] # name of the camera on which box is detected
        box = list(self._current_box.values())[0] # detected box
        # recover corresponding depth image infos
        depth_image = self._depth_images[cam_name] 
        inv_intrinsic_matrix = self._inv_intrinsic_matrices[cam_name]
        camera_params = self._cameras_params[cam_name]
        
        # recover box parameters
        xmin, ymin, xmax, ymax = box.get_bounds()
        xmin = xmin*400
        xmax = xmax*400
        ymin = ymin*400
        ymax = ymax*400

        # recover distance and pixel position of the TL
        depth = 1000 #Distance of the sky
        for i in range(int(xmin), int(xmax+1)):
            for j in range(int(ymin), int(ymax+1)):
                if j < 400 and i < 400:
                    if depth > depth_image[j][i]:
                        y = j
                        x = i
                        depth = depth_image[y][x]

        pixel = [x, y, 1]
        pixel = np.reshape(pixel, (3,1))
        
        depth = depth_image[y][x] * 1000  # Consider depth in meters  

        if depth!=1000:
            # Projection Pixel to Image Frame
            image_frame_vect = np.dot(inv_intrinsic_matrix,pixel) * depth

            # Create extended vector
            image_frame_vect_extended = np.zeros((4,1))
            image_frame_vect_extended[:3] = image_frame_vect 
            image_frame_vect_extended[-1] = 1

            # Projection Image to Camera Frame
            camera_frame = self._image_to_camera_frame(image_frame_vect_extended)
            camera_frame = camera_frame[:3]
            camera_frame = np.asarray(np.reshape(camera_frame, (1,3)))

            camera_frame_extended = np.zeros((4,1))
            camera_frame_extended[:3] = camera_frame.T 
            camera_frame_extended[-1] = 1
            
            # Projection Camera to Vehicle Frame
            camera_to_vehicle_frame = np.zeros((4,4))
            camera_to_vehicle_frame[:3,:3] = to_rot([camera_params["pitch"], camera_params["yaw"], camera_params["roll"]])
            camera_to_vehicle_frame[:,-1] = [camera_params["x"], camera_params["y"], camera_params["h"], 1]

            vehicle_frame = np.dot(camera_to_vehicle_frame, camera_frame_extended)
            vehicle_frame = vehicle_frame[:3]
            vehicle_frame = np.asarray(np.reshape(vehicle_frame, (1,3)))

            return vehicle_frame
        else: return None
    
    def get_new_goal(self, waypoints, ego_state):
        """this function computes the next goal waypoint, based on current state of the vehicle
        Args:
            waypoints: list of the waypoints on the path
            ego_state: (x, y, yaw, current_speed) of the vehicle
        Returns:
            [int]: index of the waypoint target
        """
        # First, find the closest index to the ego vehicle.
        closest_len, closest_index = get_closest_index(waypoints, ego_state)

        # Next, find the goal index that lies within the lookahead distance along the waypoints.
        goal_index = self.get_goal_index(waypoints, ego_state, closest_len, closest_index)
        while goal_index < (len(waypoints) - 1) and waypoints[goal_index][2] <= 0.1:
            goal_index += 1

        return closest_index,goal_index

    def update_goal(self, waypoints, goal_index, speed=None):
        """Updates the internal goal state given the waypoints and the goal index.
        Args:
            waypoints: list of the waypoints on the path
            goal_index: index of the waypoint target
            speed ([float], optional): speed to have in the gal waypoint. Defaults to None.
        """
        self._goal_index = goal_index
        self._goal_state = waypoints[goal_index]

        if speed is not None:
            self._goal_state[2] = speed

    def transition_state(self, waypoints, ego_state, closed_loop_speed):
        """Handles state transitions and computes the goal state.  
        
        args:
            waypoints: current waypoints to track (global frame). 
                length and speed in m and m/s.
                (includes speed to track at each x,y location.)
                format: [[x0, y0, v0],
                            [x1, y1, v1],
                            ...
                            [xn, yn, vn]]
                example:
                    waypoints[2][1]: 
                    returns the 3rd waypoint's y position
                    waypoints[5]:
                    returns [x5, y5, v5] (6th waypoint)
            ego_state: ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                    ego_x and ego_y     : position (m)
                    ego_yaw             : top-down orientation [-pi to pi]
                    ego_open_loop_speed : open loop speed (m/s)
            closed_loop_speed: current (closed-loop) speed for vehicle (m/s)
        variables to set:
            self._goal_index: Goal index for the vehicle to reach
                i.e. waypoints[self._goal_index] gives the goal waypoint
            self._goal_state: Goal state for the vehicle to reach (global frame)
                format: [x_goal, y_goal, v_goal]
            self._state: The current state of the vehicle.
                available states: 
                    FOLLOW_LANE         : Follow the global waypoints (lane).
                    DECELERATE_AND_STOP  : Decelerate to stop.
                    STOP_FOR_OBSTACLES  : Stay stopped until there's an obstacle on the lane.
        """
        # In this state, continue tracking the lane by finding the
        # goal index in the waypoint list that is within the lookahead
        # istance. Then, check to see if there's a pedestrian/vehicle that 
        # intersects our trajectory, in this case perform an emergency brake.
        # Otherwise check if there's a red traffic light; If it does, then ensure
        # that the goal state enforces the car to be stopped 5 m before the traffic light.
        if self._state == FOLLOW_LANE:
            #print("State: FOLLOW_LANE")

            #closest_index,goal_index = self.get_new_goal(waypoints, ego_state)
            closest_index,goal_index = self.get_new_goal(waypoints, ego_state)
            # if there's an agent that intersects our trajectory, stop immediately 
            if self._obstacle_on_lane:
                self.update_goal(waypoints, goal_index, 0)
                print("State: FOLLOW_LANE -> STOP_FOR_OBSTACLES")
                self._state = STOP_FOR_OBSTACLES

            else:
                intersection_goal = None
                # if there's a red traffic light, get the waypoint at 5 m from it
                if self._light_state == TrafficLightState.STOP:
                    intersection_goal = self.get_tl_stop_goal(waypoints, ego_state, closest_index, goal_index)
                
                if intersection_goal is not None:
                    self.update_goal(waypoints, intersection_goal, 0)
                    print("State: FOLLOW_LANE -> DECEL_AND_STOP")
                    self._state = DECELERATE_TO_STOP
                else:
                    self.update_goal(waypoints, goal_index)
                
        # In this state, check if there's an agent that intersects our trajectory for stopping immediately;
        # otherwise, check if the traffic light has becomed green or is disappeared from our vision for passing in follow lane.
        # (the second case is not real but we take care of it for avoiding to remain blocked at traffic light)
        elif self._state == DECELERATE_TO_STOP:
            #print("State: DECELERATE_AND_STOP")

            if self._obstacle_on_lane:
                print("State: DECEL_AND_STOP -> STOP_FOR_OBSTACLES")
                self._state = STOP_FOR_OBSTACLES

            elif self._light_state == TrafficLightState.GO or self._light_state == TrafficLightState.NO_TL:
                print("State: DECEL_AND_STOP -> FOLLOW_LANE")
                self._state = FOLLOW_LANE

        # in this state, check if agent has released our trajectory for passing in follow lane
        elif self._state == STOP_FOR_OBSTACLES:
            #print("State: STOP_FOR_OBSTACLES")
            if not self._obstacle_on_lane:
                print("State: STOP_FOR_OBSTACLES -> FOLLOW_LANE")
                self._state = FOLLOW_LANE

        else: raise ValueError('Invalid state value.')

    # Gets the goal index in the list of waypoints, based on the lookahead and
    # the current ego state. In particular, find the earliest waypoint that has accumulated
    # arc length (including closest_len) that is greater than or equal to self._lookahead.
    def get_goal_index(self, waypoints, ego_state, closest_len, closest_index):
        """Gets the goal index for the vehicle. 
        
        Set to be the earliest waypoint that has accumulated arc length
        accumulated arc length (including closest_len) that is greater than or
        equal to self._lookahead.

        args:
            waypoints: current waypoints to track. (global frame)
                length and speed in m and m/s.
                (includes speed to track at each x,y location.)
                format: [[x0, y0, v0],
                         [x1, y1, v1],
                         ...
                         [xn, yn, vn]]
                example:
                    waypoints[2][1]: 
                    returns the 3rd waypoint's y position

                    waypoints[5]:
                    returns [x5, y5, v5] (6th waypoint)
            ego_state: ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                    ego_x and ego_y     : position (m)
                    ego_yaw             : top-down orientation [-pi to pi]
                    ego_open_loop_speed : open loop speed (m/s)
            closest_len: length (m) to the closest waypoint from the vehicle.
            closest_index: index of the waypoint which is closest to the vehicle.
                i.e. waypoints[closest_index] gives the waypoint closest to the vehicle.
        returns:
            wp_index: Goal index for the vehicle to reach
                i.e. waypoints[wp_index] gives the goal waypoint
        """
        # Find the farthest point along the path that is within the
        # lookahead distance of the ego vehicle.
        # Take the distance from the ego vehicle to the closest waypoint into
        # consideration.
        arc_length = closest_len
        wp_index = closest_index

        # In this case, reaching the closest waypoint is already far enough for
        # the planner.  No need to check additional waypoints.
        if arc_length > self._lookahead:
            return wp_index

        # We are already at the end of the path.
        if wp_index == len(waypoints) - 1:
            return wp_index

        # Otherwise, find our next waypoint.
        while wp_index < len(waypoints) - 1:
            arc_length += np.sqrt((waypoints[wp_index][0] - waypoints[wp_index+1][0])**2 + (waypoints[wp_index][1] - waypoints[wp_index+1][1])**2)
            if arc_length > self._lookahead: break
            wp_index += 1

        return wp_index % len(waypoints)
    # Checks to see if we need to modify our velocity profile to accomodate the
    # lead vehicle.
    def check_for_lead_vehicle(self, ego_state, lead_car_position):
        """Checks for lead vehicle within the proximity of the ego car, such
        that the ego car should begin to follow the lead vehicle.

        args:
            ego_state: ego state vector for the vehicle. (global frame)
                format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                    ego_x and ego_y     : position (m)
                    ego_yaw             : top-down orientation [-pi to pi]
                    ego_open_loop_speed : open loop speed (m/s)
            lead_car_position: The [x, y] position of the lead vehicle.
                Lengths are in meters, and it is in the global frame.
        sets:
            self._follow_lead_vehicle: Boolean flag on whether the ego vehicle
                should follow (true) the lead car or not (false).
        """
        # Check lead car position delta vector relative to heading, as well as
        # distance, to determine if car should be followed.
        # Check to see if lead vehicle is within range, and is ahead of us.
        if not self._follow_lead_vehicle:
            # Compute the angle between the normalized vector between the lead vehicle
            # and ego vehicle position with the ego vehicle's heading vector.
            lead_car_delta_vector = [lead_car_position[0] - ego_state[0], 
                                     lead_car_position[1] - ego_state[1]]
            lead_car_distance = np.linalg.norm(lead_car_delta_vector)
            # In this case, the car is too far away.   
            if lead_car_distance > self._follow_lead_vehicle_lookahead:
                return

            lead_car_delta_vector = np.divide(lead_car_delta_vector, 
                                              lead_car_distance)
            ego_heading_vector = [math.cos(ego_state[2]), 
                                  math.sin(ego_state[2])]
            # Check to see if the relative angle between the lead vehicle and the ego
            # vehicle lies within +/- 45 degrees of the ego vehicle's heading.
            if np.dot(lead_car_delta_vector, 
                      ego_heading_vector) < (1 / math.sqrt(2)):
                return

            self._follow_lead_vehicle = True

        else:
            lead_car_delta_vector = [lead_car_position[0] - ego_state[0], 
                                     lead_car_position[1] - ego_state[1]]
            lead_car_distance = np.linalg.norm(lead_car_delta_vector)

            # Add a 15m buffer to prevent oscillations for the distance check.
            if lead_car_distance < self._follow_lead_vehicle_lookahead + 15:
                return
            # Check to see if the lead vehicle is still within the ego vehicle's
            # frame of view.
            lead_car_delta_vector = np.divide(lead_car_delta_vector, lead_car_distance)
            ego_heading_vector = [math.cos(ego_state[2]), math.sin(ego_state[2])]
            if np.dot(lead_car_delta_vector, ego_heading_vector) > (1 / math.sqrt(2)):
                return

            self._follow_lead_vehicle = False

    
# Compute the waypoint index that is closest to the ego vehicle, and return
# it as well as the distance from the ego vehicle to that waypoint.
def get_closest_index(waypoints, ego_state):
    """Gets closest index a given list of waypoints to the vehicle position.

    args:
        waypoints: current waypoints to track. (global frame)
            length and speed in m and m/s.
            (includes speed to track at each x,y location.)
            format: [[x0, y0, v0],
                     [x1, y1, v1],
                     ...
                     [xn, yn, vn]]
            example:
                waypoints[2][1]: 
                returns the 3rd waypoint's y position

                waypoints[5]:
                returns [x5, y5, v5] (6th waypoint)
        ego_state: ego state vector for the vehicle. (global frame)
            format: [ego_x, ego_y, ego_yaw, ego_open_loop_speed]
                ego_x and ego_y     : position (m)
                ego_yaw             : top-down orientation [-pi to pi]
                ego_open_loop_speed : open loop speed (m/s)

    returns:
        [closest_len, closest_index]:
            closest_len: length (m) to the closest waypoint from the vehicle.
            closest_index: index of the waypoint which is closest to the vehicle.
                i.e. waypoints[closest_index] gives the waypoint closest to the vehicle.
    """
    closest_len = float('Inf')
    closest_index = 0

    for i in range(len(waypoints)):
        temp = (waypoints[i][0] - ego_state[0])**2 + (waypoints[i][1] - ego_state[1])**2
        if temp < closest_len:
            closest_len = temp
            closest_index = i
    closest_len = np.sqrt(closest_len)

    return closest_len, closest_index

# Checks if p2 lies on segment p1-p3, if p1, p2, p3 are collinear.        
def pointOnSegment(p1, p2, p3):
    if (p2[0] <= max(p1[0], p3[0]) and (p2[0] >= min(p1[0], p3[0])) and \
       (p2[1] <= max(p1[1], p3[1])) and (p2[1] >= min(p1[1], p3[1]))):
        return True
    else:
        return False
