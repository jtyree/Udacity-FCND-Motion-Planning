import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        if len(self.waypoints) > 0:
            print("waypoint transition")
            self.target_position = self.waypoints.pop(0)
            print('target position', self.target_position)
            self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])
        else:
            print('no waypoints')

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # DONE: read lat0, lon0 from colliders into floating point values
        la, lo = lat_lon_from_csv('colliders.csv')

        # DONE: set home position to (lat0, lon0, 0)
        self.set_home_position(lo, la, 0)

        # DONE: retrieve current global position
        print('Global position: {0}'.format(self.global_position))

        # DONE: convert to current local position using global_to_local()
        print('Local position: {0}'.format(global_to_local(self.global_position, self.global_home)))
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)

        # DONE: convert start position to current position rather than map center
        actual_start = (grid_start[0] + int(self.local_position[0]), grid_start[1] + int(self.local_position[1]))
        print('Actual starting position: {0}'.format(actual_start))

        # Set goal based on command line arguments
        local_goal = global_to_local((args.long, args.lat, 0), self.global_home)
        grid_goal = (-north_offset + int(local_goal[0]), -east_offset + int(local_goal[1]))
        print('Grid goal: {0}'.format(grid_goal))
        # DONE: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', actual_start, grid_goal)
        
        if actual_start == grid_goal:
            print('start and goal are the same - no path to plan')
            waypoints = []
        else:
            path, _ = a_star(grid, heuristic, actual_start, grid_goal)

            # DONE: prune path to minimize number of waypoints
            # TODO (if you're feeling ambitious): Try a different approach altogether!
            pruned = prune_path(path)

            # Convert path to waypoints
            waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned]

        # Set self.waypoints
        self.waypoints = waypoints
        # DONE: send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


def point(p):
    return np.array([p[0], p[1], 1.])


def collinearity_check(p1, p2, p3, epsilon=1e-2):
    mat = np.vstack((point(p1), point(p2), point(p3)))
    if np.abs(np.linalg.det(mat)) < epsilon:
        return True
    else:
        return False


def prune_path(path):
    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])

        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path


# Read the latitude and longitude from the first line of the colliders file and return it
def lat_lon_from_csv(csvfile):
    with open(csvfile) as f:
        line = f.readline().strip('\n')

    line_list = line.split(',')

    lat_list = line_list[0].strip().split(' ')
    long_list = line_list[1].strip().split(' ')

    lat_lon = (float(lat_list[1]), float(long_list[1]))
    return lat_lon


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--lat', type=float, default=37.79248, help="latitude")
    parser.add_argument('--long', type=float, default=-122.39745, help="longitude")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
