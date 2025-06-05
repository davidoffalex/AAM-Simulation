import math
import random
import numpy as np
from typing import Optional
from aam_simulation.sim_utils import unit_vector, signed_angle_between
from aam_simulation.entities.vertiport import Vertiport
from aam_simulation.entities.corridor import Corridor, SplitMergePoint
from aam_simulation.entities.route import Route
import config

KNOTS_TO_FT_PER_SEC = (6076.0 / 3600.0)

class UAV:
    # UAV States
    STATE_TAXI = "TAXI"
    STATE_CLIMB = "CLIMB"
    STATE_CRUISE = "CRUISE"
    STATE_DESCENT = "DESCENT"
    STATE_EVASIVE = "EVASIVE"
    STATE_HOLD = "HOLD"
    STATE_CHARGING = "CHARGING"

    def __init__(self,
                 uav_id: int,
                 route: Route,
                 vertiport_dict: dict,
                 corridor_list: list,
                 split_merge_points: list,
                 ref_lat: float):
        self.id = uav_id
        self.route = route
        self.vertiport_dict = vertiport_dict
        self.corridors = corridor_list
        self.smp = split_merge_points
        self.ref_lat = ref_lat

        origin_vert = route.waypoints[0] #type: Vertiport
        self.position = np.array([origin_vert.position[0], origin_vert.position[1], 0.0])
        self.speed = 0.0 # knots
        self.heading = np.array([0.0, 0.0, 0.0])
        self.sphere_radius = config.UAV_RADIUS_FT

        self.state = UAV.STATE_TAXI
        self.time_in_current_state = 0
        self.current_leg_index = 0
        self.altitude = 0.0
        self.destination_vertiport = route.waypoints[-1]
        self.flight_plan = route.waypoints.copy()
        self.eta = None
        self.trip_duration = 0
        self.time_to_charge = 0
        self.has_deviated = False

        # Evasive/conflict tracking
        self.in_conflict = False
        self.evasion_start_time = None
        self.evasion_type = None
        self.original_speed = None
        self.original_altitude = None
        self.evasion_phase = None

    def _find_current_corridor(self):
        """
        Return (Corridor, origin_vert, dest_vert, altitude, heading_unit) for the current leg.
        If none, return None.
        """
        if self.current_leg_index >= len(self.route.legs):
            return None
        origin, dest = self.route.legs[self.current_leg_index]
        for corridor in self.corridors:
            seg = corridor.get_segment_info(origin, dest)
            if seg:
                altitude, heading_unit = seg
                return corridor, origin, dest, altitude, heading_unit
        return None
    
    def _update_state(self, time_step: int):
        """
        Called once per second by Simulation. Updates position, state transitions,
        conflict recovery, charging countdown, and other UAV state attributes.
        """

        # If airborne (in any flight-phase), update trip_duration
        if self.state in {
            UAV.STATE_CLIMB,
            UAV.STATE_CRUISE,
            UAV.STATE_DESCENT,
            UAV.STATE_EVASIVE,
            UAV.STATE_HOLD
        }:
            self.trip_duration += 1
        
        # 1. CHARGING State
        if self.state == UAV.STATE_CHARGING:
            self.time_to_charge -= 1
            if self.time_to_charge <= 0:
                self.state = UAV.STATE_TAXI
                self.time_to_charge = 0
            return
        
        # 2. TAXI State
        if self.state == UAV.STATE_TAXI:
            return # waiting for takeoff clearance
        
        # 3. EVASIVE or HOLD
        if self.state == UAV.STATE_EVASIVE:
            self._handle_evasive(time_step)
            return
        
        if self.state == UAV.STATE_HOLD: 
            # Remain in HOLD until Simulation clears conflict
            return
        
        # 4. CLIMB (Diagonal climb + accelerate)
        if self.state == UAV.STATE_CLIMB:
            # First, fetch the corridor we are climbing into:
            corridor_info = self._find_current_corridor()
            if corridor_info is None:
                # No more legs, we must already be at destination (should not happen here)
                self.state = UAV.STATE_DESCENT
                return
            
            _, origin, dest, cruise_altitude, heading_unit = corridor_info

            # a) Vertical climb: 900 ft/min = 15 ft/s
            climb_rate_fps = config.CLIMB_RATE_FPS
            self.altitude += climb_rate_fps
            if self.altitude > cruise_altitude:
                self.altitude = cruise_altitude
            
            # b) Lateral acceleration: linearly from 0 to 115 knots over the same vertical distance
            # Time to climb from 0 to cruise altitude at 15 ft/s: t_climb_sec = cruise_altitude / 15
            if cruise_altitude > 0:
                t_climb_sec = cruise_altitude / climb_rate_fps
                accel_rate_kt_per_sec = config.CRUISE_SPEED_KT / t_climb_sec
            else: 
                accel_rate_kt_per_sec = 0.0

            self.speed += accel_rate_kt_per_sec
            if self.speed > config.CRUISE_SPEED_KT:
                self.speed = config.CRUISE_SPEED_KT
            
            # c) Horizontal movement: move forward by current speed (in ft/s)
            speed_fps = self.speed * KNOTS_TO_FT_PER_SEC # convert knots to ft/s
            movement = np.array([heading_unit[0], heading_unit[1], 0.0]) * speed_fps
            self.position += movement

            # d) Once both altitude and speed have reached cruise targets, transition to CRUISE
            if self.altitude >= cruise_altitude and self.speed >= config.CRUISE_SPEED_KT:
                self.altitude = cruise_altitude
                self.speed = config.CRUISE_SPEED_KT
                self.state = UAV.STATE_CRUISE

            return
        
        # 5. CRUISE (constant altitude & speed, check for descent trigger)
        if self.state == UAV.STATE_CRUISE:
            corridor_info = self._find_current_corridor()

            # If we have flown all legs, begin descent immediately
            if corridor_info is None:
                self.state = UAV.STATE_DESCENT
                return
            
            _, origin, dest, cruise_altitude, heading_unit = corridor_info

            # Compute how far (2D) we are currently from the next waypoint (dest)
            dest_xy = np.array([dest.position[0], dest.position[1]])
            cur_xy = np.array([self.position[0], self.position[1]])
            horiz_dist_to_dest_ft = np.linalg.norm(dest_xy - cur_xy)

            # Compute the required horizontal distance to descent diagonally from cruise to 0
            if cruise_altitude > 0:
                descent_rate_fps = config.DESCENT_RATE_FPS
                t_descent_sec = cruise_altitude / descent_rate_fps
                initial_speed_ftps = config.CRUISE_SPEED_KT * KNOTS_TO_FT_PER_SEC
                avg_speed_ftps = initial_speed_ftps / 2.0 # (cruise speed + 0) / 2, average of the initial and final speed
                required_horiz_dist_for_descent = avg_speed_ftps * t_descent_sec
            else:
                required_horiz_dist_for_descent = 0.0
            
            # If we are within that descent distance and dest is a vertiport, begin diagonal descent
            if isinstance(dest, Vertiport) and horiz_dist_to_dest_ft <= required_horiz_dist_for_descent:
                self.state = UAV.STATE_DESCENT
                return
            
            # Otherwise, remain at cruise altitude & speed, moving forward by the cruise speed each sec
            speed_fps = config.CRUISE_SPEED_KT * KNOTS_TO_FT_PER_SEC
            movement = np.array([heading_unit[0], heading_unit[1], 0.0]) * speed_fps
            self.position += movement
            self.altitude = cruise_altitude
            self.speed = config.CRUISE_SPEED_KT

            # Check if arrived at or passed destination (for split/merge point)
            if horiz_dist_to_dest_ft <= speed_fps:
                # Snap to exactly destination
                self.position[0], self.position[1] = dest_xy[0], dest_xy[1]
                if not isinstance(dest, Vertiport): # passed a split/merge point
                    self.current_leg_index += 1
                # Else: we are at a vertiport boundary. If this wasn't caught just before, next tick handles it.