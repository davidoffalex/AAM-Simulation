import math
import numpy as np
from typing import Dict
from aam_simulation.entities.vertiport import Vertiport
from aam_simulation.entities.uav import UAV
from aam_simulation.airspace import Airspace
from aam_simulation import config

class Simulation:
    def __init__(self,
                 airspace: Airspace,
                 num_uavs_per_route: Dict[int, int],
                 min_lat_sep: float,
                 min_vert_sep: float,
                 enable_delays: bool = False):
        self.airspace = airspace
        self.min_lat_sep = min_lat_sep
        self.min_vert_sep = min_vert_sep
        self.enable_delays = enable_delays

        self.uavs: Dict[int, UAV] = {}
        self.active_uav_ids = set()
        self.next_uav_id = 1

        # Instantiate UAVs on each route, enqueue for takeoff at t = 0
        for route_id, count in num_uavs_per_route.items():
            route = airspace.routes[route_id]
            for _ in range(count):
                uav = UAV(self.next_uav_id,
                          route,
                          airspace.corridors,
                          airspace.split_merge_points,
                          airspace.ref_lat)
                self.uavs[self.next_uav_id] = uav
                origin_name = route.waypoints[0].name
                airspace.vertiports[origin_name].request_takeoff(self.next_uav_id)
                self.active_uav_ids.add(self.next_uav_id)
                self.next_uav_id += 1
            
        print(f"UAV {uav.id} route legs:") #DEBUG
        for leg in uav.route.legs: #DEBUG
            print(f"  {leg[0].name} → {leg[1].name}") #DEBUG

        # Statistics
        self.time = 0
        self.conflict_count = 0
        self.total_conflicts = 0
        self.collision_count = 0
        self.total_collisions = 0
        self.throughput_count = 0
        self.ter_list = []

        # Delay tracking
        self.delay_end_time = None

    def run(self, total_time_sec: int):
        for t in range(total_time_sec):
            self.time = t
            self._process_takeoffs()
            self._update_uavs()
            for uav_id in list(self.active_uav_ids):
                self.uavs[uav_id].update_eta()
            if self.enable_delays and t > 0 and t % (5*60) == 0: # Delays refresh every 5 minutes
                self._evaluate_and_apply_delays()
            for v in self.airspace.vertiports.values():
                v.tick(current_time=self.time)
    
    def _process_takeoffs(self):
        for v in self.airspace.vertiports.values():
            # Takeoff
            if v.takeoff_queue:
                uav_id = v.takeoff_queue[0]

                # Condition 1: Another UAV is landing within 60 seconds
                imminent_landing = any(
                    uav.destination_vertiport == v and uav.eta is not None and 0.0 < uav.eta < 1.0
                    for uav in self.uavs.values()
                    if uav.state in {UAV.STATE_CLIMB, UAV.STATE_CRUISE, UAV.STATE_DESCENT}
                )
                # Condition 2: Recent landing
                recent_landing = (self.time - v.last_landing_time) < 60

                # Condition 3: Recent takeoff
                recent_takeoff = (self.time - v.last_takeoff_time) < 60

                # Condition 4: Global conflict-based delay
                delay_active = self.delay_end_time is not None and self.time < self.delay_end_time

                if not (imminent_landing or recent_landing or recent_takeoff or delay_active):
                    uav = self.uavs[uav_id]
                    uav.initiate_takeoff()
                    v.takeoff_queue.popleft()
                    v.last_takeoff_time = self.time
    
    def _update_uavs(self):
        airborne_ids = []
        for uav_id in list(self.active_uav_ids):
            uav = self.uavs[uav_id]
            prev_state = uav.state
            uav._update_state(self.time)
            if uav.state == UAV.STATE_DESCENT: # DEBUG
                print(f"[{uav.id}] entered DESCENT from {prev_state}") # DEBUG

            if prev_state == UAV.STATE_DESCENT and uav.state == UAV.STATE_CHARGING:
                # Landed and moved to charging
                pass

            if prev_state == UAV.STATE_CHARGING and uav.state == UAV.STATE_TAXI:
                # Completed one full trip
                self.throughput_count += 1
                print(f"trip duration: {uav.trip_duration}") # DEBUG
                actual_min = uav.trip_duration / 60.0
                start_pos = uav.flight_plan[0].position
                dest_pos = uav.destination_vertiport.position
                horiz_ft = np.linalg.norm(dest_pos - start_pos)
                ideal_nm = horiz_ft / config.FT_IN_NM
                ideal_min = ((ideal_nm / config.CRUISE_SPEED_KT) + config.CHARGE_TIME_SEC) * 60.0
                ter = actual_min / ideal_min if ideal_min > 0 else 1.0
                self.ter_list.append(ter)

                #Re-enqueue this UAV for takeoff at its current vertiport
                uav.route.waypoints.reverse()
                uav.route.legs = [
                    (uav.route.waypoints[i], uav.route.waypoints[i+1])
                    for i in range(len(uav.route.waypoints) - 1)
                ]
                uav.flight_plan = uav.route.waypoints.copy()
                uav.current_leg_index = 0
                uav.destination_vertiport = uav.route.waypoints[-1]
                uav.has_deviated = False
                current_vertiport_name = uav.route.waypoints[0].name
                self.airspace.vertiports[current_vertiport_name].request_takeoff(uav_id)

                # Reset its trip duration counter
                uav.trip_duration = 0

            if uav.state in {UAV.STATE_CLIMB, UAV.STATE_CRUISE, UAV.STATE_DESCENT, UAV.STATE_EVASIVE, UAV.STATE_HOLD}:
                airborne_ids.append(uav_id)
        
        # Pairwise conflict detection
        for i in range(len(airborne_ids)):
            u1 = self.uavs[airborne_ids[i]]
            for j in range(i+1, len(airborne_ids)):
                u2 = self.uavs[airborne_ids[j]]
                if u1.check_for_conflicts(u2, self.min_lat_sep, self.min_vert_sep):
                    horiz_d = math.hypot(u1.position[0] - u2.position[0],
                                         u1.position[1] - u2.position[1])
                    vert_d = abs(u1.altitude - u2.altitude)
                    if horiz_d <= (u1.sphere_radius + u2.sphere_radius) and vert_d <= 0.0:
                        self.collision_count += 1
                        self.total_collisions += 1
                        if self.enable_delays:
                            self.delay_end_time = self.time + config.MIN_DELAY_TIME_S
                    else:
                        self.conflict_count += 1
                        self.total_conflicts += 1
                        # If the UAV is not already initiating an evasive manuever, initiate one
                        if not u1.in_conflict:
                            u1.initiate_evasive_action(u2, self.time, self.min_vert_sep)
        
        # After conflict handling, allow mid-flight deviations
        for uav_id in airborne_ids:
            self.uavs[uav_id].maybe_deviate()

    def _evaluate_and_apply_delays(self):
        rate_per_min = self.conflict_count / 5.0
        if rate_per_min >= 1.0 or self.collision_count > 0:
            self.delay_end_time = self.time + config.MIN_DELAY_TIME_S
        self.conflict_count = 0
        self.collision_count = 0
    
    def get_statistics(self):
        avg_ter = sum(self.ter_list) / len(self.ter_list) if self.ter_list else None
        throughput = self.throughput_count / (config.RUN_TIME_SEC / 60)
        return {
            "num_conflicts": self.total_conflicts,
            "num_collisions": self.total_collisions,
            "throughput": throughput,
            "average_TER": avg_ter
        }