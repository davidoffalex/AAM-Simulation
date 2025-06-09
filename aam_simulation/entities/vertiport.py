from collections import deque
from aam_simulation.sim_utils import latlon_to_cartesian
from aam_simulation import config

class ChargeStation: 
    def __init__(self):
        self.available = True
        self.current_uav = None
        self.time_remaining = 0 # seconds
    
    def assign_uav(self, uav_id: int, charge_time_sec: int):
        """Assigns UAV to the charge station and marks charge station as unavailable."""
        self.available = False
        self.current_uav = uav_id
        self.time_remaining = charge_time_sec
    
    def tick(self):
        """Defines charge station behavior in each simulation tick."""
        if not self.available:
            self.time_remaining -= 1
            if self.time_remaining <= 0:
                self.available = True
                self.current_uav = None
                self.time_remaining = 0

class Vertiport:
    def __init__(self, name: str, lat: float, lon: float, ref_lat: float, num_charge_stations: int): 
        self.name = name
        self.lat = lat
        self.lon = lon
        self.position = latlon_to_cartesian(lat, lon, ref_lat) # (x, y) in feet

        self.pending_takeoff_ids = deque()
        self.takeoff_queue = deque() # UAV IDs waiting to take off
        self.incoming_landing_times = [] # List of (time, uav_id) tuples

        self.charge_stations = [ChargeStation() for _ in range(num_charge_stations)]
        self.last_takeoff_time = -float('inf')
        self.last_landing_time = -float('inf')

    def request_takeoff(self, uav_id: int) -> bool:
        """Store UAV as pending. Actual queueing is handled during tick()."""
        self.pending_takeoff_ids.append(uav_id)
    
    def assign_charge_station(self, uav_id: int) -> bool:
        """Assigns UAV to an available charge station."""
        for station in self.charge_stations:
            if station.available:
                station.assign_uav(uav_id, config.CHARGE_TIME_SEC)
                return True
        return False
    
    def tick(self, current_time: int):
        """Defines vertiport behavior in each simulation tick."""
        for station in self.charge_stations:
            station.tick()
        
        # Try to promote pending UAVs into takeoff queue
        for _ in range(len(self.pending_takeoff_ids)):
            uav_id = self.pending_takeoff_ids.popleft()
            if (self._can_takeoff(current_time)):
                self.takeoff_queue.append(uav_id)
                self.last_takeoff_time = -float('inf')  # Let _process_takeoffs() enforce time gap
            else:
                self.pending_takeoff_ids.append(uav_id)

    def _can_takeoff(self, current_time: int) -> bool:
        """Returns True if no takeoff or landing occurred in the last 60 seconds."""
        return ((current_time - self.last_takeoff_time) >= 60 and
                (current_time - self.last_landing_time) >= 60)