from typing import List, Union
from aam_simulation.entities.vertiport import Vertiport
from aam_simulation.entities.corridor import SplitMergePoint

class Route:
    def __init__(self, route_id: int, 
                 waypoints: List[Union[Vertiport, SplitMergePoint]], 
                 alternate_vertiport: Vertiport):
        """
        route_id: integer identifier
        waypoints: ordered list of Vertiport or SplitMergePoint objects
        alternate_vertiport: if UAV deviates, it will fly here instead
        """
        self.route_id = route_id
        self.waypoints = waypoints
        self.alternate_vertiport = alternate_vertiport

        # Build legs: pairs of consecutive waypoints, legs is a list of (origin, destination) pairs
        self.legs = []
        for i in range(len(waypoints) - 1):
            self.legs.append((waypoints[i], waypoints[i + 1]))