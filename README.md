# AAM-Simulation

This repository presents an AAM simulation to use for traffic studies and experiments on air traffic management frameworks. 

To run the simulation, ensure that numpy is installed in your environment and run aam_simulation as a python module. All simulation parameters may be changed in config.py. 

The simulation provides four default entities: corridor, route, uav, and vertiport (charge station logic is included within vertiport.py). The airspace coordinates for vertiports and split/merge points must be manually set in __main__.py. Examples are shown in the functions build_standard_airspace() and build_simple_airspace(). 

Outside of aam_simulation, experiment1.py, experiment2.py, and experiment3.py present sample experiments for traffic studies on the simulated AAM airspaces. The airspace type, "simple" or "standard," is set in config.py.