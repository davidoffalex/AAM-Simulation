from aam_simulation.airspace import Airspace
from aam_simulation.simulation import Simulation
from aam_simulation import config

def main():

    # 1. Build the airspace
    if config.AIRSPACE == "Standard":
        airspace = build_standard_airspace()
        num_uavs = {i: config.UAVS_PER_ROUTE for i in range (1, 7)}
    else:
        airspace = build_simple_airspace()
        num_uavs = {i: config.UAVS_PER_ROUTE for i in range (1, 3)}

    # 3. Create simulation
    sim = Simulation(airspace, num_uavs, min_lat_sep=config.DEFAULT_MIN_LAT_SEP,
                     min_vert_sep=config.DEFAULT_MIN_VERT_SEP, enable_delays=False)
    
    # 4. Run for alloted time in config
    sim.run(total_time_sec=config.RUN_TIME_SEC)

    #5. Print results
    stats = sim.get_statistics()
    print("Simulation Complete")
    print(f"Throughput: {stats['throughput']}")
    print(f"Average TER: {stats['average_TER']:.3f}" if stats["average_TER"] else "No TER data")
    print(f"Total Conflicts Detected: {stats['num_conflicts']}")
    print(f"Total Collisions Detected: {stats['num_collisions']}")

def build_standard_airspace() -> Airspace:
    ref_lat = 28.468 # average of all latitudes of vertiports/split points
    airspace = Airspace(ref_lat)
    airspace.add_vertiport("A", lat=28.32, lon=-82.20, num_charge_stations=config.CHARGE_STATIONS)
    airspace.add_vertiport("B", lat=28.86, lon=-81.64, num_charge_stations=config.CHARGE_STATIONS)
    airspace.add_vertiport("C", lat=28.63, lon=-81.36, num_charge_stations=config.CHARGE_STATIONS)
    airspace.add_vertiport("D", lat=28.41, lon=-80.70, num_charge_stations=config.CHARGE_STATIONS)
    airspace.add_vertiport("E", lat=28.12, lon=-81.65, num_charge_stations=config.CHARGE_STATIONS)

    airspace.add_corridor("A", "B", alt_ab=3000.0, alt_ba=3200.0)
    airspace.add_corridor("A", "C", alt_ab=3000.0, alt_ba=3200.0)
    airspace.add_corridor("B", "C", alt_ab=3000.0, alt_ba=3200.0)
    airspace.add_corridor("B", "E", alt_ab=3500.0, alt_ba=3700.0)
    airspace.add_corridor("C", "D", alt_ab=3000.0, alt_ba=3200.0)
    airspace.add_corridor("C", "E", alt_ab=3000.0, alt_ba=3200.0)

    airspace.add_split_merge_point("A-C", lat=28.63, lon=-81.86, vert_a_name="A", vert_b_name="C")

    airspace.define_route(1, ["A", "B"], alternate_vert_name="C")
    airspace.define_route(2, ["A", "C"], alternate_vert_name="B")
    airspace.define_route(3, ["B", "C"], alternate_vert_name="E")
    airspace.define_route(4, ["B", "E"], alternate_vert_name="C")
    airspace.define_route(5, ["C", "D"], alternate_vert_name="E")
    airspace.define_route(6, ["C", "E"], alternate_vert_name="B")

    return airspace

def build_simple_airspace() -> Airspace:
    ref_lat = 28.99 # average of all latitudes of vertiports/split points
    airspace = Airspace(ref_lat)
    airspace.add_vertiport("A", lat=28.77, lon=-81.57, num_charge_stations=config.CHARGE_STATIONS)
    airspace.add_vertiport("B", lat=29.02, lon=-81.33, num_charge_stations=config.CHARGE_STATIONS)
    airspace.add_vertiport("C", lat=29.17, lon=-81.05, num_charge_stations=config.CHARGE_STATIONS)

    airspace.add_corridor("A", "B", alt_ab=3000.0, alt_ba=3200.0)
    airspace.add_corridor("B", "C", alt_ab=3000.0, alt_ba=3200.0)

    airspace.define_route(1, ["A", "B"], alternate_vert_name="C")
    airspace.define_route(2, ["B", "C"], alternate_vert_name="A")

    return airspace

if __name__ == "__main__":
    main()