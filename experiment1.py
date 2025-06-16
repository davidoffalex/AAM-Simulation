import csv
from aam_simulation import config
from aam_simulation.__main__ import build_standard_airspace, build_simple_airspace
from aam_simulation.simulation import Simulation

def run_experiment():
    increment = 3
    n_configs = 15
    runs_per_config = 10
    runtime_sec = config.RUN_TIME_SEC
    min_lat_sep = config.DEFAULT_MIN_LAT_SEP
    min_vert_sep = config.DEFAULT_MIN_VERT_SEP

    enable_delays = False

    results = []

    for idx in range(n_configs):
        per_route = config.UAVS_PER_ROUTE + (idx * increment)

        conflict_counts = []
        collision_counts = []

        for run in range(runs_per_config):
            # 1. Build a fresh airspace
            # 1. Build the airspace
            if config.AIRSPACE == "Standard":
                airspace = build_standard_airspace()
                num_uavs = {i: per_route for i in range (1, 7)}
            else:
                airspace = build_simple_airspace()
                num_uavs = {i: per_route for i in range (1, 3)}

            # 3. Create and run simulation
            sim = Simulation(airspace,
                             num_uavs_per_route=num_uavs,
                             min_lat_sep=min_lat_sep,
                             min_vert_sep=min_vert_sep,
                             enable_delays=enable_delays)
            sim.run(total_time_sec=runtime_sec)

            stats = sim.get_statistics()
            conflict_counts.append(stats["num_conflicts"])
            collision_counts.append(stats["num_collisions"])

        # average over runs
        avg_conflict_rate = sum(conflict_counts) / runs_per_config
        avg_collision_rate = sum(collision_counts) / runs_per_config

        results.append((per_route, avg_conflict_rate, avg_collision_rate))

        print(f"Done config {idx+1}/{n_configs}: "
              f"UAVs/route={per_route} → "
              f"conf_rate={avg_conflict_rate:.3f}/min, "
              f"coll_rate={avg_collision_rate:.3f}/min")

    # write out CSV
    with open("experiment1_results.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["UAVs_per_route", "avg_conflicts_per_min", "avg_collisions_per_min"])
        writer.writerows(results)

    print("Experiment complete. Results saved to experiment1_results.csv")

if __name__ == "__main__":
    run_experiment()
