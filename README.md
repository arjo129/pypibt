# pypibt

This repo consists of two extensions to PiBT. The
first is a path smoother which is able to take pibt
and extend it to free space, while the second is [a
version of PiBT that works on heterogenous fleets](docs/HeterogenousPiBT.md).
It is based on Keisuke Okumura's original python pypibt implementation.

# Development
To start developing this workspace use poetry:
```
poetry install
```

# Free-Space Path Smoothing for PiBT

![room-based-free-space-planning](docs/resources/freespace_planning.gif)
![berlin](docs/resources/berlin.gif)

This set of experiments explores a post-hoc path smoothing solution for MAPF
in free space. To run a demo first run PiBT the following command:

```
poetry run python3 tools/benchmark_path_smoothing.py --map assets/Berlin_1_256.map --scenario assets/Berlin_1_256-random-10.scen --agents 100
```

We were able to successfully test all instances of the mapf benchmark except the maze instances. You may find the details of the results [here](docs/results/results.pibt.csv). We compare against AA-CCBS+SIPP [here](docs/results/results.aaccbs.csv).

If you use this work in an academic setting consider citing:
```

```
