# Heterogenous Benchmark Format

A heterogenous benchmark file consists of:
* A base map, the file is based on. These are taken from http://mapf.info. NOTE: Each cell is assumed to be 10*10 in size.
* An agent list with goals and agent footprints/dynamics.

```
agent_id fleet_id footprint_size velocity start_x start_y goal_x goal_y grid_width grid_height
```

Fleet IDs represent a homogenous fleet, footprint_size and velocity stay the same within a fleet.

Coordinates are given in absolute free-space.

# Generating benchmarks

You can generate benchmarks for a benchmark for a mapf scenario of this kind by using the `tools/generate_het_benchmark_file.py`
file.
```bash
poetry run python3 tools/generate_het_banchmark_file.py --map_file assets/room_64_64_8.map --scenarios 10 --outfile het_bench/scen --num_fleets 3 --num_agents 3 --b_factor 5 --width 12 --height 12
```

To run HetPiBT
```bash
poetry run python3 vis_het_bench.py --map_file assets/room-64-64-8.map --scene_file het_bench/scen.1.scen
```

For using the open-RMF mapf library to compare against baseline CBS based methods one can use `tools/het_mapf_tools.py` script.

```bash
poetry run python tools/het_mapf_tools.py --map_file assets/room-64-64-8.map --scene_file het_bench/scen.1.scen --out_file tmp/my_out.yaml  
```
