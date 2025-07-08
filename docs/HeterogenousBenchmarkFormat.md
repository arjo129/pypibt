# Heterogenous Benchmark Format

A heterogenous benchmark file consists of:
* A base map, the file is based on. These are taken from http://mapf.info. NOTE: Each cell is assumed to be 10*10 in size.
* An agent list with goals and agent footprints/dynamics.

```
agent_id fleet_id footprint_size velocity start_x start_y goal_x goal_y grid_width grid_height
```

Fleet IDs represent a homogenous fleet, footprint_size and velocity stay the same within a fleet.

Coordinates are given in absolute free-space.