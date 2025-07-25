# HetPiBT

This is an extension to PiBT. This extension enables us to plan around heterogenous systems.
The idea is that different robots may move at different paces and have different sizes, we use a simple
precomputed collision cache to reason over the different robots.

## Benchmarking Methodology

After some survey, we determined we don't yet have a good set of benchmarks for heterogenous fleets. We reuse the MAPF
benchmarks for maps however we generate synthetic scenarios. Each scenario grid map gets projected onto each robot's nav-graph.
Subsequently in order to generate valid agent paths we randomly select start positions for agents and then add end goals for agents.
To ensure that the scenarios can actually be solved, we rely on doing a BFS for each domain and sampling from it. Furthermore, we perform basic collision checking to ensure start and end goals don't overlap. For more information about benchmarking look over
[here](HeterogenousBenchmarkFormat.md).

## Some cool demos
Here we have a bunch of robots with different sizes and dynamics. The blue robots are bigger and move faster while the red robots are smaller and move more slowly, we show that HetPiBT correctly routes the individual vehicles in a safe manner.
![](resources/het_pibt.gif)

## Observations and Conclusions

While HetPiBT is fast, it can generate really suboptimal paths and introduce additional movements where unnecessary.
Using PiBT alone for heterogenous search is not a bad idea, but it needs a higher level of optimization to prevent unwanted oscillations.

Additionally it is very much possible to deadlock HetPiBT in denser settings. I have some ideas for how to fix this but this instance is an example where HetPiBT completely fails:

![deadlock](resources/deadlock.gif)

There are several remedies to this. The first is to adapt WinPiBT's dynamic window. The second is to adapt the reservation and priority inheritance system to include, minimum kinodynamic move required. Finally we could do backtracking in the event of a loop and perform deadlock detection and resolution at a higher level.