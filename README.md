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

For more info read [this paper](https://arxiv.org/abs/2506.16748).

If you use this work in an academic setting consider citing:
```
@misc{chakravarty2025scalablepostprocessingpipelinelargescale,
      title={A Scalable Post-Processing Pipeline for Large-Scale Free-Space Multi-Agent Path Planning with PiBT}, 
      author={Arjo Chakravarty and Michael X. Grey and M. A. Viraj J. Muthugala and Mohan Rajesh Elara},
      year={2025},
      eprint={2506.16748},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2506.16748}, 
}
```
