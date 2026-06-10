This directory contains the Schneider et al. EVRPTW benchmark set and
WCCI2020-style EVRP conversions for the current codebase.

Layout:
- `evrptw_instances/`: original EVRPTW `.txt` instances
- `*.evrp`: converted instances in WCCI2020-style layout

Conversion notes:
- Time-window and service-time fields are omitted in the converted `.evrp` files.
- `VEHICLES` is set to `ceil(total_demand / vehicle_capacity)` for each instance.
- `ENERGY_CAPACITY` is copied directly from the source value `Q`.
- `ENERGY_CONSUMPTION` is copied directly from the source value `r`.
- The current solver then derives the effective cruise-distance budget internally as `Q / r`.
- Recharging stations are preserved, including any station colocated with the depot.

Source:

@article{schneider2014electric,
  title={The electric vehicle-routing problem with time windows and recharging stations},
  author={Schneider, Michael and Stenger, Andreas and Goeke, Dominik},
  journal={Transportation science},
  volume={48},
  number={4},
  pages={500--520},
  year={2014},
  publisher={INFORMS}
}
