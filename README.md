# DD-NMPC replay viewer

Static viewer for `https://ferhannb.github.io/dd-nmpc-playground/`. It replays frozen closed-loop simulation results of a compact Panamax manoeuvring model (not validated at ship scale) under a distance-domain NMPC, for three routes and three N/M settings, with zero current and no measurement noise.

The browser only draws the recorded data in `data/`. The controller, the solver and the vessel model are not included.
