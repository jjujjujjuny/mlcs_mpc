# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

F1TENTH autonomous-racing stack for a real car (Jetson Orin Nano, Ubuntu 22.04, ROS 2 Humble). The controller is an MPCC (Frenet contouring MPC, acados). Its formulation started from the [HyperMPC](https://arxiv.org/abs/2508.06181) paper, but the MPC itself does not depend on HyperMPC. Learned models (e.g. HyperPM) plug in separately through `learned/`. Localization comes from OptiTrack motion capture (lab) or from LiDAR + Cartographer (competition). Docs, comments, commit messages and CLI output are in **Korean**, so match that.

The workflow is: tune on the PC in the ODE simulator (`./sim.sh`), commit, then run on the Jetson (`./race.sh`). Both read the same `race.yaml` and `src/mlcs_mpc/config/mpcc.yaml` (written `config/mpcc.yaml` below).

There are three machines:
- **Jetson** (the car): the workspace is `~/f1tenth_ws`, and `src/mlcs_mpc` is **symlinked** into it and built with `colcon build --symlink-install`. Changes to `setup.py` entry points or packages need a rebuild. casadi/acados exist only in `~/hmpc-venv` (the OCP worker runs there).
- **Main PC**: Ubuntu 20.04 with ROS 1 Noetic in `.bashrc`, plus ROS 2 Foxy in `/opt/ros/foxy`. `sim.sh` strips the ROS 1 environment and builds `~/mlcs_sim_ws` (mlcs_mpc + `third_party/vesc_msgs`). System python3.8 has casadi, acados_template and `~/acados`.

- **MacBook** (Apple Silicon): RoboStack conda env `mlcs_ros` (ROS 2 Humble, casadi, `acados_template`, torch) + `~/acados` v0.6.1. Always use that env's python, never `/usr/bin/python3`, because SIP strips `DYLD_LIBRARY_PATH`. `sim.sh` activates the env itself when there is no `/opt/ros`. See `docs/SETUP_MAC.md`.

numpy is pinned to 1.26.x on Ubuntu/Jetson (2.x breaks cv2/matplotlib/colcon). The Mac conda env needs numpy 2.x (RoboStack is built against it) and `setuptools<80` (for colcon `--symlink-install`).

Platform-specific code goes through `utils/runenv.LIB_PATH_VAR` (`DYLD_`/`LD_LIBRARY_PATH`) and `env/mpc/acados_solver.LIB_EXT` (`.dylib`/`.so`). Don't hard-code `LD_LIBRARY_PATH` or `.so`.

## Tests

There is no pytest config. Each test is a standalone script run with `python3 test/<name>.py`, collecting `test_*` functions in `__main__`. Each test adds the directory it tests to `sys.path` itself. To run one test, import the file and call its function. acados tests call `mlcs_mpc.utils.runenv.ensure_acados()`, which sets `ACADOS_SOURCE_DIR`/`LD_LIBRARY_PATH` from `~/acados` and re-execs itself (`ensure_ros2()` does the same for ROS 2 and strips ROS 1).

```bash
python3 test/test_vehicle_model.py   # shared car model: 0/0 guard, real-mass drag, frenet==cartesian, vs original HyperMPC eqs (if ~/hypermpc_code)
python3 test/test_simulator.py       # ODE sim: start/stop, circle radius, steer map inverse, lap on a circle track, low-level (default == old eqs, delay, gym steer/speed, vesc speed delay)
python3 test/test_mpcc_core.py       # env/mpc/core + env/track (Frenet, commands, mocap health)
python3 test/test_opponent.py
python3 test/test_algorithms.py      # legacy iLQR / KF / path manager (README "verified" table)
python3 test/test_map_tools.py; python3 test/test_draw_track.py; python3 test/test_clean_drive_log.py
python3 test/test_hyperpm.py         # learned/hyperpm numpy port vs torch reference in .npz
python3 test/test_ocp_kappa.py [track]  # OCP uses κ(s) correctly (casadi + acados_template, no build)
python3 test/test_stage_params.py    # per-stage p[30] reaches the solver (acados build, ~4 s)
python3 test/test_drivetrain.py      # env/drivetrain VESC low-level model (stop-brake patch, FW speed PID, currents)
python3 test/test_vec_vehicle.py     # torch env/vec == casadi model / Simulator / Track.frenet; RL env API (needs torch)
python3 test/test_realtime.py        # env/estimation (PoseKF == mocap_bridge, kf/gyro lag, fake vy) + env/timing event queue (state→cmd ≈ 34 ms)
python3 test/test_mpcc_node.py [--only NAME]   # Jetson only: real node + acados worker on ROS_DOMAIN_ID=87
```

OCP check without ROS (acados env, i.e. `~/hmpc-venv` on the Jetson):
```bash
export ACADOS_SOURCE_DIR=$HOME/acados LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib
python3 src/mlcs_mpc/mlcs_mpc/env/mpc/offline.py --scenario track --plant --track src/mlcs_mpc/mlcs_mpc/env/track/data/<t>/prep_<t>.csv
```
Generated code is cached per track in `~/mpcc_build/<tag>_trk<hash>/`. It rebuilds automatically when `acados_ocp.py`/`acados_solver.py`/`vehicle_model.py`, ★ config values, baked car values (mass, taus) or the track change.

## Running

- ROS-free tuning loop (acados python): `python3 src/mlcs_mpc/main.py [--track T] [--set v_max=1.2 q_n=0.05] [--car friction=0.35] [--laps N] [-i]` (`-i` = interactive, `--car` = plant-only mismatch). `main_sim.py` is the same in real time with RViz. Config priority: `config/mpcc.yaml` → `race.yaml` `mpcc:` → `--config` files → `--set`; unknown names are rejected.
- `main.py --realtime [--estimator kf|gyro|true] [--vy kf|rot] [--timing solve=jetson ...]`: the real car's async path (`env/timing.py` event queue: mocap 100 Hz ± jitter + transport, 30 Hz timer with Jetson solve-time distribution and pending skip, VESC 50 Hz; distributions from `data/timing/jetson_mocap_20261110.json`) with `env/estimation.py` (the same PoseKF as `io/mocap_bridge`, or `gyro` = vx from wheel speed, r from gyro). The RL env has the same choices as `obs_vel`/`obs_vy` (`train.py --obs-vel`). `tools/analysis/sim_fidelity.py <drive logs>` replays real /drive commands open-loop and reports 1 s position error (servo sweep).
- `./sim.sh build`, then `./sim.sh [race.yaml]`, `./sim.sh track <name> [v_max]`, `./sim.sh go|halt|status`. Runs `sim_node` + `mpcc_node` (`source:=sim`) + RViz, logs to `logs/sim/`. `main.py`/`main_sim.py` save reports/plots to `logs/tune/` (gitignored).
- `./race.sh [race.yaml]` (Jetson): validates `race.yaml`'s `mpcc:` keys against `config/mpcc.yaml` (unknown keys are rejected; `hmpc:` is accepted as the old name), then runs `./drive.sh mpcc`. Map names must be quoted (`"1108_5"`).
- `./drive.sh <mode>` (Jetson): the launcher for everything. It **kills f1tenth_stack's own `joy_teleop`**. `mpcc` (alias `hmpc`) uses env `MLCS_MPCC_*` (old `MLCS_HMPC_*` is still accepted). See its header for all modes.
- `./bat.sh [-w]`: battery voltage via VESC.
- RL (Mac/PC, torch): `python learned/rl/train.py [--profile paper|fast] [--actuator vesc|lag|none]  (default vesc: the firmware PID runs inside the env at 1 kHz) [--dr none|friction|all|full] [--curriculum none|adaptive|linear]` → `logs/rl/<run>/best.pt`. `--profile fast` = MPS, 16384 envs (~35 min / 120M steps vs ~4.6 h for `paper`). Evaluate with `python learned/rl/evaluate.py <ckpt> [--wheel vesc]` (vec env + casadi Simulator, paper metrics + low-level metrics). Make a video with `python learned/rl/render.py <ckpt>... -o out.mp4`. See `docs/RL_RACING.md`.
- The car never moves until `/mpc/enabled` is true (`./drive.sh go` / `halt`).

## Architecture

**ROS package** `src/mlcs_mpc/mlcs_mpc/`. Node names in `ros2 run mlcs_mpc <node>` come from `setup.py` entry points (`mlcs_mpc.<sub>.<module>:main`), so a new node needs an entry there, and a new subpackage needs adding to `packages`. Repo-relative paths in node params (tracks, `param_model` files) resolve against `mpcc_node`'s `repo_root` param (default `~/mlcs_mpc`).
- `env/` is the **ROS-free core** (modeled on race_mpc's `env/`):
  - `config.py`: `MpccConfig`. It holds all MPC settings (N, dt, rk_steps, input/track bounds, cost weights). Values always come from `config/mpcc.yaml`. Node, worker, offline and sim use `MpccConfig.from_params/from_yaml`. Don't hard-code these elsewhere.
  - `vehicle_model.py`: the **single shared car model** (Pacejka single-track + 1st-order actuators, real mass). `frenet()` is used symbolically by the OCP, and `cartesian()` numerically by the simulator. The parameter vector p[30] = vehicle 10 + front tire 10 + rear tire 10.
  - `track/track.py`: closed track with Frenet, per-point walls and `kappa_grid()`.
  - `vesc.py`: the car's VESC tuning (Jetson `f1tenth_stack/config/vesc.yaml`, falling back to `third_party/patches` values). This gives ERPM gain/limit and servo gain/offset/limits.
  - `lowlevel.py` + `config/lowlevel.yaml`: the simulator car's low-level controller (f1tenth_gym-style layer between `/drive` targets and the dynamics): command delay buffers, steer law `lag`|`gym` (bang-bang) + rate limit, speed law `lag`|`gym` (P) + accel/decel limits, `speed_scale` (real speed / erpm conversion, measured 1.044). Evaluated at every RK4 stage, fed to the unchanged shared model as `u = x + τ·ẋ`. `LowLevelConfig()` = old behavior (tests); `main.py`/`sim_node` load `config/lowlevel.yaml` (sim-only real-car values; MPC values stay in `mpcc.yaml`), `--lowlevel KEY=VALUE|gym|none` overrides.
  - `simulator.py`: the ODE simulator. `step(steer_cmd, speed_cmd)` clips the commands like the VESC, maps servo command to actual steer, then integrates the model (RK4 substeps, or scipy `solve_ivp`). It blends to a kinematic model below 0.2–0.4 m/s. `wheel_model='lag'` (default, the MPC's 1st-order wheel) or `'vesc'` (`drivetrain.py`: the stop-brake patch, a port of the VESC firmware speed PID, motor current, then wheel force. Selected with `--wheel vesc` / sim_node `wheel_model`). `docs/VEHICLE_DYNAMICS.md` compares these with f1tenth_gym and the real logs.
  - `vec/`: **batched torch port** of the same car model (`vehicle.py`, must stay equation-identical to `vehicle_model.py`, checked by `test_vec_vehicle.py`), `track.py` (batched Frenet) and `race_env.py`, the RL env from arXiv:2504.02420. Its two experiment axes are `actuator` (`lag` + motor limits | `vesc` = torch port of `drivetrain.py` in `vec/drivetrain.py`, integrated at 1 ms | `none`) and `dr` (`none`|`friction`|`all`|`full`, scaled by the curriculum `dr_level`; sensor noise is injected at the mocap reference point = **front-axle center**). Envs never auto-reset (`reset(ids)`). Stopping (<0.3 m/s for 1 s) terminates with −1, like a wall hit. Without the penalty, policies learned to stop in the hairpin. If you change `vehicle_model.py`, mirror the change in `vec/vehicle.py`.
  - `mpc/`: `acados_ocp.py` (OCP: curvature κ(s) is a **B-spline of the track baked into the generated code** and evaluated from state s; p[30] is set **per stage for every solve**), `acados_solver.py` (options + cached build), `bounds.py`, `core.py` (state conversion, commands, steer map), `mpcc.py` (`MPCC` solver wrapper used by the worker, and `MpccController` = one control cycle of `mpcc_node`, used by `main.py`), `param_model.py`, `worker.py`, `offline.py`.
  - `racing.py`: shared setup for `main.py`/`main_sim.py` (`setup()` → params, track, controller, sim; `control_tick()`). It runs the same computation as the ROS path.
- `control/mpcc_node.py` is the main controller (control_rate 30 Hz). The solve runs in a **separate process** (`env/mpc/worker.py`, binary pipe; request `[id, cmd, t, x(9), n_lo(N+1), n_hi(N+1), v_cap(N)]`) because acados `solve()` can hang on NaN, and the VESC keeps its last speed when commands stop. On timeout the node SIGKILLs the worker, keeps publishing speed 0, and latches faults until `/mpc/enabled` false→true. Its worker config is its own ROS params (MpccConfig names + car values + track). `control/legacy/` holds the old iLQR `mpc_node` and `stanley_node`.
- `io/`: state sources that all publish `/mpc/state` (rear-axle pose + body twist) and `/mocap/valid`. These are `mocap_bridge` (100 Hz), `lidar_bridge` and `sim_node`. `sim_node` also publishes `/sensors/core` (50 Hz, ERPM), so the controller sees exactly what it sees on the car.
- `ui/`, `utils/`: joystick (mux priority joystick 100 > navigation 10; speed 0 on `/joy` dropout), RViz, safety, calibration, loggers.

A safety invariant applies to all nodes: **stopping publishing does not stop the car**. Every failure path must actively publish speed 0.

**Learned params**: `env/mpc/param_model.py` is the interface the worker calls each solve (`observe`, `stage_params → (N+1, 30)`, `set_plan`, `clear_plan`). `ConstantParams` is the default. Top-level `learned/rl/` is PPO for `env/vec/race_env.py` (paper hyperparameters; `train.py`, `evaluate.py`). Top-level `learned/hyperpm/` is HyperPM (numpy inference, `export_hyper.py`, `train/`). The config key is `param_model: 'hyperpm:<npz>'`. **Never remove `WANDB_MODE=disabled`** from `learned/hyperpm/train/train.sh`, because upstream `main.py` hardcodes the author's W&B key. `train/` needs `~/hypermpc_code`; nothing else does.

**Tracks/maps**: `src/mlcs_mpc/mlcs_mpc/env/track/data/<name>/prep_<name>.csv` (`s,x,y,heading,curvature,track_width[,w_right,w_left]`; `find(name)` resolves it, and `Track` actually reads the spline-smoothed `ref_<name>.csv` it generates next to it). `rl17_track` is the paper's 17 m track digitized from its Fig. 6 (`tools/track/track_from_paper_2504_02420.py`). Frames must match the localization source (`*_mocap` tracks are for mocap, map-frame tracks are for LiDAR). They are produced by `tools/track/` (usually via `drive.sh maptrack/laptrack/preptrack`). Cartographer maps live in `data/maps/`, and the lab boundary is `data/lab_map/data/boundary/map_boundary_data.json`.

**Data pipeline**: `drive.sh log`/`--log` → `~/mlcs_data/drive_*.csv` → `drive.sh cleanlog` → `drive.sh hpmdata` → `learned/hyperpm/train/train.sh` → `export_hyper.py` → `config/hyper/*.npz`.

**`third_party/`**: `patches/` holds local fixes to repos in `~/f1tenth_ws/src` (applied with `third_party/patches/apply.sh`; the VESC calibration values live in `f1tenth_system-servo-offset.patch`). `vesc_msgs/` is for the sim PC only (switched to plain `ament_cmake`). Don't put it in the Jetson workspace, which already has vesc_msgs.

## Docs

`docs/SETUP_JETSON.md`, `docs/SETUP_MAC.md`, `docs/VEHICLE_DYNAMICS.md`, `docs/RL_RACING.md`, `docs/MOCAP_SETUP.md`, `docs/CALIBRATION.md`, `docs/TRACKS.md`, `docs/ROADMAP.md`, `src/mlcs_mpc/mlcs_mpc/env/mpc/README.md` (OCP files, track pipeline, `track_width` is a half-width). `docs/REQUIREMENTS.txt` (requirements, which real-car values were applied and which were deliberately not, environments) and `docs/REFERENCES.txt` (papers/code and where each is used) — update both when requirements or citations change. VESC config XMLs live in `docs/vesc/` (`drivetrain.VESC_XML` = the latest), reference PDFs/original author code in `docs/references/`, competition track notes in `docs/competition_tracks/`. RL experiment recipes are `learned/rl/experiments/*.sh` (results go to `logs/rl/exp/`). `docs/notes/` holds dated historical logs, so don't rewrite old paths there. README's verified/unverified table is the project's honesty convention: update it when something gets verified.
