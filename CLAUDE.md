# Pupper Experiments

## Overview
This workspace contains a MuJoCo simulation setup for the Pupper V3 robot model.

## Main files
- sim.py: launches the MuJoCo viewer and runs a simple trot-style gait controller.
- pupper_v3_description/: robot description files, meshes, and MuJoCo XML assets.

## Running the simulation
From the repository root:

```bash
conda activate pupper-sim
mjpython sim.py
```

If MuJoCo is installed in a different environment, use that Python interpreter instead.

## Notes about the model
The simulation currently uses:
- pupper_v3_description/description/mujoco_xml/pupper_v3_complete.position.fixed_base.xml

This is a fixed-base MuJoCo model, so it is suitable for basic locomotion testing without worrying about body dynamics from a floating base.

## How sim.py works
1. Loads the MuJoCo model from the repository.
2. Initializes the simulation state.
3. Disables mesh geoms in the model so the viewer shows the simplified visual setup.
4. Defines a trot gait for the four legs.
5. Drives the hip and knee actuators with sinusoidal commands.
6. Steps the simulation and updates the viewer in a loop.

## Common tasks
- Change gait parameters in sim.py: FREQ, HIP_AMP, KNEE_AMP.
- Adjust the leg layout in the LEGS list if you want different gait timing.
- Swap in another MuJoCo XML file from the description/mujoco_xml folder if you want a different robot configuration.
