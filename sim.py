import time
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer

# Run with: mjpython sim.py
# mjpython is at: /opt/homebrew/Caskroom/miniforge/base/envs/pupper-sim/bin/mjpython

REPO_ROOT = Path(__file__).resolve().parent
MODEL_PATH = REPO_ROOT / "pupper_v3_description/description/mujoco_xml/pupper_v3_complete.position.fixed_base.xml"

m = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)

MESH = int(mujoco.mjtGeom.mjGEOM_MESH)
for i in range(m.ngeom):
    if m.geom_type[i] == MESH:
        m.geom_group[i] = 0

# Actuator layout — verified by perturbing qpos and measuring foot displacement:
#  _1 joints (0,3,6,9)  → foot moves forward/back = hip flexion
#  _2 joints (1,4,7,10) → foot moves sideways     = abduction (don't drive for trot)
#  _3 joints (2,5,8,11) → knee flexion
#
#  0 FR_hip_flex  1 FR_abduct  2 FR_knee
#  3 FL_hip_flex  4 FL_abduct  5 FL_knee
#  6 BR_hip_flex  7 BR_abduct  8 BR_knee
#  9 BL_hip_flex 10 BL_abduct 11 BL_knee

# Trot gait: diagonal pairs (FR+BL) and (FL+BR) alternate 180° out of phase.
# Each entry: (hip_flex_ctrl_idx, knee_ctrl_idx, phase_offset)
LEGS = [
    (0,  2,  0),    # Front-right
    (9,  11, 0),    # Back-left  (same phase as FR — diagonal pair)
    (3,  5,  np.pi),  # Front-left
    (6,  8,  np.pi) # Back-right (same phase as FL — diagonal pair)
 ] 
FREQ     = 1.5   # gait frequency in Hz
HIP_AMP  = 0.1 # hip swing amplitude (radians)
KNEE_AMP = 0.7  # knee flex amplitude (radians)

with mujoco.viewer.launch_passive(m, d) as v:
    t0 = time.time()
    while v.is_running():
        t = time.time() - t0
        w = 2 * np.pi * FREQ

        for hip_idx, knee_idx, phase in LEGS:
            d.ctrl[hip_idx] = HIP_AMP  * np.sin(w * t + phase)
            # Knee bends opposite to hip so the foot lifts during swing
            d.ctrl[knee_idx] = KNEE_AMP * np.sin(w * t + phase + np.pi)

        mujoco.mj_step(m, d)
        v.sync()
