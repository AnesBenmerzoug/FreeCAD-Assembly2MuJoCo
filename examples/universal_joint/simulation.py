# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "mujoco",
# ]
# ///

import math
import time

import mujoco
import mujoco.viewer

STEP_SIZE = 0.01


def main() -> None:
    m = mujoco.MjModel.from_xml_path("mujoco/universal_joint.xml")
    d = mujoco.MjData(m)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            step_start = time.monotonic()

            d.qpos[:] = [
                2.0 * math.sin(0.25 * math.pi * step_start),
                0.3 * math.cos(1 * math.pi * step_start),
                0.3 * math.sin(1 * math.pi * step_start),
            ]

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(m, d)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            # time_until_next_step = m.opt.timestep - (time.monotonic() - step_start)
            time_until_next_step = STEP_SIZE - (time.monotonic() - step_start)
            if time_until_next_step > 0:
                time.sleep(STEP_SIZE)


if __name__ == "__main__":
    main()
