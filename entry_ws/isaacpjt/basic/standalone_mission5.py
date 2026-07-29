from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube = DynamicCuboid(                                   # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 0.15]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube)

world.reset()

step_count = 0
reset_needed = False

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)

    # Stop 상태를 지나갔다는 사실을 기억해 둔다
    if world.is_stopped() and not reset_needed:
        reset_needed = True

    if world.is_playing():
        # Stop -> Play 로 넘어온 첫 프레임에서만 처음 상태로 되돌린다
        if reset_needed:
            world.reset()
            step_count = 0
            reset_needed = False
            print(f"[리셋] Play 시작 -> step_count = {step_count}")

        step_count += 1

        if step_count % 100 == 0:
            print(f"step: {step_count}")

        if step_count == 300:
            # 물리 속도를 죽이고 1m 높이로 순간이동
            cube.set_world_pose(position=np.array([0.0, 0.0, 1.0]))
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))
            print("[이동] 큐브 순간이동")

        if step_count == 500:
            simulation_app.close()


simulation_app.close()
