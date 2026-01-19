import mujoco
import mujoco.viewer
from teledex import TeledexConnector
import random

def main():

    # MuJoCo initialization
    model = mujoco.MjModel.from_xml_path('demos/box_scene.xml')
    data = mujoco.MjData(model)
    target_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "target")

    # Teledex Setup
    teledex = TeledexConnector(mujoco_model=model, mujoco_data=data)
    

    def button_press():
        model.site_rgba[target_site_id][:3] = [random.random() for _ in range(3)]

    def toggle_press():
        if model.site_size[target_site_id][0] == 0.05:
            model.site_size[target_site_id][0] = 0.1
        else:
            model.site_size[target_site_id][0] = 0.05

    def vibrate_when():
        pos = model.site("target").pos.copy()
        return abs(pos[0]) == 0.3 or abs(pos[1]) == 0.3 or abs(pos[2]-0.3) == 0.3

    # Linking the target site with the AR position
    teledex.link_site(
        name="target",
        scale=3.0,
        position_origin=[0.0, 0.0, 0.2],
        post_transform=[
                [0,-1, 0, 0],
                [1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
        position_limits = [[-0.3,0.3],[-0.3,0.3],[0,0.6]],
        disable_rot=True,
        button_fn=button_press,
        toggle_fn=toggle_press,
        vibrate_fn=vibrate_when,
    )

    # Start Teledex
    teledex.start()

    try:
        with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
    except KeyboardInterrupt:
        teledex.stop()

if __name__ == "__main__":
    main()
