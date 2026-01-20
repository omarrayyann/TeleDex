import mujoco
import mujoco.viewer
from teledex import Session, MujocoHandler
import random

def main():

    # MuJoCo initialization
    model = mujoco.MjModel.from_xml_path('demos/simple_scene.xml')
    data = mujoco.MjData(model)
    target_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "target")

    # Teledex Setup
    session = Session()
    mujoco_handler = MujocoHandler(model=model, data=data)
    session.add_handler(mujoco_handler)
    

    def button_press():
        model.site_rgba[target_site_id][:3] = [random.random() for _ in range(3)]

    def toggle_press():
        if model.site_size[target_site_id][0] == 0.05:
            model.site_size[target_site_id][0] = 0.1
        else:
            model.site_size[target_site_id][0] = 0.05

    # Linking the target site with the AR position
    mujoco_handler.link_site(
        name="target",
        scale=3.0,
        position_origin=[0.0, 0.0, 0.2],
        post_transform=[
                [0,-1, 0, 0],
                [1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
        disable_rot=False,
        button_fn=button_press,
        toggle_fn=toggle_press,
    )

    # Start Teledex
    session.start()

    try:
        with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
    except KeyboardInterrupt:
        session.stop()

if __name__ == "__main__":
    main()
