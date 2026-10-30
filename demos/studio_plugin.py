"""TeleDex plugins for the Studio command line (drives the model's first mocap body):

    python -m mujoco.studio.viewer --model=demos/studio_scene.xml --gfx=web --plugins=demos/studio_plugin.py
"""
from teledex import studio


class TeleDexSim(studio.TeleDexSim):
    pass


class TeleDexPanel(studio.TeleDexPanel):
    pass
