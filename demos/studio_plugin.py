"""TeleDex plugins for the Studio command line (drives the model's first mocap body):

    python -m mujoco.studio.viewer --model=demos/scene0.xml --gfx=web --plugins=demos/studio_plugin.py
"""
from teledex import studio


class TeleDexSim(studio.TeleDexSim):
    def __init__(self):
        super().__init__(scale=1.25)  # the ladle moves 1.25 cm per cm of phone motion


class TeleDexPanel(studio.TeleDexPanel):
    pass
