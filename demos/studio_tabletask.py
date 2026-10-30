"""Teleoperate the tabletask floating gripper in MuJoCo Studio with your phone.

Move the phone to move the gripper; tap Toggle in the app to close or open it.
Needs tabletask and a mujoco build with Studio (pip install --pre mujoco -f https://py.mujoco.org/).

    python demos/studio_tabletask.py                                  # banana in bowl
    python demos/studio_tabletask.py --task robolab-marker_in_mug --app-clip
"""
import argparse
import dataclasses
import os

import mujoco
import numpy as np
import tabletask
from tabletask.language.monitor import Monitor
from mujoco.studio import launch_passive, messages, plugin, viewer_app, viewer_protocol
from mujoco.studio.dear_imgui import dear_imgui as imgui
from tabletask.robots import Gripper

from teledex.studio import TeleDexPanel, TeleDexSim

STATE = mujoco.mjtState.mjSTATE_INTEGRATION
TASKS = os.environ.get("TABLETASK_TASKS", os.path.expanduser("~/Documents/tabletask/tasks/base"))


@dataclasses.dataclass(frozen=True)
class TaskSnapshot(messages.Snapshot):
    instruction: str
    stages_done: int
    stages_total: int
    success: bool
    violated: str


class TaskMonitor(plugin.SimPlugin):
    """Checks the task's goal on the live simulation with tabletask's own monitor."""

    def __init__(self, env):
        self.env = env
        self.monitor = Monitor(env.task)
        self.status = self.monitor.reset(env.runtime)
        self._last_check = -1.0
        # The sampled, settled start, which Studio's Reset would otherwise lose
        model, data = env.runtime.model, env.runtime.data
        self._start = np.zeros(mujoco.mj_stateSize(model, STATE))
        mujoco.mj_getState(model, data, self._start, STATE)

    # Priority below Studio's own reset handler (INTERNAL), so this runs after it
    @messages.handler(priority=messages.Priority.INTERNAL - 1)
    def on_reset(self, event: messages.ResetEvent) -> None:
        if event.key is None or event.key < 0:
            model, data = self.env.runtime.model, self.env.runtime.data
            mujoco.mj_setState(model, data, self._start, STATE)
            mujoco.mj_forward(model, data)
        self.status = self.monitor.reset(self.env.runtime)
        self._last_check = -1.0

    @messages.handler
    def on_post_step(self, event: messages.PostStepEvent) -> None:
        if event.data.time - self._last_check < 0.05:
            return
        self._last_check = event.data.time
        self.status = self.monitor.update(self.env.runtime, event.data.time)
        task = self.env.task
        self.handle.send_to_viewer(TaskSnapshot(
            instruction=task.instruction,
            stages_done=len(self.status.done),
            stages_total=len(task.goal.stages),
            success=self.status.succeeded(task),
            violated=self.status.violated or "",
        ))


class TaskPanel(plugin.ViewerPlugin):
    def __init__(self):
        self.state = None

    @messages.handler
    def on_task(self, snapshot: TaskSnapshot) -> None:
        self.state = snapshot

    @messages.handler
    def on_build_gui(self, _: messages.BuildGuiEvent) -> None:
        imgui.SetNextWindowPos(imgui.Vec2(222, 400), imgui.Cond.Once)
        expanded, _ = imgui.Begin("Task")
        if expanded and self.state is not None:
            imgui.TextUnformatted(self.state.instruction)
            if self.state.success:
                imgui.TextColored(imgui.Vec4(0.3, 0.8, 0.45, 1.0), "Success!")
            elif self.state.violated:
                imgui.TextColored(imgui.Vec4(0.95, 0.35, 0.3, 1.0), self.state.violated)
            else:
                imgui.TextUnformatted(f"Stages: {self.state.stages_done}/{self.state.stages_total}")
            imgui.TextUnformatted("Toggle in the app opens/closes the gripper. Reset restarts the task.")
        imgui.End()


def build_task(name, episode):
    env = tabletask.make(os.path.join(TASKS, f"{name}.json"), robot=Gripper())
    env.cameras = False
    env.reset(episode)
    model, data = env.runtime.model, env.runtime.data

    # Start the camera behind the gripper looking along +x, so pushing the phone forward
    # moves the gripper away from you
    model.stat.center[:] = [0.55, 0.0, 0.85]
    model.stat.extent = 0.75
    model.vis.global_.azimuth = 0.0
    model.vis.global_.elevation = -30.0
    return env, model, data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", default="robolab-banana_in_bowl", help=f"Task name in {TASKS}")
    parser.add_argument("--episode", type=int, default=0, help="Start to sample (the seed).")
    parser.add_argument("--scale", type=float, default=2.0, help="Gripper motion per metre of phone motion.")
    parser.add_argument("--app-clip", action="store_true", help="Encode the QR code as a TeleDex App Clip link.")
    parser.add_argument("--native", action="store_true", help="Open a desktop window instead of the browser viewer.")
    args = parser.parse_args()

    env, model, data = build_task(args.task, args.episode)
    gripper = env.runtime.robot
    actuators = [model.actuator(int(a)).name for a in np.ravel(gripper.gripper_actuators)]
    low, high = model.actuator_ctrlrange[int(np.ravel(gripper.gripper_actuators)[0])]
    # The phone drives the gripper body; keep its fingertip point (TCP) just above the table
    tcp_height = float(np.atleast_2d(gripper.tcp_offset)[0][2])
    table_top = float(env.runtime.aabb("table/top")[0][2])  # tabletask varies the table height

    config = viewer_protocol.ViewerConfig(title="TeleDex", gfx="" if args.native else "web", open_browser=True)
    launch_passive.run(
        config,
        model=model,
        data=data,
        plugins=[
            viewer_app.ViewerApp(),
            *viewer_app.SIM_PLUGINS,
            TeleDexSim(
                body="gripper/target",
                scale=args.scale,
                gripper_actuator=actuators,
                gripper_ctrl=(low, high),  # Robotiq: 0 open, 255 closed; Toggle on closes
                position_limits=[[-0.2, 1.4], [-0.9, 0.9], [table_top - 0.005 + tcp_height, 1.6]],
                app_clip=args.app_clip,
            ),
            TeleDexPanel(),
            TaskMonitor(env),
            TaskPanel(),
        ],
    )


if __name__ == "__main__":
    main()
