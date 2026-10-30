"""TeleDex plugins for MuJoCo Studio (requires a mujoco build that ships ``mujoco.studio``).

``TeleDexSim`` runs on Studio's simulation thread: it owns a ``teledex.Session`` and moves a
mocap body to the phone's pose right before every physics step. ``TeleDexPanel`` runs on the
viewer thread: it shows the connection QR code, status and live pose, and draws the commanded
target in the scene.

Example:

    from mujoco.studio import launch_passive, viewer_app, viewer_protocol
    from teledex.studio import TeleDexSim, TeleDexPanel

    launch_passive.run(
        viewer_protocol.ViewerConfig(gfx="web"),
        model_path="scene.xml",
        plugins=[viewer_app.ViewerApp(), *viewer_app.SIM_PLUGINS, TeleDexSim(body="target"), TeleDexPanel()],
    )
"""
import dataclasses
import logging
import threading
import time

import mujoco
import numpy as np
from mujoco.studio import messages
from mujoco.studio import plugin
from mujoco.studio.dear_imgui import dear_imgui as imgui
from scipy.spatial.transform import Rotation as R

from .main import Session

try:
    import qrcode
    HAS_QRCODE = True
except ImportError:
    HAS_QRCODE = False

logger = logging.getLogger(__name__)


@dataclasses.dataclass(frozen=True, eq=False)
class TeleDexSnapshot(messages.Snapshot):
    """Sim -> viewer: latest TeleDex state for the panel and scene overlay."""

    connected: bool
    address: str
    qr_payload: str
    body: str
    mode: str
    paused: bool
    phone_position: np.ndarray | None
    target_position: np.ndarray | None
    target_rotation: np.ndarray | None
    button: bool
    toggle: bool
    error: str = ""


@dataclasses.dataclass(frozen=True)
class TeleDexCommandEvent(messages.Event):
    """Viewer -> sim: a panel action ("pause", "resume", "recenter" or "refresh")."""

    action: str


class TeleDexSim(plugin.SimPlugin):
    """Drives a mocap body from the TeleDex phone pose before every physics step.

    Args:
        body: Name of the mocap body to drive. Defaults to the model's first mocap body.
        port: Port for the teledex Session.
        scale: Multiplier from phone motion (meters) to target motion.
        frame: (3, 3) rotation from the teledex frame (x forward, y left, z up) to the world.
        mode: "absolute" follows the phone pose (default); "delta" accumulates pose changes
              with per-step caps, for rate-limited targets.
        max_dpos: Delta mode only: cap on translation per step, in meters.
        max_drot: Delta mode only: cap on rotation per step, in radians.
        position_limits: Optional (3, 2) [[xmin, xmax], [ymin, ymax], [zmin, zmax]] world bounds.
        disable_rot: Only drive position; keep the body's orientation.
        gripper_actuator: Optional actuator name, or list of names, driven by the app's Toggle.
        gripper_ctrl: (open, closed) controls for the gripper actuators. Defaults to the low and
                      high ends of each actuator's ctrlrange.
        app_clip: Encode the QR code as a TeleDex App Clip link.
    """

    def __init__(self, body=None, port=8888, scale=1.0, frame=None, mode="absolute",
                 max_dpos=None, max_drot=None, position_limits=None, disable_rot=False,
                 gripper_actuator=None, gripper_ctrl=None, app_clip=False):
        if mode not in ("absolute", "delta"):
            raise ValueError('mode must be "absolute" or "delta"')
        self.body = body
        self.port = port
        self.scale = scale
        self.frame = np.eye(3) if frame is None else np.asarray(frame, dtype=float)
        self.mode = mode
        self.max_dpos = max_dpos
        self.max_drot = max_drot
        self.position_limits = None if position_limits is None else np.asarray(position_limits, dtype=float)
        self.disable_rot = disable_rot
        self.gripper_actuators = ([gripper_actuator] if isinstance(gripper_actuator, str)
                                  else list(gripper_actuator or []))
        self.gripper_ctrl = gripper_ctrl
        self.app_clip = app_clip

        self.session = None
        self._mocap_id = -1
        self._body_name = ""
        self._actuator_ids = []
        self._error = ""
        self._paused = False
        self._recenter = True  # take the phone's current pose as the reference on the next step
        self._phone_ref = (np.zeros(3), np.eye(3))
        self._target_ref = (np.zeros(3), np.eye(3))
        self._target = None  # commanded (position, rotation)
        self._was_connected = False
        self._last_snapshot_time = 0.0

    # -- Lifecycle -----------------------------------------------------------------

    @messages.handler
    def on_sim_init(self, _: plugin.SimInitEvent) -> None:
        self.session = Session(port=self.port, app_clip=self.app_clip, delta=True)
        self.session.start()

    @messages.handler
    def on_post_model(self, event: messages.PostModelEvent) -> None:
        model = event.model
        self._mocap_id, self._body_name, self._error = -1, "", ""

        body_id = (mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.body) if self.body
                   else next((b for b in range(model.nbody) if model.body_mocapid[b] >= 0), -1))
        if body_id < 0:
            self._error = f'No body named "{self.body}"' if self.body else "Model has no mocap body"
        elif model.body_mocapid[body_id] < 0:
            self._error = f'Body "{self.body}" is not a mocap body (add mocap="true")'
        else:
            self._mocap_id = int(model.body_mocapid[body_id])
            self._body_name = model.body(body_id).name

        self._actuator_ids = []
        for name in self.gripper_actuators:
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if actuator_id < 0:
                self._error = f'No actuator named "{name}"'
            else:
                self._actuator_ids.append(actuator_id)

        if self._error:
            logger.warning("TeleDex: %s", self._error)
            self._target = None
        else:
            self._target = self._mocap_pose(event.data)
        self._recenter = True

    @messages.handler
    def on_command(self, event: TeleDexCommandEvent) -> None:
        if event.action == "pause":
            self._paused = True
        elif event.action == "resume":
            self._paused = False
            self._recenter = True  # continue from where the target is, without a jump
        elif event.action == "recenter":
            self._recenter = True
        elif event.action == "refresh" and self.session is not None:
            # The server listens on all interfaces, so after a network switch only the advertised
            # address (and QR code) is stale. Detect it off the sim thread: it can block for ~1 s.
            session = self.session
            threading.Thread(target=lambda: setattr(session, "ip_address", session._get_local_ip()),
                             daemon=True).start()

    @messages.handler
    def on_reset(self, _: messages.ResetEvent) -> None:
        # Studio has just moved the mocap body back to its initial pose; follow from there
        self._recenter = True

    @messages.handler
    def on_pre_step(self, event: messages.PreStepEvent) -> None:
        if self.session is None:
            return
        latest = self.session.get_latest_data()
        connected = bool(self.session.connected_clients)
        if connected and not self._was_connected:
            self._recenter = True  # a (re)connected phone starts from its own origin
        self._was_connected = connected

        if self._mocap_id >= 0 and connected and latest["position"] is not None:
            phone = (np.array(latest["position"], dtype=float),
                     np.array(latest["rotation"], dtype=float).reshape(3, 3))
            self._update_target(event.data, phone)
            for actuator_id in self._actuator_ids:
                opened, closed = (self.gripper_ctrl if self.gripper_ctrl is not None
                                  else event.model.actuator_ctrlrange[actuator_id])
                event.data.ctrl[actuator_id] = closed if latest["toggle"] else opened

        # Physics can step much faster than the display; the panel only needs ~60 Hz
        now = time.monotonic()
        if now - self._last_snapshot_time < 1.0 / 60.0:
            return
        self._last_snapshot_time = now

        self.handle.send_to_viewer(TeleDexSnapshot(
            connected=connected,
            address=f"{self.session.ip_address}:{self.session.port}",
            qr_payload=self.session.connection_link() if self.app_clip else f"{self.session.ip_address}:{self.session.port}",
            body=self._body_name,
            mode=self.mode,
            paused=self._paused,
            phone_position=None if latest["position"] is None else np.array(latest["position"], dtype=float),
            target_position=None if self._target is None else self._target[0].copy(),
            target_rotation=None if self._target is None else self._target[1].copy(),
            button=bool(latest["button"]),
            toggle=bool(latest["toggle"]),
            error=self._error,
        ))

    @messages.handler
    def on_exit(self, _: messages.ExitEvent) -> None:
        if self.session is not None:
            self.session.stop()
            self.session = None

    # -- Target computation ----------------------------------------------------------

    def _update_target(self, data, phone):
        if self._recenter:
            self._phone_ref = (phone[0].copy(), phone[1].copy())
            self._target_ref = self._mocap_pose(data)
            self._target = self._target_ref
            self.session.reset_delta()
            self._recenter = False

        if self._paused:
            self.session.reset_delta()
            return

        if self.mode == "absolute":
            ref_pos, ref_rot = self._phone_ref
            dpos = phone[0] - ref_pos
            drot = phone[1] @ ref_rot.T
            base_pos, base_rot = self._target_ref
        else:
            delta = self.session.get_delta(max_dpos=self.max_dpos, max_drot=self.max_drot)
            dpos, drot = delta["dpos"], delta["drot"]
            base_pos, base_rot = self._target

        position = base_pos + self.frame @ (dpos * self.scale)
        rotation = base_rot if self.disable_rot else (self.frame @ drot @ self.frame.T) @ base_rot
        if self.position_limits is not None:
            position = np.clip(position, self.position_limits[:, 0], self.position_limits[:, 1])
        self._target = (position, rotation)

        data.mocap_pos[self._mocap_id] = position
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, rotation.flatten())
        data.mocap_quat[self._mocap_id] = quat

    def _mocap_pose(self, data):
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, data.mocap_quat[self._mocap_id])
        return data.mocap_pos[self._mocap_id].copy(), rotation.reshape(3, 3)


class TeleDexPanel(plugin.ViewerPlugin):
    """Studio window with the TeleDex QR code, connection status and live pose."""

    QR_MODULE_PX = 4

    def __init__(self, show_target=True):
        self.show_target = show_target
        self._state = None
        self._qr_payload = ""
        self._qr_texture = 0
        self._qr_size = 0
        self._geoms = []

    @messages.handler
    def on_state(self, snapshot: TeleDexSnapshot) -> None:
        self._state = snapshot

    @messages.handler
    def on_update(self, _: messages.UpdateEvent) -> None:
        # Remove only our own overlay geoms so other plugins' overlays survive
        if self._geoms:
            self.viewer.extra_geoms[:] = [g for g in self.viewer.extra_geoms if all(g is not m for m in self._geoms)]
            self._geoms = []

        state = self._state
        if state is None:
            return

        if HAS_QRCODE and state.qr_payload != self._qr_payload:
            self._upload_qr(state.qr_payload)

        if self.show_target and state.target_position is not None and not state.error:
            self._draw_axes(state.target_position, state.target_rotation, faded=state.paused or not state.connected)

    @messages.handler
    def on_build_gui(self, _: messages.BuildGuiEvent) -> None:
        # Open next to ViewerApp's Options sidebar on launch; users can move or dock it after
        imgui.SetNextWindowPos(imgui.Vec2(222, 36), imgui.Cond.Once)
        expanded, _ = imgui.Begin("TeleDex")
        if expanded:
            self._build_gui()
        imgui.End()

    def _build_gui(self):
        state = self._state
        if state is None:
            imgui.TextUnformatted("Starting TeleDex session...")
            return

        if state.error:
            imgui.TextColored(imgui.Vec4(0.95, 0.35, 0.3, 1.0), state.error)
        elif state.connected:
            imgui.TextColored(imgui.Vec4(0.3, 0.8, 0.45, 1.0), "Phone connected")
        else:
            imgui.TextColored(imgui.Vec4(0.95, 0.7, 0.25, 1.0), "Waiting for phone")

        imgui.TextUnformatted(f"Address: {state.address}")
        if not state.connected:
            if self._qr_texture:
                app_clip = state.qr_payload.startswith("https://")
                imgui.TextUnformatted("Scan with the iPhone Camera:" if app_clip else "Scan in the TeleDex app:")
                imgui.Image(self._qr_texture, imgui.Vec2(self._qr_size, self._qr_size))
            elif not HAS_QRCODE:
                imgui.TextUnformatted("pip install qrcode to show the QR code here")

        imgui.Separator()
        imgui.TextUnformatted(f"Body: {state.body or '-'}    Mode: {state.mode}")
        if state.phone_position is not None:
            x, y, z = state.phone_position
            imgui.TextUnformatted(f"Phone   x {x:+.3f}  y {y:+.3f}  z {z:+.3f}")
        if state.target_position is not None:
            x, y, z = state.target_position
            imgui.TextUnformatted(f"Target  x {x:+.3f}  y {y:+.3f}  z {z:+.3f}")
        imgui.TextUnformatted(f"Button: {'down' if state.button else 'up'}    Toggle: {'on' if state.toggle else 'off'}")

        imgui.Separator()
        if imgui.Button("Resume" if state.paused else "Pause"):
            self.viewer.send_to_sim(TeleDexCommandEvent("resume" if state.paused else "pause"))
        imgui.SameLine()
        if imgui.Button("Recenter"):
            self.viewer.send_to_sim(TeleDexCommandEvent("recenter"))
        imgui.SameLine()
        if imgui.Button("Refresh"):
            self.viewer.send_to_sim(TeleDexCommandEvent("refresh"))
        if imgui.IsItemHovered():
            imgui.SetTooltip("Re-detect this computer's address, e.g. after switching Wi-Fi")

    def _upload_qr(self, payload):
        qr = qrcode.QRCode(border=2, error_correction=qrcode.constants.ERROR_CORRECT_L)
        qr.add_data(payload)
        qr.make(fit=True)
        modules = np.array(qr.get_matrix(), dtype=bool)
        pixels = np.where(modules, 0, 255).astype(np.uint8)
        pixels = np.kron(pixels, np.ones((self.QR_MODULE_PX, self.QR_MODULE_PX), dtype=np.uint8))
        rgba = np.stack([pixels, pixels, pixels, np.full_like(pixels, 255)], axis=-1)
        size = rgba.shape[0]
        self._qr_texture = self.viewer.upload_image(self._qr_texture, rgba.tobytes(), size, size, 4)
        self._qr_size = size
        self._qr_payload = payload

    def _draw_axes(self, position, rotation, faded):
        alpha = 0.35 if faded else 0.9
        length, width = 0.1, 0.004
        for axis in range(3):
            color = np.array([0.15, 0.15, 0.15, alpha], dtype=np.float32)
            color[axis] = 0.95
            geom = mujoco.MjvGeom()
            # Capsules rather than arrows: the web viewer's renderer only draws regular geom types
            mujoco.mjv_initGeom(geom, int(mujoco.mjtGeom.mjGEOM_CAPSULE), np.zeros(3), np.zeros(3),
                                np.eye(3).flatten(), color)
            mujoco.mjv_connector(geom, int(mujoco.mjtGeom.mjGEOM_CAPSULE), width,
                                 position, position + length * rotation[:, axis])
            self.viewer.extra_geoms.append(geom)
            self._geoms.append(geom)

    @messages.handler
    def on_post_model(self, _: messages.PostModelEvent) -> None:
        self._state = None
        self._geoms = []  # the viewer clears extra_geoms on model load
