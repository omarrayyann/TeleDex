"""TeleDex plugins for MuJoCo Studio (requires a mujoco build that ships ``mujoco.studio``).

``TeleDexSim`` runs on Studio's simulation thread: it owns a ``teledex.Session`` and moves a
mocap body to the phone's pose right before every physics step. ``TeleDexPanel`` runs on the
viewer thread: it shows the connection QR code, status and live pose, and draws the commanded
target in the scene.

Example:

    from mujoco.studio import launch_passive, viewer_protocol
    from teledex.studio import TeleDexSim, TeleDexPanel, studio_plugins

    launch_passive.run(
        viewer_protocol.ViewerConfig(gfx="web"),
        model_path="scene.xml",
        plugins=[*studio_plugins(), TeleDexSim(body="target"), TeleDexPanel()],
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


def studio_plugins():
    """Studio's built-in app and simulation plugins, for ``launch_passive.run``."""
    try:  # nightlies from Nov 2026 on
        from mujoco.studio import app
        return [app.App(), *app.SIM_PLUGINS]
    except ImportError:
        from mujoco.studio import viewer_app
        return [viewer_app.ViewerApp(), *viewer_app.SIM_PLUGINS]

# Panel colours, RGBA in [0, 1]
GREEN = (0.30, 0.80, 0.45, 1.0)
AMBER = (0.95, 0.70, 0.25, 1.0)
RED = (0.95, 0.35, 0.30, 1.0)
IDLE = (0.35, 0.37, 0.40, 1.0)


def _route(session):
    """How the phone is connected, e.g. "Same network · IPv6", or "" when it is not."""
    remote = session.remote
    if remote is not None and remote.connected and remote.peer:
        ip = remote.peer[0]
        family = "IPv6" if ":" in ip else "IPv4"
        return f"{'Same network' if remote.same_network(ip) else 'Internet'} · {family}"
    if session.connected_clients:
        return "Same network · Wi-Fi"
    return ""


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
    internet: str = ""  # "", "ready" or "connecting"
    route: str = ""  # how the phone is connected; see _route
    rate: float = 0.0  # pose messages per second from the phone
    scale: float = 1.0


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
        gripper_actuator: Actuator name, or list of names, driven by the app's Toggle. Defaults to
                          the actuator named "gripper", if the model has one.
        gripper_ctrl: (open, closed) controls for the gripper actuators. Defaults to the low and
                      high ends of each actuator's ctrlrange.
        app_clip: Encode the QR code as a TeleDex App Clip link.
        remote: Also accept a phone on another network (see ``Session(remote=True)``).
        signal_url: Rendezvous server for remote connections.
    """

    def __init__(self, body=None, port=8888, scale=1.0, frame=None, mode="absolute",
                 max_dpos=None, max_drot=None, position_limits=None, disable_rot=False,
                 gripper_actuator=None, gripper_ctrl=None, app_clip=False, remote=False,
                 signal_url=None):
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
        self.gripper_actuators = (None if gripper_actuator is None
                                  else [gripper_actuator] if isinstance(gripper_actuator, str)
                                  else list(gripper_actuator))
        self.gripper_ctrl = gripper_ctrl
        self.app_clip = app_clip
        self.remote = remote
        self.signal_url = signal_url

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
        self._rate = 0.0
        self._rate_count = 0
        self._rate_time = time.monotonic()

    # -- Lifecycle -----------------------------------------------------------------

    @messages.handler
    def on_sim_init(self, _: plugin.SimInitEvent) -> None:
        # The panel shows the QR code, so do not also print it in the terminal
        self.session = Session(port=self.port, show_qr=False, app_clip=self.app_clip, delta=True,
                               remote=self.remote, signal_url=self.signal_url)
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
        names = self.gripper_actuators
        if names is None:  # by convention, drive an actuator named "gripper" if there is one
            names = ["gripper"] if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper") >= 0 else []
        for name in names:
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
        connected = self.session.is_connected
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
        if now - self._rate_time >= 1.0:
            count = self.session.messages_received
            self._rate = (count - self._rate_count) / (now - self._rate_time)
            self._rate_count, self._rate_time = count, now

        self.handle.send_to_viewer(TeleDexSnapshot(
            connected=connected,
            address=(f"{self.session.ip_address}:{self.session.port}"
                     if not self.session.ip_address.startswith(("192.0.0.", "127.")) else ""),
            internet="" if self.session.remote is None else ("ready" if self.session.remote.ready else "connecting"),
            route=_route(self.session) if connected else "",
            rate=self._rate if connected else 0.0,
            scale=self.scale,
            qr_payload=(self.session.connection_link() if self.app_clip or self.session.remote is not None
                        else f"{self.session.ip_address}:{self.session.port}"),
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

    QR_SIZE_PX = 150  # target width of the QR code; longer links get smaller squares

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
        # Open next to Studio's Options sidebar on launch; users can move or dock it after
        imgui.SetNextWindowPos(imgui.Vec2(222, 36), imgui.Cond.Once)
        expanded, _ = imgui.Begin("TeleDex", None, imgui.WindowFlags.AlwaysAutoResize)
        if expanded:
            self._build_gui()
        imgui.End()

    def _build_gui(self):
        state = self._state
        if state is None:
            imgui.TextDisabled("Starting TeleDex…")
            return

        self._build_status(state)
        if state.connected and not state.error:
            self._build_pose(state)
        elif not state.error:
            self._build_pairing(state)
        self._build_controls(state)
        imgui.Spacing()
        imgui.TextDisabled(f"{state.body or '-'}  ·  {state.mode}  ·  {state.scale:g}× motion")

    def _build_status(self, state):
        if state.error:
            colour, title, detail = RED, state.error, ""
        elif state.connected:
            colour, title = (AMBER, "Paused") if state.paused else (GREEN, "Connected")
            detail = f"{state.route}  ·  {state.rate:.0f} Hz" if state.route else ""
        else:
            colour, title, detail = AMBER, "Waiting for phone", ""
        _dot(colour)
        imgui.SameLine()
        imgui.TextColored(imgui.Vec4(*colour), title)
        if detail:
            imgui.TextDisabled(detail)
        imgui.Spacing()

    def _build_pairing(self, state):
        if self._qr_texture:
            _centered(self._qr_size)
            imgui.Image(self._qr_texture, imgui.Vec2(self._qr_size, self._qr_size))
            caption = ("Scan with the iPhone Camera or the TeleDex app" if state.qr_payload.startswith("https://")
                       else "Scan with the TeleDex app")
            _centered(imgui.CalcTextSize(caption).x)
            imgui.TextDisabled(caption)
        elif not HAS_QRCODE:
            imgui.TextDisabled("pip install qrcode to show the QR code here")

        imgui.Spacing()
        if imgui.BeginTable("##teledex_routes", 2):
            imgui.TableSetupColumn("##what", imgui.TableColumnFlags.WidthFixed, 70)
            imgui.TableSetupColumn("##value", imgui.TableColumnFlags.WidthStretch)
            _row("Local", state.address or "not available on this network", dim=not state.address)
            if state.internet:
                _row("Internet", state.internet, colour=GREEN if state.internet == "ready" else AMBER)
            else:
                _row("Internet", "off (remote=True to enable)", dim=True)
            imgui.EndTable()

    def _build_pose(self, state):
        imgui.SeparatorText("Pose")
        if imgui.BeginTable("##teledex_pose", 4):
            imgui.TableSetupColumn("##what", imgui.TableColumnFlags.WidthFixed, 56)
            for axis in ("x", "y", "z"):
                imgui.TableSetupColumn(axis, imgui.TableColumnFlags.WidthFixed, 64)
            imgui.TableNextRow()
            for heading in ("", "x", "y", "z"):
                imgui.TableNextColumn()
                imgui.TextDisabled(heading)
            for label, values in (("Phone", state.phone_position), ("Target", state.target_position)):
                if values is None:
                    continue
                imgui.TableNextRow()
                imgui.TableNextColumn()
                imgui.TextUnformatted(label)
                for value in values:
                    imgui.TableNextColumn()
                    imgui.TextUnformatted(f"{value:+.3f}")
            imgui.EndTable()

        imgui.SeparatorText("Inputs")
        _chip("Button", state.button)
        imgui.SameLine(0, 18)
        _chip("Toggle", state.toggle)

    def _build_controls(self, state):
        imgui.SeparatorText("Controls")
        spacing = 8.0
        width = max(70.0, (imgui.GetContentRegionAvail().x - 2 * spacing) / 3)
        size = imgui.Vec2(width, 0)
        if imgui.Button("Resume" if state.paused else "Pause", size):
            self.viewer.send_to_sim(TeleDexCommandEvent("resume" if state.paused else "pause"))
        if imgui.IsItemHovered():
            imgui.SetTooltip("Hold the target still; resuming continues from where it is")
        imgui.SameLine(0, spacing)
        if imgui.Button("Recenter", size):
            self.viewer.send_to_sim(TeleDexCommandEvent("recenter"))
        if imgui.IsItemHovered():
            imgui.SetTooltip("Make the phone's current pose the reference for the target's current pose")
        imgui.SameLine(0, spacing)
        if imgui.Button("Refresh", size):
            self.viewer.send_to_sim(TeleDexCommandEvent("refresh"))
        if imgui.IsItemHovered():
            imgui.SetTooltip("Re-detect this computer's address, e.g. after switching Wi-Fi")

    def _upload_qr(self, payload):
        qr = qrcode.QRCode(border=2, error_correction=qrcode.constants.ERROR_CORRECT_L)
        qr.add_data(payload)
        qr.make(fit=True)
        modules = np.array(qr.get_matrix(), dtype=bool)
        module_px = max(2, self.QR_SIZE_PX // len(modules))  # whole pixels per square keep edges crisp
        pixels = np.where(modules, 0, 255).astype(np.uint8)
        pixels = np.kron(pixels, np.ones((module_px, module_px), dtype=np.uint8))
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



def _dot(colour, size=10.0):
    """A small coloured square used as a status light."""
    imgui.ColorButton("##dot", imgui.Vec4(*colour), _NO_TOOLTIP, imgui.Vec2(size, size))


def _chip(label, on):
    _dot(GREEN if on else IDLE)
    imgui.SameLine()
    if on:
        imgui.TextUnformatted(label)
    else:
        imgui.TextDisabled(label)


def _row(label, value, dim=False, colour=None):
    imgui.TableNextRow()
    imgui.TableNextColumn()
    imgui.TextDisabled(label)
    imgui.TableNextColumn()
    if colour is not None:
        imgui.TextColored(imgui.Vec4(*colour), value)
    elif dim:
        imgui.TextDisabled(value)
    else:
        imgui.TextUnformatted(value)


def _centered(width):
    """Move the cursor so an item of ``width`` pixels is centred in the window."""
    available = imgui.GetContentRegionAvail().x
    if available > width:
        imgui.SetCursorPosX(imgui.GetCursorPosX() + (available - width) / 2)


_NO_TOOLTIP = int(getattr(imgui.ColorEditFlags, "NoTooltip", 0)) | int(getattr(imgui.ColorEditFlags, "NoDragDrop", 0))
