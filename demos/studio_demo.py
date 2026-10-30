"""Drive a paddle in MuJoCo Studio with your phone.

Needs a mujoco build with Studio (nightly: pip install --pre mujoco -f https://py.mujoco.org/).

    python demos/studio_demo.py          # browser viewer
    python demos/studio_demo.py --native # desktop window
    python demos/studio_demo.py --app-clip  # QR opens the TeleDex App Clip from the iPhone Camera
"""
import argparse
import os

from mujoco.studio import launch_passive, viewer_app, viewer_protocol

from teledex.studio import TeleDexPanel, TeleDexSim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", action="store_true", help="Open a desktop window instead of the browser viewer.")
    parser.add_argument("--mode", choices=["absolute", "delta"], default="absolute")
    parser.add_argument("--app-clip", action="store_true", help="Encode the QR code as a TeleDex App Clip link.")
    args = parser.parse_args()

    config = viewer_protocol.ViewerConfig(title="TeleDex", gfx="" if args.native else "web", open_browser=True)
    launch_passive.run(
        config,
        model_path=os.path.join(os.path.dirname(__file__), "studio_scene.xml"),
        plugins=[
            viewer_app.ViewerApp(),
            *viewer_app.SIM_PLUGINS,
            TeleDexSim(body="target", scale=1.5, mode=args.mode, max_dpos=0.02, app_clip=args.app_clip,
                       position_limits=[[-0.8, 0.8], [-0.8, 0.8], [0.03, 0.8]]),
            TeleDexPanel(),
        ],
    )


if __name__ == "__main__":
    main()
