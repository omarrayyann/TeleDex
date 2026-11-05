"""Drive the gripper in MuJoCo Studio with your phone.

Needs a mujoco build with Studio (nightly: pip install --pre mujoco -f https://py.mujoco.org/).

    python demos/studio_demo.py                      # scene0: put the marker in the mug
    python demos/studio_demo.py --scene 1            # scene1: the lazy Susan
    python demos/studio_demo.py --app-clip --remote  # QR opens the App Clip; phone can be on another network
    python demos/studio_demo.py --native             # desktop window instead of the browser viewer
"""
import argparse
import os

from mujoco.studio import launch_passive, viewer_protocol

from teledex.studio import TeleDexPanel, TeleDexSim, studio_plugins


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=int, default=0, help="Which demos/scene<N>.xml to open.")
    parser.add_argument("--native", action="store_true", help="Open a desktop window instead of the browser viewer.")
    parser.add_argument("--mode", choices=["absolute", "delta"], default="absolute")
    parser.add_argument("--scale", type=float, default=1.25, help="Gripper motion per metre of phone motion.")
    parser.add_argument("--app-clip", action="store_true", help="Encode the QR code as a TeleDex App Clip link.")
    parser.add_argument("--remote", action="store_true", help="Also accept a phone on another network.")
    args = parser.parse_args()

    config = viewer_protocol.ViewerConfig(title="TeleDex", gfx="" if args.native else "web", open_browser=True)
    launch_passive.run(
        config,
        model_path=os.path.join(os.path.dirname(__file__), f"scene{args.scene}.xml"),
        plugins=[
            *studio_plugins(),
            TeleDexSim(scale=args.scale, mode=args.mode, max_dpos=0.02, app_clip=args.app_clip, remote=args.remote),
            TeleDexPanel(),
        ],
    )


if __name__ == "__main__":
    main()
