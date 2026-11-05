from setuptools import setup, find_packages

setup(
    name="teledex",
    version="0.0.9",
    description="Phone-based Dexterous Robots teleoperation library",
    packages=find_packages(),
    extras_require={"web": ["aiortc"]},
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "scipy",
        "websockets",
        "qrcode",
    ],
)
