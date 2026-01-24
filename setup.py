from setuptools import setup, find_packages

setup(
    name="teledex",
    version="0.0.5",
    description="Phone-based Dexterous Robots teleoperation library",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "scipy",
        "websockets",
        "qrcode",
    ],
)
