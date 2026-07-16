from setuptools import setup


setup(
    name="bin-ekf-replay-core",
    version="0.1.0",
    description="Reusable ArduPilot DataFlash EKF parser and state builder",
    packages=["bin_ekf_replay"],
    package_dir={"bin_ekf_replay": "."},
    install_requires=["numpy", "pandas", "scipy", "pymavlink"],
    python_requires=">=3.9",
)
