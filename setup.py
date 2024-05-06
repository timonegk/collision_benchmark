import glob

from setuptools import setup

package_name = "benchmark"


setup(
    name=package_name,
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/scenarios", glob.glob("config/*")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch")),
    ],
    scripts=[
        "scripts/benchmark.py",
        "scripts/run_optuna.py",
    ],
    install_requires=[
        "launch",
        "setuptools",
    ],
    zip_safe=True,
    keywords=["ROS"],
    license="MIT",
)