import glob

from setuptools import setup

package_name = "collision_benchmark"


setup(
    name=package_name,
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/scenarios", glob.glob("config/*")),
    ],
    scripts=[
        "scripts/run_benchmark.py",
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    keywords=["ROS"],
    license="MIT",
)