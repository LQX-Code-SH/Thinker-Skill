from glob import glob

from setuptools import setup, find_packages

package_name = "thinker_robot_tienkung_dex"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "capabilities.yaml"]),
        ("share/" + package_name + "/resources", ["resources/joints.yaml"]),
        ("share/" + package_name + "/resources/urdf", glob("resources/urdf/*.urdf")),
        ("share/" + package_name + "/resources/calib", glob("resources/calib/*.yaml")),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="qingxiangliu",
    maintainer_email="1627049654@qq.com",
    description="Thinker-Skill thinker_robot_tienkung_dex",
    license="Apache-2.0",
    python_requires=">=3.10",
)
