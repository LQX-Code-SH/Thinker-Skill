from setuptools import setup, find_packages

package_name = "thinker_skill_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # ros2 run 从 <prefix>/lib/<pkg>/ 查找可执行文件
        ("lib/" + package_name, ["scripts/thinker_host"]),
    ],
    entry_points={
        "console_scripts": [
            "thinker_host = thinker_skill_core.host_main:main",
        ],
    },
    install_requires=[],
    zip_safe=True,
    maintainer="qingxiangliu",
    maintainer_email="1627049654@qq.com",
    description="Thinker-Skill thinker_skill_core",
    license="Apache-2.0",
    python_requires=">=3.10",
)
