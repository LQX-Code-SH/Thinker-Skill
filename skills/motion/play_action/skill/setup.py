from setuptools import setup, find_packages

package_name = "play_action"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "skill.yaml"]),
    ],
    install_requires=["pyyaml"],
    zip_safe=True,
    maintainer="qingxiangliu",
    maintainer_email="1627049654@qq.com",
    description="固定动作播放：按动作文件回放上半身动作",
    license="Apache-2.0",
    python_requires=">=3.10",
)
