from setuptools import setup, find_packages

package_name = "thinker_skill_sdk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="qingxiangliu",
    maintainer_email="1627049654@qq.com",
    description="Thinker-Skill 技能契约层",
    license="Apache-2.0",
    python_requires=">=3.10",
)
