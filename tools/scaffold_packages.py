#!/usr/bin/env python3
"""生成 ament_python 包骨架（脚手架，可重复运行）。

用法：python3 tools/scaffold_packages.py
已存在的文件不会覆盖。
"""
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent

# 相对目录 -> (ROS 包名, package.xml 的 exec_depend 列表)
PACKAGES = {
    "core/common": ("thinker_common", ["thinker_skill_sdk"]),
    "core/sensor": ("thinker_sensor", ["thinker_skill_sdk"]),
    "core/body": ("thinker_body", ["thinker_skill_sdk"]),
    "core/skill_core": ("thinker_skill_core", ["thinker_skill_sdk", "thinker_interfaces"]),
    "robots/tienkung_dex": ("tienkung_dex_robot", ["thinker_skill_sdk"]),
}

PACKAGE_XML = """<?xml version="1.0"?>
<package format="3">
  <name>{pkg}</name>
  <version>0.1.0</version>
  <description>Thinker-Skill {pkg}</description>
  <maintainer email="1627049654@qq.com">qingxiangliu</maintainer>
  <license>Apache-2.0</license>

{deps}  <exec_depend>python3</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""

SETUP_PY = '''from setuptools import setup, find_packages

package_name = "{pkg}"

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
    description="Thinker-Skill {pkg}",
    license="Apache-2.0",
    python_requires=">=3.10",
)
'''

PYPROJECT = """[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"
"""


def main():
    for rel, (pkg, deps) in PACKAGES.items():
        d = ROOT / rel
        (d / pkg).mkdir(parents=True, exist_ok=True)
        (d / "resource").mkdir(exist_ok=True)
        (d / "resource" / pkg).touch()
        init = d / pkg / "__init__.py"
        if not init.exists():
            init.write_text(f'"""{pkg}."""\n', encoding="utf-8")
        dep_lines = "".join(f"  <exec_depend>{x}</exec_depend>\n" for x in deps)
        (d / "package.xml").write_text(
            PACKAGE_XML.format(pkg=pkg, deps=dep_lines), encoding="utf-8"
        )
        (d / "setup.py").write_text(SETUP_PY.format(pkg=pkg), encoding="utf-8")
        (d / "pyproject.toml").write_text(PYPROJECT, encoding="utf-8")
        print("scaffolded:", rel)


if __name__ == "__main__":
    main()
