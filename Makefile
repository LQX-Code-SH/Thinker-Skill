# Thinker-Skill 构建入口
#   make build                     全量构建（core / robots / skills）
#   make build-skill SKILL=<name>  只构建某个技能
#   make test                      单测（无需 ROS）
#   make clean

COLCON       ?= colcon
BUILD_BASE   := .colcon/build
INSTALL_BASE := .colcon/install
LOG_BASE     := .colcon/log
SKILL        ?=
SIM_DIR      ?= sim/robot_sim
SIM_SKILL    ?= look_down_table
SIM_PARAMS   ?= {}
SIM_POSE_ARGS ?= --count 1
# 场景默认出生点（实测 /manip/base_pose）：(4.729, -2.600)，yaw≈0
SIM_INIT_X   ?= 4.1
SIM_INIT_Y   ?= -2.60
SIM_INIT_Z   ?=
SIM_INIT_YAW ?= 180
# FSM 模式等 SWR 期间的吊带骨盆高度（默认 1.03 太高，用 1.00）
SIM_HANG_Z   ?= 1.00

.PHONY: help build build-skill test clean sim-init sim-up sim-down sim-build \
        sim-probe sim-pose sim-bridge sim-bridge-bg sim-fsm sim-fsm-bg \
        sim-host sim-host-bg sim-logs sim-selftest sim-call sim-shell sim-image-view

help:
	@echo "make build                     全量构建"
	@echo "make build-skill SKILL=<name>  只构建某个技能"
	@echo "make test                      单测（无需 ROS）"

build:
	$(COLCON) --log-base $(LOG_BASE) build --base-paths core robots skills \
		--build-base $(BUILD_BASE) \
		--install-base $(INSTALL_BASE)

build-skill:
	@test -n "$(SKILL)" || { echo "用法：make build-skill SKILL=<name>"; exit 1; }
	$(COLCON) --log-base $(LOG_BASE) build --packages-select $(SKILL) \
		--build-base $(BUILD_BASE) \
		--install-base $(INSTALL_BASE)

test:
	pytest -q tests

clean:
	rm -rf .colcon

# 仿真环境：只初始化子模块 + 选择性拉取 manip_description 资产，不参与默认构建
sim-init:
	GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --depth 1
	git -C $(SIM_DIR) lfs pull --include "src/manip_description/assets/**"

sim-up:
	tools/sim/container.sh up

sim-down:
	tools/sim/container.sh down

sim-build:
	tools/sim/container.sh build

# 前台：带 MuJoCo 查看器窗口（需要 DISPLAY）
sim-bridge:
	SIM_INIT_X=$(SIM_INIT_X) SIM_INIT_Y=$(SIM_INIT_Y) SIM_INIT_Z=$(SIM_INIT_Z) SIM_INIT_YAW=$(SIM_INIT_YAW) tools/sim/container.sh bridge

sim-host:
	tools/sim/container.sh host

# 打印基座位姿（x/y/z + yaw），等价于手算四元数
sim-pose:
	tools/sim/container.sh pose $(SIM_POSE_ARGS)

# 后台：无窗口，日志 /tmp/sim_bridge.log
sim-bridge-bg:
	SIM_INIT_X=$(SIM_INIT_X) SIM_INIT_Y=$(SIM_INIT_Y) SIM_INIT_Z=$(SIM_INIT_Z) SIM_INIT_YAW=$(SIM_INIT_YAW) tools/sim/container.sh bridge-bg

# 带 xmigcs FSM + 遥控器的半身模式（前台，带窗口）
sim-fsm:
	SIM_HANG_Z=$(SIM_HANG_Z) SIM_INIT_X=$(SIM_INIT_X) SIM_INIT_Y=$(SIM_INIT_Y) \
	SIM_INIT_Z=$(SIM_INIT_Z) SIM_INIT_YAW=$(SIM_INIT_YAW) tools/sim/container.sh fsm

# 同上，后台无窗口，日志 /tmp/sim_fsm.log
sim-fsm-bg:
	SIM_HANG_Z=$(SIM_HANG_Z) SIM_INIT_X=$(SIM_INIT_X) SIM_INIT_Y=$(SIM_INIT_Y) \
	SIM_INIT_Z=$(SIM_INIT_Z) SIM_INIT_YAW=$(SIM_INIT_YAW) tools/sim/container.sh fsm-bg

sim-host-bg:
	tools/sim/container.sh host-bg

# 真机接口对齐检查：/robot_state、/{arm,head,waist}/cmd、相机与手话题
sim-probe:
	tools/sim/container.sh probe

# 网页相机视图：容器内取流，宿主机浏览器打开 http://localhost:8090
sim-image-view:
	tools/sim/container.sh image-view

sim-logs:
	tools/sim/container.sh logs

# 进程内装配 host + 适配器，直接调技能（不经过 ROS 通道）
sim-selftest:
	tools/sim/container.sh selftest --skills $(SIM_SKILL)

# 走 ROS Service，与外部调用方完全一致的链路
sim-call:
	tools/sim/container.sh call $(SIM_SKILL) --params '$(SIM_PARAMS)'

sim-shell:
	tools/sim/container.sh shell