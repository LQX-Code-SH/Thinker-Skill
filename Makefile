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

.PHONY: help build build-skill test clean

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
