# 修改要求

## 规划阶段时不要全面列举文件，列表说明修改内容和原因
## 修改内容大白话解释，并列表说明修改
## 完成后自行colcon build --packages-select <包名>

# 每次让你分析时，记得全面查看相关日志，不得凭单日志下定论

## 日志存放位置
- ROS2 运行时日志：`~/.ros/log/`（即 `/root/.ros/log/`），含两种形式：
  - 顶层扁平 `.log` 文件（如 `origincar_base_node_*_*.log`、`component_container_isolated_*.log`）
  - 时间戳子目录（如 `2026-08-12-15-34-56-396239-ubuntu-xxxx/`）
  - 排查问题优先看顶层修改时间最新的 `.log` 文件
- colcon 构建日志：`/root/ros2_ws/log/`（`build_*` 目录，仅与编译相关，与运行问题无关）

## 日志查看方法
- 最新日志：`ls -lt ~/.ros/log/ | head`
- 查看内容：`tail -f ~/.ros/log/<最新日志文件名>` 或 `cat`

## 内容建议
- 当你给出方案的时候，你应该联网查找相关，是否已经有比较成熟的方案
