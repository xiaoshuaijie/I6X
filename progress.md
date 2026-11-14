# 进度

- 2026-11-14：完成仓库状态、xrobot-skills、SBUS 与 iBus 参考实现检查。
- 2026-11-14：建立本轮任务计划和发现记录。
- 2026-11-14：新增统一 `Modules/I6X/I6X.hpp`，通过 `Param::protocol` 切换 SBUS/iBus，并保留两套解析字段。
- 2026-11-14：新增双语 Module README，记录协议参数、Topic、配置示例和 I6/IA6/IA6B/IA10B 支持矩阵。
- 2026-11-14：`xrobot module show`、clang-format、git diff 检查和私密信息 hook 通过；SBUS/iBus 确定性解析回归返回 0。
- 2026-11-14：`xrobot check-module --offline` 因当前目录不是 BSP 而无法运行，已在计划错误记录中注明。
- 2026-11-14：按讨论将构造器默认校准改为 `Calibration{}`；保留 `DefaultCalibration()` 作为兼容的 constexpr 公共工具函数，`channels[]` 继续作为 union 别名。
