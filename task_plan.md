# I6X-I6 第二轮融合计划

目标：在本模块仓库中融合 SBUS 与 iBus 接收实现，通过 `I6X::Param::protocol` 选择协议，保持 LibXR Module 规范，并用 README 记录接收机支持矩阵。

范围：`Modules/I6X/I6X.hpp`、`README.md`、必要的模块元数据与验证记录；不修改外部 BSP、锁文件或生成代码。

## 阶段

- [complete] 1. 读取规范、参考实现与仓库状态
- [complete] 2. 设计并实现统一协议接口
- [complete] 3. 按 Module 规范编写 README 与元数据
- [complete] 4. 运行格式、解析回归、编译和 hook 检查
- [complete] 5. 复核差异并交付

## 已知约束

- SBUS：25 字节、100000 8E2、16 个 11 位通道。
- iBus：32 字节、115200 8N1、14 个 16 位通道和减法校验和。
- `Data` 需要容纳两种协议；iBus 的旧 `ch/channel/channels` 别名保留。
- 协议选择只新增一个公开配置参数，默认 iBus 以兼容 h7 实现。

## 错误记录

| 错误 | 尝试 | 处理 |
| --- | ---: | --- |
| `xrobot check-module --offline` 找不到 BSP | 1 | 用 `xrobot module show Modules/I6X/I6X.hpp` 完成本地 MANIFEST/构造器 hook 检查；记录独立 Module 缺少 `Modules/modules.yaml` 的环境限制 |
| 旧 MinGW g++ 8.1 缺少 `<concepts>` | 1 | 使用带临时 LibXR API stub 的主机解析回归和 ARM GCC 14.3 语法探针；临时文件已清理 |

