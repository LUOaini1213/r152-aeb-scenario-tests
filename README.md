# AEB 法规场景测试：把已有 AEB 功能放进 UN R152 / Euro NCAP 场景

把我自己仓库 [can-bus-analyzer](https://github.com/LUOaini1213/can-bus-analyzer)（提交 `27da004`）里的 AEB 状态机**原样复用**，放进 esmini（OpenSCENARIO / OpenDRIVE）里跑 UN R152 和 Euro NCAP 2026 的追尾场景（CCRs / CCRm / CCRb）。依次做了判定、找失效点、改参数、再验证；深化部分做了湿滑路面、示意性 HARA、SOTIF 触发条件和需求追溯矩阵。

> 本项目在 AI 编码助手（Claude）辅助下完成。阈值和模型参数都是项目假设，结论只在本模型内成立；**不声称**任何功能符合 R152 或 Euro NCAP。完整说明见 [report.md](report.md)。

## 结果一览

基线条件：无附加时延，制动 80 ms 纯延迟 + τ 0.12 s。每个点跑 5 个雷达采样相位，取最差。

| 指标 | 原参数（上游默认） | 新参数 |
|---|---|---|
| R152 官方 6 点碰撞速度 | 全部在限值内 | 全部在限值内 |
| R152 5.2.1.1 预警提前量（≥ 0.8 s） | 1.00 s | 1.00 s |
| R152 5.2.1.2 制动需求 ≥ 5 m/s²（字面） | 19 点中 12 点不满足 | 全部满足 |
| R152 压力角（+0.2 s 时延，τ 0.4 s） | 6 个官方点中 4 个超限 | 0 个 |
| Euro NCAP 简化估计（只含 50 % 撞击位置，满分 5.2） | 4.17 | 5.20 |
| 规程结束（V_VUT = V_target）之后的二次接触 | 0 | 245 次中 18 次，最高 8.0 km/h |
| 1 帧幻影目标（TTC 1.8 s） | 只报警 | 6 m/s² 误制动，速度损失 10.8 km/h |
| 湿路 μ = 0.5 的简化估计（模型内） | 1.98 | 4.17（摩擦自适应参数 5.07） |
| 雨天量程 40 m 时的 R152 预警提前量 | S60 恰好 0.80 s | S60 / S55 不足（0.4 / 0.6 s） |

**新参数**只改了 `AebConfig` 的 4 个值：TTC 预警 / 部分 / 全力制动 2.6 / 1.6 / 0.9 s → 3.0 / 2.0 / 1.3 s，部分制动 4 → 6 m/s²。选择规则在跑之前写进了 `scripts/param_search.py`。

**更保守的备选**：只把部分制动 4 → 5 m/s²，就能补上 R152 的字面缺口和压力角超限，且没有二次接触（NCAP 估计 4.64）。

## 信号级加深（`signal_level/`，报告第 10 节）

把同一套 R152 闭环下沉到 CAN 帧层：自写 DBC 解析与编解码（Intel / Motorola）、Profile-1-like E2E（CRC-8 0x1D + 4 bit 计数器，对照 AUTOSAR R24-11 官方示例做单元测试）、逐报文超时监控和降级，以及可注入丢帧、突发、时延、比特翻转、bus-off 的虚拟总线。

| 问题 | 结果 |
|---|---|
| 无故障总线能否复现直连结果 | 透明模式 490/490 完全一致；DBC 量化后 R152 判定 0 翻转，95 处差异都能用量化解释 |
| 制动起点附近 100 ms 全总线突发 | 新参数 330 个组合中 30 个超限（最差 38.0 km/h，限值 35）；只把 `fault_recover_s` 从 1.0 s 改为 0.2 s 后为 0 |
| 单帧单比特损坏 | 无 E2E：313 次误制动 / 提前制动；有 E2E：0 次，但被拒收的帧等于丢了一帧 |
| 附加时延 10 / 20 / 50 ms | 制动起点推迟 20 / 40 / 100 ms，两套参数都不超限 |

## 怎么做的

- **要求只来自官方文本**：
  - UN R152：EUR-Lex 合并文本 OJ L 2024/2497，到 02 系列补充件 3；
  - Euro NCAP：Crash Avoidance – Frontal Collisions Protocol v1.2（2026-07）。

  每个数字都在 `aebtest/regulations.py` 注明条款。
- **场景与闭环**：
  - 3 个参数化 `.xosc`，加 6 个 OpenSCENARIO ParameterValueDistribution 网格（51 个用例）；
  - esmini v3.10.2 通过 `esminiLib.dll`（ctypes）无界面运行，固定步长 2 ms；
  - 自车交给 ExternalController，由 Python 中的雷达模型 → 上游状态机 → 一阶制动执行器 → 纵向模型闭环控制。
- **可信度**：
  - 纯 Python 后端与 esmini 在 490 次运行中结果一致；
  - 1 ms 与 2 ms 步长相比最差碰撞速度最多差 0.6 km/h；
  - CCRs 70 / 80 km/h 的碰撞速度与上游仓库（CAN 总线仿真）一致；
  - 复用代码的 SHA-256 / git blob 与上游一致；
  - 14 项单元测试通过（没有 esmini 时 13 项通过、1 项跳过）；信号级另有 39 项。

## 复现

```bash
# 有 esmini 时把 --backend python 换成 --esmini-lib <esminiLib.dll / libesminiLib.so 的路径>
python -I scripts/check_vendor.py; python -I scripts/make_scenarios.py
python -I scripts/param_search.py --backend python --workers 8
python -I scripts/run_all.py --backend python --workers 8
python -I scripts/run_deepening.py --backend python --workers 8
python -I scripts/make_traceability.py; python -I scripts/make_tables_deepening.py
python -I scripts/make_plots.py
python -I -m pytest -q tests                                              # 设置 ESMINI_LIB 时 14 项全跑，否则 1 项跳过
python -B -I -m pytest -q signal_level/tests -c signal_level/pytest.ini   # 信号级 39 项
python -B -I signal_level/run_signal_level.py --workers 4                 # 信号级实验，约 22 分钟
```

- 没有 esmini 时加 `--backend python`。
- esmini 打不开含中文的绝对路径，脚本会自动以 `scenarios/` 为工作目录、用相对路径调用它。
- 依赖：Python 3.13、matplotlib（画图）、pytest（测试），见 `requirements.txt`。esmini v3.10.2 官方二进制（MPL-2.0，[esmini/esmini](https://github.com/esmini/esmini) releases）可选，不包含在本仓库里。

## 文件

```
scenarios/        straight_road.xodr、CCRs/CCRm/CCRb.xosc、6 个 *_grid.xosc
vendor/           原样复用的上游 aeb_function.py 等（含来源说明和许可证）
aebtest/          加载器、法规参数、模型、esmini 桥接、闭环、判定、需求注册表
scripts/          场景生成、参数搜索、实验、追溯矩阵、图表、报告拼装
results/          CSV / 表格 / 时间序列 / esmini 日志 / plots/（图 1–7）
traceability.csv  条款 → 需求 → 信号 → OpenSCENARIO 用例 → 结果（758 行，前后向完整）
signal_level/     信号级加深：DBC、E2E、超时降级、虚拟总线、39 项测试、results/（表格由 CSV 生成）
report.md         完整报告（表格摘自 results/tables*.md 和 signal_level/results/tables_signal_level.md）
```

## 局限

- 一维纵向模型，只覆盖 Euro NCAP 网格的 50 % 撞击位置；NCAP 数字只是简化估计，不是得分。
- 感知、制动、路面附着都是理想化的玩具模型。
- 没有驾驶员和后车模型。主结果是直连的；CAN 帧层只在 `signal_level/` 里覆盖，而且总线是抽象模型（没有仲裁、位定时、错误帧）。
- 误作用率（误报 / 误制动）无法在仿真里评估，只能用幻影目标注入做定性对比。

## 未核实

- R152 在 02 系列补充件 3 之后的修订。
- 技术服务机构对 5.2.1.2 字面要求的解释。
- Euro NCAP 图 5-1 除 60 km/h 一行外的色带边界、标准 / 扩展范围划分（这两项都是从官方矢量图读出的）。
- ISO 26262 / ISO 21448 原文（没有查阅；HARA 只是示意性练习）。
- 湿路 μ 取值。
- ASEAN NCAP 没有纳入。

- 信号级：R152 5.1.4.1 失效警告的原文措辞与时间要求、ISO 26262 FTTI。

完整清单见 report.md 第 14 节和第 10.5 节。
