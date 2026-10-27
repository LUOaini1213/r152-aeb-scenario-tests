# P5 信号级深化：DBC / E2E / 超时降级 / 总线故障下的 R152 回放

> **个人项目，使用 AI 工具辅助完成**（Claude）。所有报文 ID、周期、信号分辨率、超时与恢复参数都是**设计假设**，不来自任何主机厂、供应商或标准；所有结果都是玩具模型的仿真数据（**SYNTHETIC**）。本目录**不声称**符合 UN R152 或 AUTOSAR。

## 1 做了什么、为什么

P5 把上游 AEB 状态机放进 UN R152 车对车场景时，雷达距离、相对速度、车速都是**浮点数直连**进状态机的。真车上它们是 CAN 帧：要按 DBC 量化，会丢帧、延迟、被破坏，接收端还要做 E2E 校验和超时监控，并决定什么时候降级。本目录把 P5 的同一套 R152 闭环**下沉到信号级**重跑，回答三个问题：

1. 无故障的总线能不能复现 P5 的结果？不同之处能不能全部用信号量化解释？
2. 丢帧、关键时刻的突发丢帧、附加时延会不会让 R152 判定翻转？差多少？
3. E2E（CRC + 计数器）到底挡住了什么？没有它，一帧坏数据会造成什么后果？

闭环结构（每 2 ms 一步，全部在一个 Python 进程内）：

```
P5 RadarModel ─► RADAR 节点：DBC 编码 + E2E ─┐                         ┌─► BRAKE ECU：E2E + 超时 ─► P5 制动执行器 ─► P5 纵向模型
P5 车速        ─► ESC 节点：DBC 编码 + E2E ──┼─► 虚拟总线（时延/抖动/ ──┤
                                             │   丢帧/突发/比特翻转/    └─► HMI：预警显示 + 失效报警灯
                 AEB ECU：E2E + 可信性 + 超时 ◄┘   bus-off/发送方卡死）
                 → 上游 analyzer.ttc() → 上游 StateMachine（原样复用）→ AEB_CMD 帧 ─► 总线
```

被测状态机就是 P5 用的 `vendor/canbus_analyzer_aeb/aeb_function.py`（与 LUOaini1213/can-bus-analyzer @27da004 字节一致），通过 P5 的 `aebtest/aeb_loader.py` 导入，**一行未改**（测试里核对 SHA-256）。上游自带的 CAN 外壳（`AebEcu` / `Supervisor`）依赖 cantools 和 opendbc 的 Toyota DBC，本机不能装，所以由本目录的 `ecus.py` 替代，只替代外壳，状态机本体不动。

## 2 设计

### 2.1 DBC（`aeb_demo.dbc`，标准 Vector DBC 语法）

| 报文 | ID | 发送方 → 接收方 | 周期（设计假设） | 字节序 | 信号（分辨率、范围均为设计假设） |
|---|---|---|---|---|---|
| RADAR_TGT | 0x120 | RADAR → AEB_ECU | 50 ms（= P5 雷达更新周期） | Motorola | TgtDist 13 bit，0.05 m，0–409.55 m；TgtRelSpd 12 bit 有符号，0.05 m/s，−70–70 m/s；TgtValid 1 bit；TgtQuality 2 bit；计数器 4 bit；CRC 8 bit |
| EGO_SPEED | 0x0B0 | ESC → AEB_ECU | 20 ms（= P5 车速采样周期） | Intel | VehSpd 16 bit，0.01 km/h，0–300 km/h；VehSpdValid；计数器；CRC |
| AEB_CMD | 0x0A0 | AEB_ECU → BRAKE_ECU, HMI | 10 ms（= AEB 任务周期） | Intel | AEB_AccelReq 12 bit 有符号，0.01 m/s²（负 = 制动），−16–16；AEB_State 3 bit；AEB_BrkReqActive；AEB_WarnLvl 2 bit；AEB_Avail 2 bit（0 可用 / 1 降级 / 2 不可用）；AEB_FaultReason 4 bit；计数器；CRC |

- 周期用 `BA_ "GenMsgCycleTime"` 声明，Data ID 用自定义属性 `BA_ "E2E_DataID"` 声明；每个周期和分辨率的 `CM_` 注释里都写了 "DESIGN ASSUMPTION"（有测试检查）。
- 三个周期刻意取成 P5 模型的周期，这样无故障总线才能和 P5 逐步对齐。上游仓库的 PRE_COLLISION 是 30 ms，这里用 10 ms 是为了这个对齐，是设计选择。
- 解析器和编解码器（`dbc.py`）是自己写的，没用 cantools / python-can：支持 VERSION、BU_、BO_、SG_、CM_（含多行）、BA_DEF_、BA_DEF_DEF_、BA_、VAL_，扩展帧标志；Intel/Motorola、有符号、factor/offset；编码时四舍五入（远离零）、超出 [min|max] 或原始值范围时饱和（或严格模式下报错）。多路复用信号只解析、不支持编解码。

### 2.2 E2E：只在核实过的地方使用 AUTOSAR Profile 1 语义

读过的官方文件（autosar.org 的 PDF，本地抽取文本后逐条核对）：

- **AUTOSAR FO R24-11《E2E Protocol Specification》，文档号 849**：
  - 6.5 节：Profile 1 机制表（PRS_E2E_00865，计数器 4 bit 取 0–14，Data ID 16 bit 参与 CRC）。
  - 6.5.1.1 节：计数器从 0 开始，每次发送 +1，14 之后回到 0，跳过 0xF（PRS_E2E_00075）。
  - 6.5.1.2 节：Data ID 的 BOTH / ALT / LOW / NIBBLE 四种方式（PRS_E2E_00163）。
  - 6.5.1.3 节：多项式 0x1D，起始值和异或值都是 0x00（PRS_E2E_00070）；先算 Data ID 字节，再算除 CRC 字节外的所有数据字节（PRS_E2E_00082）。
  - 6.5.3.1 节：E2E_P01Check 流程（图 6.58、6.60–6.63）。
  - 6.5.4 节：状态枚举 OK / NONEWDATA / WRONGCRC / SYNC / INITIAL / REPEATED / OKSOMELOST / WRONGSEQUENCE（PRS_E2E_00868）。
  - 6.5.5 节：协议示例表 6.4–6.13。
  - 6.1 节：变体 1A 要求 CRC 从 bit 0 开始、计数器从 bit 8 开始。
  - 9.8.2 节：PRS_E2E_UC_00072，双 Data ID 配置下，ASIL B 及以上数据的 Data ID 算出的 CRC 不应与系统中其他数据的相同（本项目 3 个 Data ID 的 CRC 互不相同，有测试）。
- **AUTOSAR CP R24-11《Specification of CRC Routines》，文档号 16**，7.2.1.1 节：SWS_Crc_00073 规定 CRC8 SAE-J1850 为多项式 1Dh、初值 FFh、不反射、异或 FFh、校验值 4Bh；SWS_Crc_00074 给出校验表。

本项目的用法：

- **按 Profile 1 实现、并用官方示例验证的部分**：
  - CRC 计算：复现 PRS 表 6.6–6.13 的全部 8 组示例；通用 CRC-8 复现 SWS_Crc_00074 的全部 7 组。
  - 计数器 0–14。
  - E2E_P01Check 的状态判定。
  - 布局取变体 1A 的位置：CRC 在字节 0，计数器在字节 1 低半字节，Data ID 方式 BOTH。
- **本项目自己的设计（不是 Profile 1 规定）**：
  - 调用方式：每收到一帧调用一次 check（NewDataAvailable = TRUE），所以不用 NONEWDATA 路径和 Profile 1 基于计数器的超时，改用基于时间的截止监控。
  - 配置值：MaxDeltaCounterInit = 2，SyncCounterInit = 1，MaxNoNewOrRepeatedData = 15。
  - 上面的超时、降级、恢复逻辑。
- 这种逐帧调用方式下，SyncCounterInit 必须 ≥ 1：取 0 时，计数器跳变后接收端要等 4 bit 计数器绕一圈才能重新同步，测试里实测要等 ≥ 10 帧。
- **不声称符合 AUTOSAR**：没有 AUTOSAR 协议栈，没有跑一致性测试。因此整个方案（核实过的 Profile 1 CRC / 计数器 / check 逻辑 + 本项目的调用方式与时间监控）只称为 **"Profile-1-like"**。另外，PRS R24-11 第 6.5 节写明 "Profile 1 is a Legacy Profile and is only maintained for compatibility reasons. New Projects shall use Profile 11"；第 6.12 节说 Profile 11 与 Profile 1 总线兼容、接收端行为不同。迁移到 Profile 11 是下一步。

### 2.3 接收端监控与降级（`supervision.py`、`ecus.py`）

每个接收报文一个监控器：

- **E2E 故障**：连续 2 帧无效，与上游 `MonitorSpec.e2e_max_consecutive` 的默认值相同。
- **超时**：超过 3 个周期没有有效帧，即雷达 150 ms、车速 60 ms、AEB_CMD 30 ms。
- **恢复**：连续 3 帧有效才恢复。
- **超时之前**：用最后一个有效值，称为"降级"（AEB_Avail = 1，不报警）。

AEB ECU 把监控结果映射到上游状态机的输入：

- `input_fault` = 任一输入故障 → 上游规则 R04：任何开启状态 → FAULT，制动请求为 0。
- `ok_since` = 输入恢复的时刻 → 上游规则 R05：输入持续正常 `fault_recover_s` = 1.0 s（上游默认值）后，FAULT → STANDBY。
- FAULT 时 AEB_Avail = 2，HMI 点失效报警灯。HMI 自己收不到 AEB_CMD（超时）时，也点报警灯。

R152 对失效报警的要求见 5.1.4.1（条款号沿用 P5 报告第 2.2 节）。本次会话 EUR-Lex / UNECE 打不开，**原文措辞和其中的时间要求【未核实】**，所以下表只给设计值和实测值，不和法规时间比较。功能安全的容错时间（FTTI）没有来源，**没有给出**。

| 故障 | 检测（在哪、靠什么） | 反应 | 恢复条件 | 时间预算：设计值 / 实测（表 S8） |
|---|---|---|---|---|
| RADAR_TGT 丢 1–2 帧 | AEB ECU：计数器差 2–3 → OKSOMELOST；数据龄期 > 1.5 周期 | 用最后有效值，AEB_Avail = 降级，不报警 | 下一有效帧 | 数据最多旧 150 ms / 实测无 FAULT（F1） |
| RADAR_TGT 超时（> 3 × 50 ms） | AEB ECU 截止时间监控 | FAULT：请求 0，失效报警灯 | 3 帧连续有效（计数器跳变时先 WRONGSEQUENCE + 1 帧 SYNC），再加上游 1.0 s | 检测 ≤ 最后有效帧 + 150 ms + 一个任务 10 ms；恢复 ≈ 5 帧 + 1.0 s / F2：丢 500 ms，最后有效帧后 160 ms（故障开始后 110 ms）检测，报警灯同时点亮，故障结束 1.20 s 后熄灭 |
| EGO_SPEED 超时（> 3 × 20 ms） | 同上 | 同上 | 3 帧 + 1.0 s | 检测 ≤ 60 + 10 ms / F3：最后有效帧后 70 ms 检测，结束 1.08 s 后熄灭 |
| 单帧 CRC 错 | E2E_P01Check → WRONGCRC | 丢弃该帧，保持上一值 | 下一有效帧 | 无 FAULT / F4：WRONGCRC 1 帧，无 FAULT，结果与无故障相同 |
| 连续 2 帧无效（CRC 错 / 重复 / 跳变 / 越界） | 监控器连续无效计数 | FAULT + 报警灯 | 3 帧 + 1.0 s | 第 2 个坏帧时检测 / F5：第 2 个坏帧（50 ms 后）FAULT，结束 1.10 s 后熄灭 |
| 发送方卡死（同一帧反复发送） | REPEATED（存活计数器） | 第 2 个重复帧时 FAULT | 同上 | 雷达 ≤ 100 ms / F6：REPEATED 第 2 帧（50 ms 后）FAULT；无 E2E 时 AEB 会一直用冻结的距离（单元测试） |
| 计数器跳变 > MaxDelta（丢 ≥ 3 帧） | WRONGSEQUENCE（顺序计数器） | 丢弃并重新同步 | 1 帧 SYNC 后 OK | — |
| AEB_CMD 在制动 ECU 处超时（> 3 × 10 ms） | 制动 ECU 截止时间监控（2 ms 任务） | 不再执行 AEB 请求；HMI 同样超时 → 报警灯 | 3 帧连续有效 | 检测 ≤ 32 ms / F7：AEB ECU bus-off 300 ms，最后一帧后 32 ms 停止执行，制动请求丢失 0.298 s |
| 节点 bus-off | 各接收方的超时 | 同对应报文超时 | 节点恢复 + 3 帧（+1.0 s） | 视报文 |
| 信号越界（CRC 正确但超出 DBC [min\|max]） | AEB ECU 可信性检查 | 视为无效帧 | 下一有效帧 | — |

E2E 只保护**传输**，不保护**信号源**：传感器自己算错、但 CRC 正确的数据，只能靠可信性检查和传感器诊断发现。

### 2.4 虚拟总线（`vbus.py`）

- 可配置项：时延、抖动（同一 ID 不超车）、独立同分布丢帧、突发丢帧窗口、随机或定点比特翻转、节点 bus-off、发送方卡死（重复旧帧）。
- 所有随机量都由种子决定。同一种子下，丢帧率越高，丢掉的帧集合越大（低丢帧率丢的帧在高丢帧率下也丢）。
- 不是 CAN 物理层模型：没有仲裁，没有错误帧和 CAN 自身的 CRC。"比特翻转"代表 CAN CRC 看不到的错误，例如网关或 ECU 内存里的错误。
- 交付时刻量化到 2 ms 仿真步长。

## 3 怎么运行

```bash
# 在 P5_AEB法规场景测试 目录下（Windows / Linux 相同；-B 不在 P5 目录里写 __pycache__）
python -B -I -m pytest -q signal_level/tests -c signal_level/pytest.ini   # 单元 + 闭环测试
python -B -I signal_level/run_signal_level.py --workers 4                 # 全部实验，本机约 22 分钟（4 进程，约 3.1 万次闭环运行）
python -B -I signal_level/run_signal_level.py --quick --workers 3         # 冒烟测试，约 5 分钟
python -B -I signal_level/make_tables.py                                  # results/tables_signal_level.md
python -B -I signal_level/make_figures.py                                 # results/fig_sl*.png
```

- 依赖：Python 3.13、matplotlib、pytest（与 P5 相同），**不需要** cantools / python-can / esmini。
- 默认用 P5 的纯 Python 后端。P5 已验证它与 esmini 在 490 次运行中一致。

## 4 结果（全部 SYNTHETIC，完整表格见 `results/tables_signal_level.md`）

**无故障总线（E1，49 点 × 5 相位 × 2 套参数 = 490 次）**

- **透明模式**（帧里带原始浮点）：490/490 次与 P5 直连闭环逐项相同，包括碰撞、碰撞速度、预警 / 制动时刻、最小车距、状态序列。说明总线、E2E 和监控这几层本身不引入误差。
- **DBC 量化模式**：395/490 次完全相同。
  - R152 的 190 次运行碰撞速度全部不变，判定翻转 0 次；
  - Euro NCAP 的 300 次碰撞有无翻转 0 次，最大碰撞速度差 2.95 km/h（CCRb-60 原参数相位 0，制动晚一帧）；按 P5 的"5 相位取最差"口径最大差 1.08 km/h。
  - **有 1 个 NCAP 格子变色**：CCRb-60 原参数在 P5 里最差 9.93 km/h（黄），离 10 km/h 色带边界只差 0.07 km/h；量化后相位 0 为 10.53 km/h（橙）。原参数的简化 NCAP 估计因此从 4.17 变为 4.10，新参数 5.20 不变。贴着色带边界的结果本来就不稳，报告里应该标出余量。
  - 其余 95 次差异**全部**能用量化解释（`results/e1_quantization_explained.csv`）：
    - 76 次是升级阈值跨越被推后或提前一帧（预警 58、部分制动 6、全力制动 12）；
    - 19 次是制动释放条件（接近速度 ≤ 0.1 m/s，正好是 TgtRelSpd 的 2 个 LSB）提前或推后一帧；
    - 升级类差异处 \|TTC_量化 − TTC_浮点\| 中位数 4.1 ms，最大 112 ms（CCRb 目标刚开始减速，接近速度只有 0.36 m/s）。
  - 原因：P5 的场景从 TTC 6 s 开始、雷达 50 ms 一帧、阈值是 0.05 s 的整数倍，所以真实 TTC 会恰好在帧时刻等于阈值。例如浮点算出 2.599999 s 就触发预警，量化后 43.35 m / 16.65 m/s = 2.6036 s 要晚一帧。接近速度越小，TTC 量化误差越大，误差约为 (0.025/d + 0.025/v)·TTC。

**总线故障对 R152 的影响（E2–E4，判定口径同 P5：5.2.1.4 碰撞速度 + 5.2.1.1 预警提前量 + 5.2.1.2 制动需求）**

| 条件 | 原参数（2.6/1.6/0.9 s，4 m/s²） | 新参数（3.0/2.0/1.3 s，6 m/s²） |
|---|---|---|
| 随机丢帧 1 % / 5 % / 10 % / 20 %，基线 U：碰撞超限的运行占比（每格 600 次） | 0 / 1.7 % / 11.8 % / 58.7 % | 0 / 0.2 % / 3.5 % / 36.8 % |
| 随机丢帧 20 %，改进方案 HR | 0.8 %（5/600） | 0（另有 23 次预警提前量 < 0.8 s） |
| 关键时刻全总线突发 100 / 200 / 500 ms，基线 U：超限的（点, 相位, 偏移）组合（每格 330 个） | 221 / 275 / 307 | 30 / 153 / 245 |
| 同上，改进方案 HR | 0 / 0 / 42 | 0 / 0 / 0（最差 17.8 km/h，S60 限值 35） |
| 每帧附加时延 10 / 20 / 50 ms（19 点 × 5 相位） | 0 个点超限；S60 最差 8.1 / 12.0 / 18.2 km/h（无时延 5.2，限值 35） | 0 个点超限；最小车距 4.29 → 4.24 / 4.18 / 4.01 m |

- **突发丢帧最危险，且问题出在降级策略，而不是"雷达丢了几帧"**。基线 U 下，制动起点附近 100 ms 的全总线突发就让新参数在 30/330 个组合里超限。
- **机理分两种**：
  1. 车速报文 20 ms 一帧，3 倍周期即 60 ms 就判超时 → 上游进入 FAULT 并清零制动请求，输入恢复后还要再等 1.0 s；
  2. 突发发生在制动中时，制动 ECU 30 ms 收不到 AEB_CMD 就停止执行。
- **改进方案的效果**：
  - 只把恢复时间改成 0.2 s（R），新参数在所有突发条件下回到限值内（3 × 330 个组合 0 超限）；
  - 只做"保持"（H），只能修复制动中的突发；
  - 两者一起（HR）效果最好；
  - 完整对比见 `results/e3_burst_by_duration.csv`。
- **时延**：每 10 ms 总线时延使制动起点推迟 20 ms（雷达帧和指令帧各一次）。新参数在 19 个点上都有约 4 m 余量，不受影响；原参数在 S60 的碰撞速度随时延线性上升，但仍在限值内。

**E2E（E5、E6）**

- **检出能力**（表 S6，8 字节帧，CRC-8 按 Profile 1）：
  - 1 比特、2 比特错误穷举，100 % 检出；
  - 3 比特错误穷举（每个报文 5 个载荷 × 41 664 种），漏检 0.46 %（967/208 320）；
  - 4、8 比特随机样本，检出 99.6–99.7 %；
  - 只靠 DBC 范围检查，只能发现 5–21 % 的"改变了信号值"的损坏。
- **闭环单帧单比特损坏**（表 S7）：RADAR_TGT 的 28 个信号比特，从 T0（TTC 4 s）起每 100 ms 注入一帧，R152-S60 / M60 × 两套参数；有 E2E、无 E2E 各 4 256 次。
  - **无 E2E（范围检查开）**：
    - 313 次误制动 / 提前制动（TgtDist 96 次、TgtRelSpd 217 次），制动开始时真实 TTC 1.8–4.0 s，最多提前 2.45 s。这些制动都来不及提前 0.8 s 预警（多数是从 STANDBY 直接跳到部分制动），所以 R152 5.2.1.1 不满足。
    - 22 次制动推迟或中断，都在 R152-S60 原参数：
      - TgtValid / TgtQuality 翻转 12 次，让正在进行的制动松开；
      - TgtDist / TgtRelSpd 翻转 10 次，落在决定全力制动的那一帧上，使全力制动晚一帧；
      - 最大碰撞速度增加 11.6 km/h。
  - **有 E2E**：
    - 所有损坏帧都被拒收（WRONGCRC），被采用 0 次，误制动 0 次。
    - 但有 28 次在 R152-S60 原参数中，被拒收的恰好是决定进入全力制动的那一帧。这个点量化后余量约为 0 m，丢一帧就导致 8.1 km/h 碰撞（仍在 35 km/h 限值内）。
  - **结论**：E2E 把"用了错数据"变成了"少了一帧数据"，设计余量仍要能覆盖一个周期。
- **故障注入实测**（表 S8，R152-M60 新参数）：
  - 雷达丢 500 ms：110 ms 后进入 FAULT 并点亮报警灯，故障结束 1.20 s 后熄灭，FCW 提前量降到 0.34 s（< 0.8 s）。
  - 发送方卡死：第 2 个重复帧（50 ms）就判 E2E 故障。
  - AEB ECU bus-off 300 ms：制动 ECU 在最后一帧后 32 ms 停止执行，制动请求丢失 0.298 s。
  - 制动中 200 ms 突发：基线碰撞 19.8 km/h（限值 0），H 方案 0 km/h。

图：`results/fig_sl1_equivalence.png`（等价性与量化）、`results/fig_sl2_bus_faults.png`（丢帧 / 突发 / 时延）、`results/fig_sl3_timeline_e2e.png`（突发时间线与单比特损坏）。

## 5 局限

- **总线模型是抽象的**：
  - 没有仲裁、位定时、错误帧和 CAN 自身的 CRC；
  - bus-off 只阻断该节点的发送；
  - "比特翻转"是故障注入手段，不是误码率模型；
  - 时延量化到 2 ms。
- **只覆盖 R152 车对车**：信号级只跑了 R152 场景；Euro NCAP 网格只用于无故障等价性检查。
- **E2E 不是 AUTOSAR 实现**：只核实并复现了 Profile 1 的 CRC、计数器和 check 状态逻辑；调用方式、超时和降级是项目设计。Profile 1 已是 Legacy 规范。
- **改进方案 H / R / HR 只在本模型内评估**：
  - "保持"会把误制动一起保持，所以必须和 E2E 配合，并设时间上限；
  - 驾驶员模型和后车风险都没有建模；
  - 0.2 s 的恢复时间是否会造成状态抖动，没有专门评估。
- **其余沿用 P5 的局限**：一维纵向玩具模型，传感器、制动、路面理想化，没有驾驶员。

## 6 来源与未核实项

- **已核实**（官方 PDF，本地抽取文本逐条对照，并用官方示例做单元测试）：
  - AUTOSAR FO R24-11 PRS E2E Protocol（文档号 849）：6.1、6.5、6.5.1.1–6.5.1.3、6.5.2、6.5.3.1、6.5.4、6.5.5、6.12、9.8.2；
  - AUTOSAR CP R24-11 SWS CRC Library（文档号 16）：7.2.1.1。
- **沿用 P5 已核实的 R152 条款**：5.2.1.1、5.2.1.2、5.2.1.4、6.4、6.5、6.10.1（来自 EUR-Lex OJ L 2024/2497；数值直接从 `aebtest/regulations.py` 导入，没有重抄）。
- **【未核实】**：
  - R152 5.1.4.1 失效警告的原文措辞与时间要求（EUR-Lex / UNECE 本次打不开）；
  - ISO 26262 的 FTTI 概念与取值（原文没查阅）；
  - ISO 11898 的 CAN 帧位数、bus-off 恢复时间（没查阅，模型里也没用）；
  - SAE J1850 标准原文（只通过 AUTOSAR CRC 库规范引用其多项式）。

## 7 文件

```
aeb_demo.dbc            演示 DBC（SYNTHETIC，全部参数为设计假设）
dbc.py                  DBC 解析 + 信号编解码（Intel / Motorola、有符号、factor/offset、饱和）
e2e.py                  CRC-8（0x1D）与 Profile 1 protect / check
vbus.py                 虚拟总线与故障注入
supervision.py          接收监控：E2E 状态、可信性、超时、恢复
ecus.py                 RADAR / ESC / AEB / BRAKE / HMI 节点
sl_loop.py              信号级闭环（P5 closed_loop 的逐行镜像 + 总线）
sl_experiments.py       实验定义与并行运行（≤ 4 进程）
run_signal_level.py     E1–E6 全部实验 → results/*.csv、summary_signal_level.json
make_tables.py / make_figures.py   表格与图
tests/                  pytest：DBC、E2E、总线与监控、闭环（含与 P5 逐项一致性）
results/                CSV、tables_signal_level.md、fig_sl1–3.png
```

（本目录的结果已并入 P5 `report.md` 第 10 节。）
