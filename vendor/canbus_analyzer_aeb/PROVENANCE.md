# 来源说明（vendored 代码）

本目录的 3 个 `.py` 文件是从申请人自己的公开仓库**原样复制**的，一个字节都没改：

- 仓库：https://github.com/LUOaini1213/can-bus-analyzer
- 提交：`27da00499f1126000d9975d4af12c145a2ddee21`（2026-10-15，"Add AEB function state machine, bus supervision and CCR scenario matrix"）
- 复制方式：`git show 27da004:canbus_analyzer/<file>`（只读克隆，没有改动或推送 GitHub）

| 文件 | 上游路径 | git blob（可用 `git rev-parse 27da004:<路径>` 核对） | SHA-256 |
|---|---|---|---|
| aeb_function.py | canbus_analyzer/aeb_function.py | 4474f26f8f082630c35fb6452b38b053207aa151 | c90dff394375959636cf4ddac63bc53e00e0d8e9466567f73d3c060d5d188c18 |
| analyzer.py | canbus_analyzer/analyzer.py | 5583db1396d5b115d6801527ef26ee191ac6ee72 | 71cee951515fca74d14be224043c5c27d91906ce2e2d1ae26f885d367bac3518 |
| config.py | canbus_analyzer/config.py | 3c4c9a759f0651665dceccb76c96bad7dca39c6c | 31d760784d474d7f4abfbe3e8311046ea5c31629d11654c18931c4c11e12703d |

`LICENSE` 是上游仓库的 MIT 许可证原文。

## 怎么用的（`aebtest/aeb_loader.py`）

- `aeb_function.py` 原样加载执行。本项目用它的 `S`、`AebConfig`、`Inputs`、`TRANSITIONS`、`StateMachine`，即状态机本体。
- `aeb_function.py` 顶部有两条相对导入：
  - `from .analyzer import ttc`：`analyzer.py` 依赖 python-can，本机没装，也不允许再下载。所以加载器用 Python 的 `ast` 从 `analyzer.py` 原文里**截取 `ttc()` 函数的源码**执行，常数 `MIN_CLOSING_MPS` 从 `config.py` 原文读取（值为 0.1 m/s）。执行的是上游原文，不是重新手写的版本。
  - `from .dbcutil import load_db, signal_limits, toyota_checksum`：只有 CAN 外壳（`Supervisor`、`AebEcu`）会用到，状态机不用。本项目不走 CAN 总线，所以换成调用即报错的桩函数。
- 一句话：**状态机代码与上游完全一致；本项目替换的只是它外面的 CAN 帧收发层**，输入改由传感器模型直接给出。

核对命令：`python -I scripts/check_vendor.py`（重新计算 SHA-256 和 git blob 哈希并与上表比对）。
