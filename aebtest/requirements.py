"""Requirement registry for the traceability matrix (scripts/make_traceability.py -> traceability.csv).

Chain: regulation / protocol clause -> function requirement (ID) -> signals of the reused function
(opendbc Toyota DBC messages used by can-bus-analyzer's AebEcu; in P5 the same quantities are produced by
the sensor model without CAN encoding) -> OpenSCENARIO test case IDs -> result.

Signal names were checked against dbc/toyota_nodsu_pt_generated.dbc of can-bus-analyzer @27da004:
  inputs : LEAD_INFO (0x2E6) LEAD_LONG_DIST, LEAD_REL_SPEED; SPEED (0xB4) SPEED; GAS_PEDAL GAS_PEDAL;
           BRAKE_MODULE BRAKE_PRESSED
  outputs: PRE_COLLISION (0x283) STATE, PRECOLLISION_ACTIVE, FORCE (= -1500 kg x request), _COUNTER;
           PCS_HUD (0x411) FCW, PCS_INDICATOR
"""
from __future__ import annotations

from dataclasses import dataclass, field

R152 = "UN R152 (OJ L 2024/2497)"
NCAP = "Euro NCAP CA Frontal Collisions v1.2"
ISO21448 = "ISO 21448 (SOTIF) - 原文未查阅"
HARA = "本项目示意性 HARA（report.md 第 9 节）"

IN_RADAR = "LEAD_INFO.LEAD_LONG_DIST, LEAD_INFO.LEAD_REL_SPEED"
IN_SPEED = "SPEED.SPEED"
OUT_BRAKE = "PRE_COLLISION.PRECOLLISION_ACTIVE, PRE_COLLISION.FORCE, PRE_COLLISION.STATE"
OUT_WARN = "PCS_HUD.FCW"


@dataclass(frozen=True)
class Req:
    rid: str
    source: str
    clause: str
    text: str                 # requirement as tested here (Chinese)
    kind: str                 # 功能 | 性能 | 试验条件 | HMI | 安全目标
    signals: str
    safety_goal: str = ""
    tests: str = ""           # selector key used by make_traceability.py
    na_reason: str = ""       # if not verified in P5: why, and where it is verified instead
    notes: str = ""


REQS: list[Req] = [
    Req("FR-01", R152, "5.2.1.1", "车对车：碰撞预警最迟在紧急制动开始前 0.8 s 发出（恒速目标）", "功能",
        f"{IN_RADAR}; {IN_SPEED} -> {OUT_WARN}, {OUT_BRAKE}", "SG2", tests="r152:lead"),
    Req("FR-02", R152, "5.2.1.2", "判定可能发生碰撞时，对行车制动系统的制动需求至少 5.0 m/s²", "功能",
        f"{IN_RADAR} -> PRE_COLLISION.FORCE（5 m/s² ↔ -7500 N，按 1500 kg）", "SG2", tests="r152:demand",
        notes="按字面解读；部分制动 4 m/s² 就已避撞的情形是否被技术服务机构接受未核实"),
    Req("FR-03", R152, "5.2.1.3", "至少在 10–60 km/h 车速范围内处于激活状态", "功能", IN_SPEED, "SG2",
        tests="r152:active", notes="另做静态检查：v_min_kph=8 ≤ 10，v_max_kph=130 ≥ 60"),
    Req("FR-04", R152, "5.2.1.4 + 6.4", "静止目标：相对碰撞速度不超过 M1 表中限值（表间取更高一档）", "性能",
        f"{IN_RADAR} -> {OUT_BRAKE}", "SG2", tests="r152:impact:CCRs"),
    Req("FR-05", R152, "5.2.1.4 + 6.5", "20 km/h 运动目标：相对碰撞速度不超过 M1 表中限值（试验点要求不碰撞）", "性能",
        f"{IN_RADAR} -> {OUT_BRAKE}", "SG2", tests="r152:impact:CCRm"),
    Req("FR-06", R152, "5.5.1", "碰撞预警至少采用声、触、光中的两种方式", "HMI", OUT_WARN, "SG2",
        na_reason="HMI 实现不在仿真范围；状态机只输出一个 PCS_HUD.FCW 位，需在整车 HMI 层验证"),
    Req("FR-07", R152, "5.3.1 / 5.3.2", "驾驶员可通过明确操作（如急踩加速踏板）中断预警和紧急制动", "功能",
        "GAS_PEDAL.GAS_PEDAL, BRAKE_MODULE.BRAKE_PRESSED -> PRE_COLLISION.*", "SG3",
        na_reason="P5 未复测；上游 can-bus-analyzer 在 CAN 总线上测过 OVR-01/OVR-02/DRV-01（见其 results/function/testcases.md）。"
                  "状态机没有转向超越条件，属已知缺口"),
    Req("FR-08", R152, "5.1.4.1 / 6.8", "系统失效时点亮恒定黄色失效警告", "功能", "PCS_HUD.PCS_INDICATOR=2（FAULT）", "SG1/SG2",
        na_reason="P5 不经过 CAN 输入监控；上游测过 FLT-01..04（超时 / checksum → FAULT）"),
    Req("FR-09", R152, "5.1.6", "无碰撞风险时尽量少发预警、避免紧急制动（误作用抑制）", "功能",
        f"{IN_RADAR} -> {OUT_WARN}, {OUT_BRAKE}", "SG1", tests="sotif:ghost",
        notes="R152 以附件 3 文件审查方式评估；P5 用幻影目标注入量化误制动（ISO 21448 触发条件 TC4）"),
    Req("TC-01", f"{R152} / {NCAP}", "R152 6.4/6.5 (b)；NCAP 1.5.1", "试验开始时 TTC ≥ 4 s（NCAP：T0 = TTC 4 s）", "试验条件",
        "-", tests="all:ccr_constant", notes="本项目在 TTC 6 s 处开始仿真（2 s 预行驶）"),
    Req("TC-02", R152, "6.5", "运动目标车速 20 km/h（公差 +0/-2）", "试验条件", "-", tests="r152:CCRm"),
    Req("TC-03", NCAP, "3.1.1.1（p.30）", "CCRb：车头时距 1.0 s，目标减速度 -4 m/s²，1.0 s 内达到", "试验条件", "-",
        tests="ncap:CCRb_def", notes="本项目用阶跃减速度（0 s 达到），是满足“1.0 s 内”的最严格情形"),
    Req("FR-10", NCAP, "3.1.1.1 + 5.2 图 5-1 + 5.3.1", "CCRs 网格（50 % 撞击位置）按相对碰撞速度定色", "性能",
        f"{IN_RADAR} -> {OUT_BRAKE}, {OUT_WARN}", "SG2", tests="ncap:CCRs"),
    Req("FR-11", NCAP, "3.1.1.1 + 5.2 图 5-1 + 5.3.1", "CCRm 网格（50 % 撞击位置）按相对碰撞速度定色", "性能",
        f"{IN_RADAR} -> {OUT_BRAKE}", "SG2", tests="ncap:CCRm"),
    Req("FR-12", NCAP, "3.1.1.1 + 5.2 图 5-1 + 5.3.1", "CCRb 网格（50 % 撞击位置）按相对碰撞速度定色", "性能",
        f"{IN_RADAR} -> {OUT_BRAKE}", "SG2", tests="ncap:CCRb"),
    Req("FR-13", NCAP, "5.1", "CCRs 在 ≤20 km/h 的所有标准范围撞击位置必须完全避撞（得分资格）", "性能",
        f"{IN_RADAR} -> {OUT_BRAKE}", "SG2", tests="ncap:gate"),
    Req("SG4-01", HARA, "H4 → SG4", "目标仍在减速或已停止时，不应在本车停住前过早释放制动（二次接触）", "安全目标",
        OUT_BRAKE, "SG4", tests="ncap:late",
        notes="Euro NCAP 在 V_VUT = V_target 时结束试验，看不到这一项；需逻辑改动，参数改动不能完全消除"),
    Req("ODD-01", R152, "5.2.1.4 条件 (a)(i)、5.2.1.4 末段、6.1.6", "湿滑路面（μ=0.7/0.5）：表中限值只对干燥路面有约束力；"
        "偏离条件时系统不得停用或不合理地切换控制策略——本项目按表做模型内参考评价", "性能（参考）",
        f"{IN_RADAR} -> {OUT_BRAKE}", "SG2", tests="wet", notes="μ 值、μ 与减速度的关系是项目假设"),
    Req("SOTIF-01", ISO21448, "触发条件 TC2：雨天探测距离下降", "探测距离降到 60 / 40 m 时仍满足 FR-01 的 0.8 s 预警提前量", "功能（参考）",
        IN_RADAR, "SG2", tests="sotif:range", notes="探测距离数值是项目假设"),
]


def by_id() -> dict[str, Req]:
    return {r.rid: r for r in REQS}
