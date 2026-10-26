#!/usr/bin/env python3
"""站点表回写工具：把笔记里的"站点表"（markdown 表格）回写成 mission_stages.json。

为什么有它：站点在笔记（`/home/king/下载/ai.memory/nav-mission-stages.md`）里改着舒服
（表格一眼看全、能写备注），但节点读的是 `nav_data/mission_stages.json`。这个脚本是
两者之间**唯一的同步口** —— 改笔记 → 跑本脚本 → 回写 + 校验 + 离线对拍每段规划。

用法：
    python3 stages_from_md.py                      # 默认笔记 + 默认输出，直接回写
    python3 stages_from_md.py --dry                # 只解析校验 + 打规划长度，不写文件
    python3 stages_from_md.py 别的.md --out 别的.json

表格格式（在 `<!-- STAGES:BEGIN` 和 `<!-- STAGES:END -->` 之间，列顺序固定）：

    | # | 站名 | x | z | done | number | aim | pitch | wait | stop |

    done   arrive=到点即走 / hp=把 number 打到 0 / rune=符 activated（且本站叫过符）
    number hp 站打几号（2..7）；不打的写 -
    aim    auto（交回自动搜索）/ enemy（对着本站 number 号）/ enemy=6（指定别的号）
           / rune（按住打符预设角——到打符位云台就正对着符）
    pitch  云台俯仰提示角（度，游戏系：正=俯视、负=抬头）；aim 非 enemy 时写 -
    wait   到点后停留秒数（补弹）；没有写 -
    stop   距实时目标多远停车打（单位）；缺省 4.5，写 - 表示用缺省
           写 fixed = 定点打：不追实时位置、不开到目标跟前 —— 直接开到 (x,z) 停车打，
           打到 hp 条件满足为止（用于"在实测选好的射击位打"，如四号·大坡旁）

块里还可以写一行（可选）：`rune_preset: yaw=40.70 pitch=-14.68` —— 打符站的云台预设角
（aim=rune 的站按住它；也是节点"到打符位正对着符"用的那两个角）。

校验项：done/aim 取值、hp 必须有 number、坐标在场地范围内、目标点是否可通行（就近找点会提示）、
第 3 次起的 rune 站会提示"按大符打"。对拍项：用 nav_map.FieldMap 把每一段 A* 规划一遍打长度。
"""
import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from nav_map import FieldMap            # noqa: E402  （规划对拍用）

DEFAULT_MD = "/home/king/下载/ai.memory/nav-mission-stages.md"
DEFAULT_OUT = os.path.abspath(os.path.join(HERE, "..", "..", "nav_data", "mission_stages.json"))
FX0, FX1, FZ0, FZ1 = -27.4, 17.7, -41.94, 41.94     # 场地范围（nav_map_report 同源）

NUM_RE = re.compile(r"^-?\d+(\.\d+)?$")


def parse_num(s, name, row_no, default=None, required=False):
    s = (s or "").strip()
    if s in ("", "-", "—"):
        if required:
            raise SystemExit(f"第 {row_no} 行：{name} 不能为空")
        return default
    if not NUM_RE.match(s):
        raise SystemExit(f"第 {row_no} 行：{name}='{s}' 不是数字")
    v = float(s)
    return int(v) if v == int(v) else v


def parse_md(path):
    """解析 STAGES 块里的 markdown 表 → stages 列表（顺序 = 表里的顺序）。"""
    try:
        text = open(path, encoding="utf-8").read()
    except OSError as e:
        raise SystemExit(f"读不了笔记 {path}：{e}")

    m = re.search(r"<!--\s*STAGES:BEGIN.*?-->(.*?)<!--\s*STAGES:END", text, re.S)
    if not m:
        raise SystemExit(f"{path} 里找不到 <!-- STAGES:BEGIN --> ... <!-- STAGES:END --> 块")

    stages = []
    rune_seen = 0
    preset = None
    for raw in m.group(1).splitlines():
        line = raw.strip()
        # 可选参数行：rune_preset: yaw=40.70 pitch=-14.68
        mp = re.match(r"^rune_preset\s*:\s*yaw\s*=\s*(-?[\d.]+)\s*[, ]+\s*pitch\s*=\s*(-?[\d.]+)",
                      line, re.I)
        if mp:
            preset = {"yaw": float(mp.group(1)), "pitch": float(mp.group(2))}
            continue
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        if len(cells) < 10:
            continue                              # 表头/分隔行/空行
        if not re.match(r"^\d+$", cells[0]):
            continue                              # 表头行（第一列是 # 或 ---）

        row_no = cells[0]
        name = cells[1]
        if not name:
            raise SystemExit(f"第 {row_no} 行：站名不能为空")

        x = parse_num(cells[2], "x", row_no, required=True)
        z = parse_num(cells[3], "z", row_no, required=True)
        done = cells[4].lower()
        if done not in ("arrive", "hp", "rune"):
            raise SystemExit(f"第 {row_no} 行：done='{done}' 不对（只能 arrive/hp/rune）")

        number = parse_num(cells[5], "number", row_no, default=0)
        aim_s = cells[6].lower()
        pitch = parse_num(cells[7], "pitch", row_no, default=-4.0)
        wait = parse_num(cells[8], "wait", row_no, default=0.0) or 0.0
        # stop 列：数字 = 停车距离；"fixed" = 定点打（不追目标，开到点停车打）
        stop_s = cells[9].strip().lower()
        fixed = stop_s in ("fixed", "定点", "y", "yes")
        stop = 0.0 if fixed else (parse_num(cells[9], "stop", row_no, default=0.0) or 0.0)

        if not (FX0 <= x <= FX1 and FZ0 <= z <= FZ1):
            print(f"  ⚠ 第 {row_no} 行 {name}：({x},{z}) 在场外（场地 x[{FX0},{FX1}] z[{FZ0},{FZ1}]）")

        st = {"name": name, "goal": [x, z], "done": done}
        if done == "hp":
            if not (2 <= number <= 7):
                raise SystemExit(f"第 {row_no} 行：hp 站 number 必须是 2..7，现在是 {number}")
            st["number"] = int(number)
        elif number:
            print(f"  ⚠ 第 {row_no} 行 {name}：done={done} 不消耗 number，已忽略")

        if aim_s.startswith("enemy"):
            tgt = number
            if "=" in aim_s:                      # enemy=6 → 对着 6 号（不管本站 number）
                tgt = int(parse_num(aim_s.split("=", 1)[1], "aim=号", row_no, required=True))
            if not (2 <= tgt <= 7):
                raise SystemExit(f"第 {row_no} 行：aim 目标 {tgt} 不在 2..7")
            st["aim"] = {"type": "enemy", "number": int(tgt), "pitch": pitch}
        elif aim_s == "rune":
            # 按住打符预设角（rune_preset 那行）；节点里还会顺带把打符模式顶起来
            st["aim"] = {"type": "rune"}
        elif aim_s in ("auto", "-", ""):
            st["aim"] = None
        else:
            raise SystemExit(f"第 {row_no} 行：aim='{cells[6]}' 不认识（auto / enemy / enemy=N）")

        if wait > 0:
            st["wait"] = wait
        if stop > 0:
            st["stop"] = stop
        if fixed:
            st["fixed"] = True
        if done == "rune":
            rune_seen += 1
        stages.append(st)

    if not stages:
        raise SystemExit("STAGES 块里一行站点都没解析到（表格式不对？）")
    print(f"解析 {len(stages)} 站（其中 rune {rune_seen} 站"
          + ("；第 3 次起按大符打" if rune_seen > 2 else "") + "）"
          + (f"，rune_preset yaw={preset['yaw']} pitch={preset['pitch']}" if preset else ""))
    return stages, preset


def write_json(out_path, stages, preset=None):
    """回写 mission_stages.json（rune_preset 优先用笔记里那行；没写就沿用旧值）。"""
    if preset is None:
        preset = {"yaw": 45.05, "pitch": -15.04}
        if os.path.exists(out_path):              # 沿用已有预设值（别在生成时改跑掉的常量）
            try:
                old = json.load(open(out_path, encoding="utf-8"))
                preset = old.get("rune_preset", preset)
            except Exception:
                pass

    def stage_line(st):
        # 坐标/stop 用 float 的 repr（30.0 不会被压成 30，和手写版逐字一致）；
        # wait 是整数秒；number/pitch 本来就该是整数
        parts = [f'"name": "{st["name"]}"',
                 f'"goal": [{float(st["goal"][0])!r}, {float(st["goal"][1])!r}]',
                 f'"done": "{st["done"]}"']
        if "number" in st:
            parts.append(f'"number": {st["number"]}')
        if "stop" in st:
            parts.append(f'"stop": {float(st["stop"])!r}')
        if st.get("fixed"):
            parts.append('"fixed": true')
        if "wait" in st:
            parts.append(f'"wait": {int(st["wait"])}')
        parts.append('"aim": ' + ("null" if st["aim"] is None else json.dumps(st["aim"], ensure_ascii=False)))
        return "    {" + ", ".join(parts) + "}"

    body = ",\n".join(stage_line(s) for s in stages)
    text = (
        '{\n'
        '  "_说明": "全自动任务站点表。【机器生成，别手改】改站点去 /home/king/下载/ai.memory/nav-mission-stages.md，'
        '然后跑 python3 contest_node/tools/stages_from_md.py 回写。done: arrive=到点即走 / hp=把 number 打到 0 / '
        'rune=activated 变 true（且本站叫过符）。aim: null=正常搜索; {type:enemy,number,pitch}=云台对住该目标（实时位置）。'
        'wait=到点后停留秒数（补弹）。stop=距实时目标多远停车打（缺省 4.5）。",\n'
        f'  "rune_preset": {{"yaw": {preset["yaw"]}, "pitch": {preset["pitch"]}}},\n'
        '  "stages": [\n' + body + "\n  ]\n}\n"
    )
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"已回写 {out_path}")


def plan_check(stages):
    """用 FieldMap 把每一段规划一遍打长度（和节点同口径：turn_penalty=0.4）。"""
    fm = FieldMap()
    tot = 0.0
    print("\n逐段规划（离线对拍，hp 段实跑会跟实时位置走，这些是配置点之间的数）：")
    for k in range(len(stages) - 1):
        a, b = tuple(stages[k]["goal"]), tuple(stages[k + 1]["goal"])
        if not fm.is_walkable(*b):
            print(f"  ⚠ 站 {k + 2} {stages[k + 1]['name']} 目标点 ({b[0]},{b[1]}) 不在可通行区"
                  f"（节点会就近找可走点）")
        p = fm.plan(a, b, turn_penalty=0.4)
        if p is None:
            print(f"  第{k + 1:2d}段 {stages[k]['name']} → {stages[k + 1]['name']}: ✗ 无路 !!")
            continue
        L = sum(math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1])
                for i in range(1, len(p)))
        tot += L
        print(f"  第{k + 1:2d}段 {stages[k]['name']:<8s} → {stages[k + 1]['name']:<8s} "
              f"{L:6.1f} 单位 = {L / 3:5.1f} m")
    print(f"  全程 {tot:.0f} 单位 = {tot / 3:.0f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("md", nargs="?", default=DEFAULT_MD, help="站点表笔记（默认 ai.memory 那份）")
    ap.add_argument("--out", default=DEFAULT_OUT, help="输出 json（默认 nav_data/mission_stages.json）")
    ap.add_argument("--dry", action="store_true", help="只校验+对拍，不写文件")
    args = ap.parse_args()

    stages, preset = parse_md(args.md)
    plan_check(stages)
    if args.dry:
        print("\n--dry：没写文件。")
        return
    write_json(args.out, stages, preset)
    print("改动生效方式：重启节点（启动时读一次，不用重新编译）。")


if __name__ == "__main__":
    main()
