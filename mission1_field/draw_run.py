#!/usr/bin/env python3
"""mission1_node.py 기록(JSON)을 그림으로: 칸 판정 + 이동 경로 + 칸별 증거 수.

  python3 draw_run.py logs/run_1021_153000.json              # 같은 이름 .png 로 저장
  python3 draw_run.py logs/run_1021_153000.json --truth rulebook   # 규정 그림 1 배치와 비교(잘못 판정 = 자주)
"""
import argparse
import json
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib import font_manager as fm            # noqa: E402
from matplotlib.patches import Rectangle             # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planner as P                                  # noqa: E402


def korean_font():
    try:
        f = subprocess.run(["fc-match", "-f", "%{file}", "Noto Sans CJK KR"], capture_output=True, text=True).stdout
        if f:
            fm.fontManager.addfont(f)
            plt.rcParams["font.family"] = fm.FontProperties(fname=f).get_name()
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json")
    ap.add_argument("--truth", default="", help="rulebook = 규정 그림 1 배치와 비교")
    ap.add_argument("-o", "--out", default="")
    a = ap.parse_args()
    korean_font()
    d = json.load(open(a.json))
    c, st = d["cell"], d["states"]
    truth = P.parse_map(P.RULEBOOK_MAP) if a.truth == "rulebook" else None
    occ, free = d["occ_scans"], d["free_scans"]

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 7.4))
    bad = 0
    for x in range(1, 10):
        for y in range(1, 10):
            s = d["grid"].get("%d,%d" % (x, y), st["UNKNOWN"])
            col = {st["OCC"]: "#e05050", st["FREE"]: "#c8eec8"}.get(s, "#eeeeee")
            if truth is not None and ((s == st["OCC"]) != ((x, y) in truth)) and s != st["UNKNOWN"]:
                col, bad = "#ff00ff", bad + 1
            a1.add_patch(Rectangle(((x - 1) * c, (y - 1) * c), c, c, fc=col, ec="#999999", lw=0.5))
            if truth is not None and (x, y) in truth:
                a1.add_patch(Rectangle(((x - 1) * c + 0.03, (y - 1) * c + 0.03), c - 0.06, c - 0.06,
                                       fc="none", ec="black", lw=2.2))
            # 오른쪽: 칸별 증거 (장애물로 보인 스캔 수 / 빈 칸으로 보인 스캔 수)
            o, f = occ[x][y], free[x][y]
            shade = min(1.0, o / 10.0)
            a2.add_patch(Rectangle(((x - 1) * c, (y - 1) * c), c, c, fc=(1.0, 1 - 0.7 * shade, 1 - 0.7 * shade),
                                   ec="#999999", lw=0.5))
            a2.text((x - 0.5) * c, (y - 0.5) * c, "%d\n%d" % (o, f), ha="center", va="center", fontsize=8,
                    color="#a00000" if o else "#2a7a2a")
    tr = d["trail"]
    if tr:
        xs, ys = [p[1] for p in tr], [p[2] for p in tr]
        for ax in (a1, a2):
            ax.plot(xs, ys, "-", color="#1f4fd8", lw=2.2)
            ax.plot(xs[0], ys[0], "o", color="green", ms=10)
            ax.plot(xs[-1], ys[-1], "s", color="#1f4fd8", ms=8)
    if len(d.get("path", [])) >= 2:
        px = [(p[0] - 0.5) * c for p in d["path"]]
        py = [(p[1] - 0.5) * c for p in d["path"]]
        a1.plot(px, py, "--", color="#ff8800", lw=1.8)
    for ax in (a1, a2):
        ax.plot(4.5 * c, 4.5 * c, "*", color="orange", ms=18)
        ax.add_patch(Rectangle((0, 0), 9 * c, 9 * c, fc="none", ec="black", lw=3))
        ax.set_xlim(-0.02, 9 * c + 0.02)
        ax.set_ylim(-0.02, 9 * c + 0.02)
        ax.set_aspect("equal")
        ax.set_xticks([(i - 0.5) * c for i in range(1, 10)])
        ax.set_xticklabels(range(1, 10))
        ax.set_yticks([(i - 0.5) * c for i in range(1, 10)])
        ax.set_yticklabels(range(1, 10))
    t1 = "로봇이 판정한 지도 · 이동 경로" + ("  (잘못 판정 %d칸)" % bad if truth is not None else "")
    a1.set_title(t1, fontsize=13)
    a2.set_title("칸별 증거: 위 = 장애물로 보인 스캔 수, 아래 = 빈 칸으로 보인 스캔 수", fontsize=12)
    fig.text(0.5, 0.965, "%s · %s · %.0f초 · 칸 %.2f m" % (os.path.basename(a.json), d["result"], d["sec"], c),
             ha="center", fontsize=14)
    fig.text(0.5, 0.93, "빨강 = 장애물 판정 · 연두 = 빈 칸 · 회색 = 모름" + (" · 검은 테두리 = 실제 장애물 · 자주 = 잘못 판정" if truth is not None else "")
             + " · 파란 선 = 실제 이동 · 주황 점선 = 마지막 계획 경로 · 초록 점 = 출발 · 별 = 중앙", ha="center", fontsize=10)
    plt.tight_layout(rect=(0, 0, 1, 0.91))
    out = a.out or os.path.splitext(a.json)[0] + ".png"
    plt.savefig(out, dpi=85)
    print(out)


if __name__ == "__main__":
    main()
