#!/usr/bin/env python3
"""2임무 기록 그림: python3 draw_battle.py logs/battle_XXXX.csv [출력.png]. 노트북에서. 파랑 = 벽 보정 위치, 회색 = 오돔만."""
import csv, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
try:
    fm.fontManager.addfont(F); plt.rcParams["font.family"] = "Noto Sans CJK JP"
except Exception:
    pass
rows = list(csv.DictReader(open(sys.argv[1])))
out = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1][:-4] + ".png"
fig, ax = plt.subplots(figsize=(7, 7), dpi=130)
for v in [-1.8 + 0.4 * i for i in range(10)]:
    ax.plot([v, v], [-1.8, 1.8], color="#e8e8e8", lw=0.8); ax.plot([-1.8, 1.8], [v, v], color="#e8e8e8", lw=0.8)
ax.plot([-1.8, 1.8, 1.8, -1.8, -1.8], [-1.8, -1.8, 1.8, 1.8, -1.8], color="#1a1a1a", lw=2.5)
ax.plot([-1.4, 1.4, 1.4, -1.4, -1.4], [-1.4, -1.4, 1.4, 1.4, -1.4], color="#eda100", lw=2, label="노란 선")
ax.plot([-1.2, 1.2, 1.2, -1.2, -1.2], [-1.2, -1.2, 1.2, 1.2, -1.2], color="#6b6b6b", lw=1, ls=(0, (4, 3)), label="안전선")
ax.plot([float(r["odom_x"]) for r in rows], [float(r["odom_y"]) for r in rows], color="#a0a0a0", lw=1.2, label="오돔만")
ax.plot([float(r["x"]) for r in rows], [float(r["y"]) for r in rows], color="#2a78d6", lw=2, label="벽 보정 위치")
prev = None
for r in rows:
    if r["state"] != prev:
        ax.annotate("%s %.0f초" % (r["state"], float(r["t"])), (float(r["x"]), float(r["y"])), xytext=(6, 6), textcoords="offset points", fontsize=8, color="#1a1a1a")
        prev = r["state"]
ax.set_xlim(-1.95, 1.95); ax.set_ylim(-1.95, 1.95); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
for s in ax.spines.values(): s.set_visible(False)
last = rows[-1]
ax.set_title("%.0f초 · 벽 보정 %s번 · 안전선 %s번 · 맞서기 %s번 · 오류 %s번" % (float(last["t"]), last["wall_fixes"], last["guard"], last["evades"], last["errors"]), fontsize=10, loc="left")
ax.legend(loc="upper center", bbox_to_anchor=(0.5, 0.02), ncol=4, fontsize=9, frameon=False)
fig.tight_layout(); fig.savefig(out, facecolor="white"); print(out)
