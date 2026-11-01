"""캘리브레이션 촬영 (브라우저 http://<로봇IP>:8090). 카메라 설정은 바꾸지 않음(읽기만).
mono  : 카메라마다 판이 멈춰 있고 새 자세면 따로 저장 (① 렌즈 보정)
stereo: 두 카메라 모두 판이 다 보이고 멈춰 있고 새 자세면 한 쌍 저장 (② 두 카메라 관계)
check : 저장 버튼 누를 때만 한 쌍 저장 (③ 큐브 확인)
원본은 덮어쓰지 않고 번호를 이어 붙임. 판 종류(칸 크기)는 사진마다 csv 에 기록."""
import cv2, threading, time, json, glob, os, sys, csv, subprocess, datetime, numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

PAT, W, H = (7, 4), 1280, 720
CAMS = {"left": "usb-0:2.1.2:1.0", "right": "usb-0:2.1.4:1.0"}
BOARDS = {"20": 20.0, "10": 10.0}  # 2 cm 판 8칸 = 160 mm 실측(10-29), 1 cm 판은 같은 프린터·설정
STILL_PX, MAX_EDGE, NOVEL, SYNC_S, TILTED = 1.0, 8.0, 0.035, 0.10, 0.12
GOAL = {"mono": 30, "stereo": 30, "check": 10}
ROOT = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/mecanum_slam_ws/calib/session_" + datetime.date.today().strftime("%Y%m%d"))
# 기울기는 대략의 렌즈값으로 판 자세를 풀어 잼 (10-29: 테두리 길이로 어림한 기울기는 왜곡 때문에 틀렸음 — 실제 20° 넘는 판 0장)
MIN_TILT = 25.0
_G = os.path.expanduser("~/mecanum_slam_ws/calib/calib_guess.npz")
if os.path.exists(_G):
    _g = np.load(_G); KG = {"left": _g["K_L"], "right": _g["K_R"]}; DG = {"left": _g["D_L"], "right": _g["D_R"]}
else:
    KG = {s: np.array([[850.0, 0, 640], [0, 850.0, 360], [0, 0, 1]]) for s in ("left", "right")}; DG = {s: np.zeros(5) for s in ("left", "right")}


def dev_of(tag):
    m = glob.glob("/dev/v4l/by-path/*%s-video-index0" % tag)
    if not m: raise SystemExit("camera not found: " + tag)
    return os.path.realpath(m[0])


DEVS = {k: dev_of(v) for k, v in CAMS.items()}
latest, det, size = {}, {}, {}


def grab(name):
    c = cv2.VideoCapture(DEVS[name], cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")); c.set(cv2.CAP_PROP_FRAME_WIDTH, W); c.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    n = 0
    while True:
        ok, f = c.read()
        if not ok: time.sleep(0.05); continue
        n += 1; size[name] = f.shape[1::-1]; latest[name] = (f, time.time(), n)


def detect(name):
    prev, still_n, last_ok = None, 0, 0.0
    while True:
        cur = latest.get(name)
        if cur is None or (name in det and det[name]["n"] == cur[2]): time.sleep(0.01); continue
        f, t, n = cur
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        ok, c = False, None
        # 빠른 사전 검사는 판을 놓쳤을 때만. 반 크기로 줄여 검사하면 판이 다 보여도 20% 놓침(저장 사진 13/64) → 원래 크기(0/64, 7 ms)
        if t - last_ok < 1.0 or cv2.checkChessboard(g, PAT):
            # NORMALIZE_IMAGE 만 쓰면 넓은 단색 배경에서 놓침(시험 이미지로 확인) → EXHAUSTIVE 먼저, 안 되면 NORMALIZE
            ok, c = cv2.findChessboardCornersSB(g, PAT, flags=cv2.CALIB_CB_EXHAUSTIVE)
            if not ok: ok, c = cv2.findChessboardCornersSB(g, PAT, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            if t - last_ok > 0.5: prev, still_n = None, 0  # 한두 장 놓친 건 멈춤 판정을 끊지 않음
            det[name] = dict(n=n, t=t, corners=None, why="none"); time.sleep(0.05); continue
        last_ok = t
        c = c.reshape(-1, 2)
        edge = float(cv2.estimateChessboardSharpness(g, PAT, c.reshape(-1, 1, 2))[0][0])
        # 멈춤 기준: ①(한 대씩)은 3 px — 판을 젖혀 손으로 들면 1 px 안으로 못 멈춰 세로 기울임이 하나도 저장 안 됐음(10-29).
        # 흐림은 선명도 검사가 따로 거름. ②는 두 카메라 사진 시각이 최대 0.1 s 달라 1.5 px 로 더 엄격하게
        lim = 3.0 if state["mode"] == "mono" else 1.5
        if prev is not None and t - prev[1] < 0.6 and np.abs(c - prev[0]).max() < lim: still_n += 1
        else: still_n = 0
        prev = (c, t)
        tdeg, tdir = tilt(c, name)
        why = "moving" if still_n < 2 else ("blurry" if edge > MAX_EDGE else ("flat" if tdeg < MIN_TILT else "ok"))
        det[name] = dict(frame=f, n=n, t=t, corners=c, edge=edge, why=why, tilt=tdeg, tdir=tdir)


def feat(c): return (c[[0, 6, 21, 27]] / [W, H]).ravel()
def novel(f, saved): return all(np.abs(f - s).mean() > NOVEL for s in saved)


_OBJ = np.zeros((PAT[0] * PAT[1], 3), np.float32); _OBJ[:, :2] = np.mgrid[0:PAT[0], 0:PAT[1]].T.reshape(-1, 2)


def normal(c, cam):  # 판 법선(카메라 좌표). 광축과 이루는 각 = 기울기
    ok, rv, tv = cv2.solvePnP(_OBJ, c.astype(np.float32), KG[cam], DG[cam])
    n = cv2.Rodrigues(rv)[0][:, 2]
    return n if n[2] > 0 else -n


def tilt(c, cam):  # (각도°, 방향) 방향: 가로 = 좌우로 돌림, 세로 = 위아래로 젖힘
    n = normal(c, cam)
    return float(np.degrees(np.arccos(min(1.0, abs(n[2]))))), ("가로" if abs(n[0]) >= abs(n[1]) else "세로")


def fvec(c, cam): return np.concatenate([feat(c), normal(c, cam)[:2]])  # 같은 자리라도 기울기가 다르면 새 자세


def same_order(a, b):  # 좌우가 같은 꼭짓점에 같은 번호를 붙였는지 (0→6, 0→21 방향이 비슷)
    for k in (6, 21):
        u, v = a[k] - a[0], b[k] - b[0]
        if np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v) + 1e-9) < 0.5: return False
    return True


GX, GY = 8, 6
cov = {"mono": {s: np.zeros((GY, GX), int) for s in CAMS},  # ②는 판별로 따로 표시 (큰 판으로 바꾸면 큰 판 것만)
       "stereo": {b: {s: np.zeros((GY, GX), int) for s in CAMS} for b in BOARDS}}


def add_cov(arr, c):
    m = np.zeros((GY * 10, GX * 10), np.uint8)
    cv2.fillConvexPoly(m, cv2.convexHull((c / [W, H] * [GX * 10, GY * 10]).astype(np.int32)), 1)
    hit = cv2.resize(m.astype(np.float32), (GX, GY), interpolation=cv2.INTER_AREA) > 0.3
    for x, y in (c / [W, H] * [GX, GY]).astype(int):  # 작은 판은 칸의 30%를 못 덮어 초록이 안 됐음 → 꼭짓점이 들어간 칸도 표시
        hit[min(max(y, 0), GY - 1), min(max(x, 0), GX - 1)] = True
    arr += hit


state = dict(mode="mono", board="20", paused=False, ready=False, msg="이전 사진 불러오는 중", flash={"left": 0, "right": 0})
saved = {"mono": {s: [] for s in CAMS}, "stereo": []}
count = {"mono": {s: 0 for s in CAMS}, "tilted": {s: {"가로": 0, "세로": 0} for s in CAMS}, "stereo": {"20": 0, "10": 0},
         "tilt_st": {"가로": 0, "세로": 0}, "check": 0}
hint = {s: "" for s in CAMS}
lock = threading.Lock()
UI = os.path.join(ROOT, "ui_state.json")  # 다시 켜도 단계·판 버튼 유지 (재시작 때 ①·2 cm 로 돌아가 판 크기가 틀리게 기록될 뻔함)
if os.path.exists(UI): state.update(json.load(open(UI)))


def next_id(*dirs):
    ids = [0]
    for d in dirs:
        os.makedirs(d, exist_ok=True)
        ids += [int(x[:6]) for x in os.listdir(d) if x[:6].isdigit()]
    return max(ids) + 1


def write_png(path, f):
    if os.path.exists(path): raise RuntimeError("exists: " + path)
    cv2.imwrite(path, f)


def append_csv(path, head, row):
    new = not os.path.exists(path)
    with open(path, "a", newline="") as fh:
        w = csv.writer(fh)
        if new: w.writerow(head)
        w.writerow(row)


def snap(tag, sub=""):  # 웹 화면 그대로(꼭짓점·칸 표시 포함) 기록 — 포트폴리오·과정 기록용, 시각으로 이름 붙여 덮어쓰기 없음
    d = os.path.join(ROOT, "web_view", sub); os.makedirs(d, exist_ok=True)
    cv2.imwrite(os.path.join(d, "%s_%s.jpg" % (datetime.datetime.now().strftime("%H%M%S_%f")[:-3], tag)),
                np.hstack([panel("left"), panel("right")]), [cv2.IMWRITE_JPEG_QUALITY, 85])


def timelapse():
    while True:
        time.sleep(2.0)
        if state["ready"] and not state["paused"]: snap(state["mode"], "timelapse")


def save_mono(name, d):
    dd = os.path.join(ROOT, "raw_intrinsic", name); i = next_id(dd)
    p = os.path.join(dd, "%06d.png" % i); write_png(p, d["frame"])
    tl, tdir = tilt(d["corners"], name)
    append_csv(os.path.join(ROOT, "intrinsic.csv"),
               ["sample_id", "camera", "path", "frame_id", "time_unix", "board_pitch_mm", "edge_px", "tilt"],
               [i, name, os.path.relpath(p, ROOT), d["n"], "%.3f" % d["t"], BOARDS[state["board"]], "%.2f" % d["edge"], "%.1f" % tl])
    count["mono"][name] += 1; count["tilted"][name][tdir] += int(tl >= MIN_TILT)
    add_cov(cov["mono"][name], d["corners"]); state["flash"][name] = time.time()
    state["msg"] = "%s %d번 저장" % ("왼쪽" if name == "left" else "오른쪽", i); snap("mono_%s_%06d" % (name, i))


def save_pair(kind, L, R, label):
    sub = "raw_stereo" if kind == "stereo" else "validation"
    dl, dr = os.path.join(ROOT, sub, "left"), os.path.join(ROOT, sub, "right"); i = next_id(dl, dr)
    pl, pr = os.path.join(dl, "%06d.png" % i), os.path.join(dr, "%06d.png" % i)
    write_png(pl, L["frame"]); write_png(pr, R["frame"])
    append_csv(os.path.join(ROOT, "pairs.csv"),
               ["pair_id", "set", "left_path", "right_path", "left_frame_id", "right_frame_id", "left_time", "right_time",
                "dt_ms", "board_pitch_mm", "edge_L", "edge_R", "label"],
               [i, kind, os.path.relpath(pl, ROOT), os.path.relpath(pr, ROOT), L["n"], R["n"], "%.3f" % L["t"], "%.3f" % R["t"],
                "%.1f" % ((R["t"] - L["t"]) * 1000), BOARDS[state["board"]] if kind == "stereo" else "",
                "%.2f" % L["edge"] if L.get("edge") else "", "%.2f" % R["edge"] if R.get("edge") else "", label])
    for s, d in (("left", L), ("right", R)):
        if kind == "stereo": add_cov(cov["stereo"][state["board"]][s], d["corners"])
        state["flash"][s] = time.time()
    if kind == "stereo":
        count["stereo"][state["board"]] += 1; tl, tdir = tilt(L["corners"], "left"); count["tilt_st"][tdir] += int(tl >= MIN_TILT)
    else: count["check"] += 1
    state["msg"] = "%s %d번 저장" % ("한 쌍" if kind == "stereo" else "확인 " + label, i); snap("%s_%06d" % (kind, i))


def corners_of(path):
    g = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if g is None: return None
    ok, c = cv2.findChessboardCornersSB(g, PAT, flags=cv2.CALIB_CB_EXHAUSTIVE)
    if not ok: ok, c = cv2.findChessboardCornersSB(g, PAT, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    return c.reshape(-1, 2) if ok else None


def reload():  # 다시 켰을 때 이미 찍은 사진으로 개수·칸 표시·비슷한 자세 판정을 되살림
    for s in CAMS:
        d = os.path.join(ROOT, "raw_intrinsic", s)
        for fn in sorted(os.listdir(d)) if os.path.isdir(d) else []:
            c = corners_of(os.path.join(d, fn))
            if c is None: continue
            tl, tdir = tilt(c, s)
            saved["mono"][s].append(fvec(c, s)); count["mono"][s] += 1; count["tilted"][s][tdir] += int(tl >= MIN_TILT); add_cov(cov["mono"][s], c)
    pc = os.path.join(ROOT, "pairs.csv")
    for r in (csv.DictReader(open(pc)) if os.path.exists(pc) else []):
        if r["set"] == "check": count["check"] += 1; continue
        cl, cr = corners_of(os.path.join(ROOT, r["left_path"])), corners_of(os.path.join(ROOT, r["right_path"]))
        if cl is None or cr is None: continue
        b = "10" if float(r["board_pitch_mm"]) < 15 else "20"
        saved["stereo"].append(np.concatenate([fvec(cl, "left"), fvec(cr, "right")])); tl, tdir = tilt(cl, "left"); count["tilt_st"][tdir] += int(tl >= MIN_TILT); add_cov(cov["stereo"][b]["left"], cl); add_cov(cov["stereo"][b]["right"], cr)
        count["stereo"][b] += 1
    state["ready"] = True; state["msg"] = "준비됨 (이전 사진 %d장 반영)" % (sum(count["mono"].values()) + 2 * sum(count["stereo"].values()))


def fresh(d): return d is not None and d.get("corners") is not None and time.time() - d["t"] < 0.4


def saver():
    while True:
        time.sleep(0.1)
        mode = state["mode"]
        if state["paused"] or mode == "check" or not state["ready"]: continue
        with lock:
            if mode == "mono":
                for s in CAMS:
                    d = det.get(s)
                    if not fresh(d) or d["why"] != "ok": continue
                    fv = fvec(d["corners"], s)
                    if not novel(fv, saved["mono"][s]): hint[s] = "similar"; continue
                    saved["mono"][s].append(fv); save_mono(s, d); hint[s] = ""
            else:
                L, R = det.get("left"), det.get("right")
                if not (fresh(L) and fresh(R)) or L["why"] != "ok" or R["why"] != "ok": continue
                if abs(L["t"] - R["t"]) > SYNC_S: continue
                if not same_order(L["corners"], R["corners"]): state["msg"] = "좌우 꼭짓점 번호가 달라 건너뜀"; continue
                ft = np.concatenate([fvec(L["corners"], "left"), fvec(R["corners"], "right")])
                if not novel(ft, saved["stereo"]): hint["left"] = hint["right"] = "similar"; continue
                saved["stereo"].append(ft); save_pair("stereo", L, R, ""); hint["left"] = hint["right"] = ""


def panel(s):
    cur = latest.get(s)
    if cur is None: return np.zeros((360, 640, 3), np.uint8)
    p = cv2.resize(cur[0], (640, 360)); h, w = p.shape[:2]
    mode = state["mode"]
    if mode in cov:
        a = cov["mono"][s] if mode == "mono" else cov["stereo"][state["board"]][s]; ov = p.copy()
        for y in range(GY):
            for x in range(GX):
                if a[y, x] > 0:
                    cv2.rectangle(ov, (x * w // GX, y * h // GY), ((x + 1) * w // GX, (y + 1) * h // GY), (0, 200, 0), -1)
        p = cv2.addWeighted(ov, 0.25, p, 0.75, 0)
        if mode == "mono":  # 아직 판이 안 지나간 칸 = 빨간 테두리 (색이 틀어진 것처럼 보이지 않게 칠하지 않음)
            for y in range(GY):
                for x in range(GX):
                    if a[y, x] == 0:
                        cv2.rectangle(p, (x * w // GX + 3, y * h // GY + 3), ((x + 1) * w // GX - 3, (y + 1) * h // GY - 3), (0, 0, 255), 1)
    d = det.get(s); why = "none"
    if fresh(d):
        c = d["corners"] * [w / W, h / H]; why = d["why"]
        col = (0, 255, 0) if why == "ok" else (0, 200, 255)
        cv2.polylines(p, [c.astype(np.int32).reshape(-1, 1, 2)], False, col, 1, cv2.LINE_AA)
        cv2.circle(p, tuple(c[0].astype(int)), 7, (0, 0, 255), -1); cv2.circle(p, tuple(c[6].astype(int)), 7, (255, 0, 0), -1)
    if why == "ok" and hint[s] == "similar": why = "similar"
    cv2.rectangle(p, (0, 0), (w, 30), (0, 0, 0), -1)
    e = " edge %.1fpx  tilt %.0f" % (d["edge"], d["tilt"]) if fresh(d) else ""
    cv2.putText(p, "%s  %s%s" % (s.upper(), why.upper(), e), (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA)
    if time.time() - state["flash"][s] < 0.6: cv2.rectangle(p, (0, 0), (w - 1, h - 1), (0, 255, 0), 10)
    return p


# 칸별 안내: (색, 글) — 화면 위 큰 상자에 표시
HINT_KO = {"saved": ("#1a8f3c", "✔ 저장됨 → 다음 자리로 옮기세요"), "similar": ("#1a8f3c", "✔ 이 자리는 끝 → 다음 자리로 옮기세요"),
           "ok": ("#b8860b", "그대로 멈춰 있기… 곧 저장"), "moving": ("#b8860b", "판을 멈춰 주세요"),
           "blurry": ("#c0501a", "흐림 → 조금 멀리"), "none": ("#555", "판 안 보임 (판 전체가 화면에 들어와야 함)"),
           "flat": ("#b8860b", "판을 더 기울여 주세요 (25° 이상)")}
ROWS, COLS = [("위", 0, 2), ("가운데", 2, 4), ("아래", 4, 6)], [("왼쪽", 0, 3), ("가운데", 3, 5), ("오른쪽", 5, 8)]


def next_spot(a):  # 빨간 칸이 가장 많이 남은 구역
    best = max(((int((a[r0:r1, c0:c1] == 0).sum()), rn, cn) for rn, r0, r1 in ROWS for cn, c0, c1 in COLS))
    if best[0] == 0: return "빈 곳 없음 — 기울인 장을 더"
    return "다음: 화면 " + ("가운데" if best[1] == best[2] == "가운데" else best[1] + " " + best[2])


def status():
    out = dict(mode=state["mode"], board=state["board"], paused=state["paused"], msg=state["msg"], count=count, goal=GOAL, hint={})
    for s in CAMS:
        d = det.get(s); why = d["why"] if fresh(d) else "none"
        if why == "ok" and hint[s] == "similar": why = "similar"
        if time.time() - state["flash"][s] < 2.0: why = "saved"
        col, txt = HINT_KO[why]
        if fresh(d) and why != "none": txt += "  · 지금 기울기 %.0f° (%s)" % (d["tilt"], d["tdir"])
        out["hint"][s] = dict(col=col, txt=txt, next=next_spot(cov["mono"][s]) if state["mode"] == "mono" else "")
    return out


PAGE = """<html><head><meta charset="utf-8"><title>캘리브레이션 촬영</title><style>
body{background:#1e1e1e;color:#eee;font-family:sans-serif;margin:10px}button{font-size:17px;margin:3px;padding:8px 14px;border-radius:6px;border:1px solid #888;background:#333;color:#eee}
button.on{background:#2a7;border-color:#2a7}#st{font-size:18px;line-height:1.6}.big{font-size:22px;font-weight:bold}
.hb{flex:1;padding:10px;border-radius:8px;font-size:22px;font-weight:bold;line-height:1.4}.hb small{font-size:17px;font-weight:normal}</style></head><body>
<div>단계: <button id="m_mono" onclick="s('mode=mono')">① 렌즈 (카메라 한 대씩)</button><button id="m_stereo" onclick="s('mode=stereo')">② 두 카메라 같이</button>
<button id="m_check" onclick="s('mode=check')">③ 큐브 확인</button> &nbsp; 판: <button id="b_20" onclick="s('board=20')">큰 판 2 cm</button><button id="b_10" onclick="s('board=10')">작은 판 1 cm</button>
&nbsp; <button id="pz" onclick="s('pause=1')">멈춤/계속</button> <button onclick="s('resetcov=1')">초록 칸 지우기</button> <button id="snd" onclick="beepOn()">🔔 소리·진동 켜기</button> <span id="sv" style="display:none"><input id="lb" placeholder="예: Z20_X0" size="10"> <button onclick="sv()">이 장면 저장</button></span></div>
<div id="st"></div><div style="display:flex;gap:8px;margin-top:6px"><div id="hl" class="hb"></div><div id="hr" class="hb"></div></div>
<img id="v" style="width:100%;margin-top:6px">
<script>
let ac=null,last=-1;function beepOn(){ac=new (window.AudioContext||window.webkitAudioContext)();beep();document.getElementById('snd').className='on'}
function beep(){if(navigator.vibrate)navigator.vibrate(150);if(!ac)return;const o=ac.createOscillator(),g=ac.createGain();o.frequency.value=1200;g.gain.value=0.3;o.connect(g);g.connect(ac.destination);o.start();o.stop(ac.currentTime+0.15)}
function s(q){fetch('/set?'+q).then(u)}
function sv(){fetch('/save?label='+encodeURIComponent(document.getElementById('lb').value)).then(u)}
function u(){fetch('/status').then(r=>r.json()).then(j=>{
 for(const k of ['mono','stereo','check'])document.getElementById('m_'+k).className=j.mode==k?'on':'';
 for(const k of ['20','10'])document.getElementById('b_'+k).className=j.board==k?'on':'';
 document.getElementById('pz').className=j.paused?'on':'';document.getElementById('sv').style.display=j.mode=='check'?'inline':'none';
 const c=j.count,g=j.goal;let t='';
 if(j.mode=='mono')t='<span class=big>왼쪽 '+c.mono.left+'/'+g.mono+' (가로 '+c.tilted.left['가로']+' · 세로 '+c.tilted.left['세로']+') · 오른쪽 '+c.mono.right+'/'+g.mono+' (가로 '+c.tilted.right['가로']+' · 세로 '+c.tilted.right['세로']+')</span><br>판을 <b>25~45° 기울인 것만</b> 저장. 좌우로 돌리기(가로)와 위아래로 젖히기(세로)를 반반. 판이 화면의 1/4 이상 차지하게 가까이. 빨간 칸 쪽으로 옮겨 가며.';
 if(j.mode=='stereo')t='<span class=big>쌍 '+(c.stereo['20']+c.stereo['10'])+'/'+g.stereo+' (큰 판 '+c.stereo['20']+' · 작은 판 '+c.stereo['10']+' · 가로 '+c.tilt_st['가로']+' · 세로 '+c.tilt_st['세로']+')</span><br>로봇 정면 가운데, <b>25~45° 기울여서</b>. 작은 판은 20~30 cm, 큰 판은 30~50 cm. 한 장마다 1초 멈춤.';
 if(j.mode=='check')t='<span class=big>확인 '+c.check+'/'+g.check+'</span><br>큐브를 테이프 자리에 놓고 이름(예: Z20_X0)을 적은 뒤 [이 장면 저장].';
 t+=(j.paused?' &nbsp; <b>(멈춤)</b>':'')+'<br>마지막: '+j.msg;
 document.getElementById('st').innerHTML=t;
 for(const [id,k,nm] of [['hl','left','왼쪽 카메라'],['hr','right','오른쪽 카메라']]){const h=j.hint[k],e=document.getElementById(id);e.style.background=h.col;e.innerHTML=nm+'<br>'+h.txt+(h.next?'<br><small>'+h.next+' (빨간 칸)</small>':'')}
 const tot=c.mono.left+c.mono.right+c.stereo['20']+c.stereo['10']+c.check;if(last>=0&&tot>last)beep();last=tot;})}
setInterval(u,400);u();
const v=document.getElementById('v');function next(){const im=new Image();im.onload=()=>{v.src=im.src;setTimeout(next,80)};im.onerror=()=>setTimeout(next,500);im.src='/frame.jpg?'+Date.now()}next();
</script></body></html>"""


class Hd(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def send(self, body, ctype):
        self.send_response(200); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path); q = {k: v[0] for k, v in parse_qs(u.query).items()}
        if u.path == "/": return self.send(PAGE.encode(), "text/html; charset=utf-8")
        if u.path == "/frame.jpg":
            ok, j = cv2.imencode(".jpg", np.hstack([panel("left"), panel("right")]), [cv2.IMWRITE_JPEG_QUALITY, 80])
            return self.send(j.tobytes(), "image/jpeg")
        if u.path == "/set":
            if q.get("mode") in ("mono", "stereo", "check"): state["mode"] = q["mode"]
            if q.get("board") in BOARDS: state["board"] = q["board"]
            if "pause" in q: state["paused"] = not state["paused"]
            if "resetcov" in q:  # 화면의 초록 칸 표시만 지움 (저장 사진·개수·비슷한 자세 판정은 그대로)
                for arr in (cov["mono"] if state["mode"] == "mono" else cov["stereo"][state["board"]]).values(): arr[:] = 0
            with open(UI, "w") as fh: json.dump(dict(mode=state["mode"], board=state["board"]), fh)
        if u.path == "/save" and state["mode"] == "check":
            L, R = latest.get("left"), latest.get("right")
            if L and R:
                with lock:
                    save_pair("check", dict(frame=L[0], t=L[1], n=L[2]), dict(frame=R[0], t=R[1], n=R[2]), q.get("label", ""))
        return self.send(json.dumps(status(), ensure_ascii=False, default=lambda o: o.item() if hasattr(o, "item") else str(o)).encode(), "application/json")


def metadata():
    os.makedirs(ROOT, exist_ok=True)
    ctrl = {}
    for s, dv in DEVS.items():
        try: ctrl[s] = subprocess.run(["v4l2-ctl", "-d", dv, "--list-ctrls"], capture_output=True, text=True, timeout=5).stdout
        except Exception as ex: ctrl[s] = "error: %s" % ex
    for _ in range(50):
        if len(size) == 2: break
        time.sleep(0.1)
    md = dict(started=datetime.datetime.now().isoformat(timespec="seconds"), devices=DEVS, usb_path=CAMS, image_size_wh=size,
              requested_wh=[W, H], fourcc="MJPG", opencv=cv2.__version__, pattern_inner_corners=list(PAT), boards_pitch_mm=BOARDS,
              thresholds=dict(still_px=STILL_PX, max_edge_px=MAX_EDGE, novel=NOVEL, sync_s=SYNC_S), v4l2_ctrls=ctrl)
    p = os.path.join(ROOT, "metadata_%s.json" % datetime.datetime.now().strftime("%H%M%S"))
    with open(p, "w") as fh: json.dump(md, fh, ensure_ascii=False, indent=1)
    print("session", ROOT, "size", size, flush=True)


for s in CAMS:
    threading.Thread(target=grab, args=(s,), daemon=True).start()
    threading.Thread(target=detect, args=(s,), daemon=True).start()
threading.Thread(target=saver, daemon=True).start()
metadata()
threading.Thread(target=reload, daemon=True).start()
threading.Thread(target=timelapse, daemon=True).start()
ThreadingHTTPServer(("0.0.0.0", 8090), Hd).serve_forever()
