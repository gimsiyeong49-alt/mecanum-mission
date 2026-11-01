"""빨간 큐브 면 후보 찾기 (cube_locate.py · cube_grab.py 공용).
렌즈가 큐브 윗면 높이(4.7 cm)라 윗면은 안 보이고, 실루엣 = 앞면 + (옆면 하나). 윗변·왼변·오른변·아랫변(V 자 두 줄)을
각각 직선으로 맞춰 교점으로 꼭짓점을 구하고, 앞면/옆면 경계 후보를 여러 개 만든다. 고르기는 두 카메라 3D 정사각형 검사가 함.
(10-29: ① 외곽을 꼭짓점 5~6개로 줄이면 둥근 모서리 때문에 꼭짓점이 70 px 빗나감 ② 밝기 경계는 나뭇결에 속음 ③ '넓은 면 = 앞면' 은 비스듬하면 틀림)
이전 판 = cube_face_v1_poly.py"""
import cv2, numpy as np


def _candidates(m, min_area):
    """빨간 덩어리 중 큐브처럼 뭉툭한 것만, 큰 순서로. 길쭉하거나(가로세로 3 넘음) 감싸는 상자를 45% 미만 채우는
    덩어리(주황 테이프 띠·대각선 줄)는 뺌 (10-30: 2025 대회장 바닥 주황 테이프 — 화면을 가로지르는 띠가 큐브보다 커서 '못 찾음')."""
    def blobs(mm):
        n, lab, st, _ = cv2.connectedComponentsWithStats(mm)
        return [((lab == k).astype(np.uint8), st[k]) for k in range(1, n) if st[k, cv2.CC_STAT_AREA] >= min_area]

    def compact(st):
        a, w, h = st[cv2.CC_STAT_AREA], st[cv2.CC_STAT_WIDTH], st[cv2.CC_STAT_HEIGHT]
        return max(w, h) <= 3.0 * min(w, h) and a >= 0.45 * w * h

    out = []
    for comp, st in blobs(m):
        if compact(st):
            out.append((st[cv2.CC_STAT_AREA], comp)); continue
        # 뭉툭하지 않은 덩어리: 큐브에 가는 띠가 붙은 것일 수 있음 (렌즈가 큐브 윗면 높이라 먼 바닥 테이프가 큐브 윗변에 붙어 보임)
        # → 띠 두께보다 큰 원판으로 깎아 띠를 떼고, 남은 덩어리를 원래 영역 안에서 되살려 다시 검사 (큐브 70~110 px, 띠 10~28 px)
        for r in (8, 16):
            core = cv2.morphologyEx(comp, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
            found = False
            for sub, _ in blobs(core):
                rec = cv2.dilate(sub, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))) & comp   # 깎인 모서리 되살림
                n2, _, st2, _ = cv2.connectedComponentsWithStats(rec)
                if n2 >= 2 and compact(st2[1 + int(np.argmax(st2[1:, cv2.CC_STAT_AREA]))]):
                    out.append((int(rec.sum()), rec)); found = True
            if found: break
    out.sort(key=lambda t: -t[0])
    return [c for _, c in out]


def _norm(img):
    """조명 맞추기: 경기장 바닥·상자는 무채색 회색(규정 RGB 127) → 화면을 색별로 나눠 회색이 되게(화이트 밸런스) + 밝기 중앙값을 맞춤.
    큐브(빨강)는 화면의 작은 부분이라 평균에 거의 영향 없음 (10-30 조명 시험: 누런 조명 4/12, 흐린 빛 2/12 → 보정)."""
    f = img.astype(np.float32)
    med = np.median(f.reshape(-1, 3), axis=0) + 1.0                 # 색별 중앙값 (큐브·밝은 점에 덜 흔들림)
    g = med.mean()
    f *= (g / med)[None, None, :]                                     # 회색으로 맞춤
    v = np.median(f.max(axis=2))
    f *= 145.0 / max(v, 1.0)                                          # 바닥 밝기 → 145 (연구실 잡기 사진 중앙값 145, 범위 104~155)
    return np.clip(f, 0, 255).astype(np.uint8)


def _red(img, kern, s_min=120, norm=False):
    hsv = cv2.cvtColor(_norm(img) if norm else img, cv2.COLOR_BGR2HSV)
    m = (((hsv[..., 0] < 15) | (hsv[..., 0] > 165)) & (hsv[..., 1] > s_min) & (hsv[..., 2] > 60)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((kern, kern), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((kern, kern), np.uint8))


LIGHT_STEPS = [(120, False), (120, True), (90, True), (60, True)]   # (채도 기준, 조명 보정) — 앞에서 못 찾으면 다음 단계


def mask_candidates(img, small=False, step=0):
    """큐브 후보 마스크들 (큰 순서). small = 절반 크기 사진에서 (실시간 추적용, 마스크도 절반 크기).
    step = LIGHT_STEPS 단계: 0 = 예전 그대로, 1 = 조명 보정(회색 맞춤·밝기), 2·3 = 채도 기준 낮춤 (흐린 빛).
    평소엔 0 에서 찾고, 모형이 안 맞을 때만 cube_fit 이 다음 단계로 (10-30: 보정을 늘 쓰면 연구실처럼 색 있는 배경에서 평소 결과가 바뀜)."""
    s_min, norm = LIGHT_STEPS[step]
    if small:
        return _candidates(_red(cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA), 3, s_min, norm), 500)
    return _candidates(_red(img, 5, s_min, norm), 2000)


def mask_of(img):
    c = mask_candidates(img)
    return c[0] if c else None


def mask_small(img):
    """mask_of 를 절반 크기 사진에서 (실시간 추적용, 4배 빠름). 돌려주는 마스크는 절반 크기."""
    c = mask_candidates(img, small=True)
    return c[0] if c else None


def _fit(xs, ys):  # 강건한 직선 맞추기 → (점, 방향)
    P = np.column_stack([xs, ys]).astype(np.float32)
    if len(P) < 5: return None
    vx, vy, x0, y0 = cv2.fitLine(P, cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    return np.array([x0, y0], float), np.array([vx, vy], float)


def _meet(l1, l2):
    if l1 is None or l2 is None: return None
    (p1, d1), (p2, d2) = l1, l2
    M = np.array([d1, -d2]).T
    if abs(np.linalg.det(M)) < 1e-6: return None
    return p1 + np.linalg.solve(M, p2 - p1)[0] * d1


def faces(img):
    """면 후보 목록 [(이름, 4점)]. 4점 = 왼위·오위·오아래·왼아래. 이름: full = 실루엣 전체(옆면이 아주 얇을 때),
    left/right = 앞쪽 세로 모서리의 왼쪽 면/오른쪽 면 (두 카메라 모두 같은 모서리를 보므로 같은 이름 = 같은 실제 면)."""
    mk = mask_of(img)
    if mk is None: return []
    ys, xs = np.nonzero(mk)
    cols = np.arange(xs.min(), xs.max() + 1); rows = np.arange(ys.min(), ys.max() + 1)
    W, H = len(cols), len(rows)
    if W < 20 or H < 20: return []
    sub = mk[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1]
    top = rows[0] + np.argmax(sub, axis=0).astype(float)
    bot = rows[0] + (H - 1 - np.argmax(sub[::-1], axis=0)).astype(float)
    lef = cols[0] + np.argmax(sub, axis=1).astype(float)
    rig = cols[0] + (W - 1 - np.argmax(sub[:, ::-1], axis=1)).astype(float)
    cm, rm = slice(int(W * .15), int(W * .85)), slice(int(H * .2), int(H * .8))
    Lt = _fit(cols[cm], top[cm]); Ll = _fit(lef[rm], rows[rm]); Lr = _fit(rig[rm], rows[rm])
    if None in (Lt, Ll, Lr): return []
    tl, tr = _meet(Lt, Ll), _meet(Lt, Lr)
    ib = int(np.argmax(bot)); bm = np.array([cols[ib], bot[ib]], float)
    m4 = max(3, int(W * .04))
    sl, sr = slice(int(W * .08), ib - m4), slice(ib + m4, int(W * .92))
    Lbl = _fit(cols[sl], bot[sl]) if sl.stop - sl.start >= 5 else None
    Lbr = _fit(cols[sr], bot[sr]) if sr.stop - sr.start >= 5 else None
    if Lbl is None: Lbl = Lbr
    if Lbr is None: Lbr = Lbl
    if Lbl is None: return []
    bl, br = _meet(Lbl, Ll), _meet(Lbr, Lr)
    v = _meet(Lbl, Lbr)
    if v is not None and cols[0] <= v[0] <= cols[-1]: bm = v
    if any(p is None for p in (tl, tr, bl, br)): return []
    cands = [("full", np.array([tl, tr, br, bl]))]
    # 앞면/옆면 경계(bm 위로) 후보 방향: 왼변 평행, 오른변 평행, 수직, 밝기 변화
    dirs = [Ll[1], Lr[1], np.array([0.0, 1.0])]
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32), (7, 7), 0)
    gx = np.abs(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)); pts = [bm]
    ytop = 0.5 * (tl[1] + tr[1])
    for f in (0.25, 0.45, 0.65):
        y = int(bm[1] + (ytop - bm[1]) * f)
        x0, x1 = max(int(bm[0] - 0.3 * W), 0), min(int(bm[0] + 0.3 * W), mk.shape[1] - 1)
        row = gx[y, x0:x1] * (mk[y, x0:x1] > 0)
        if row.size and row.max() > 0: pts.append(np.array([x0 + int(np.argmax(row)), y], float))
    if len(pts) >= 3:
        d = _fit(np.array(pts)[:, 0], np.array(pts)[:, 1])
        if d is not None: dirs.append(d[1])
    for d in dirs:
        tm = _meet((bm, d), Lt)
        if tm is None or not (tl[0] - 2 < tm[0] < tr[0] + 2): continue
        cands.append(("left", np.array([tl, tm, bm, bl]))); cands.append(("right", np.array([tm, tr, br, bm])))
    return cands


def face(img):  # 그림 표시용: 넓은 후보
    f = [q for _, q in faces(img)]
    if not f: return None
    return max(f[1:] or f, key=lambda q: abs(q[1][0] - q[0][0]))
