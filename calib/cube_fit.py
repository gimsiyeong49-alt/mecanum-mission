"""큐브 위치·각도: 5 cm 정육면체 모형을 두 카메라 화면에 그려 실제 빨간 영역과 가장 잘 겹치는 자세를 찾음.
면 고르기·경계 찾기·각도별 갈래 없이 한 가지 방법 (캘리브레이션 인수인계 8장 '알려진 큐브 모델 공동 피팅'과 같은 생각).
찾는 값: 리그 M 좌표(mm, x 오른쪽 y 아래 z 앞)의 큐브 가운데 x·y·z 와 세로축 회전 yaw (큐브는 90° 마다 같으므로 −45~45°).
믿을지: 두 카메라 겹침(IoU)이 둘 다 --min-iou 이상.
(10-29 이전 방식 = cube_face.py 직선·면 후보·짝짓기 → 30° 넘게 돌면 가운데가 1.3~2 cm 흔들려 바꿈)"""
import math
import cv2, numpy as np
from cube_face import mask_candidates, LIGHT_STEPS

SIDE = 50.0
_V = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) * SIDE / 2
SCALE = 0.5  # 겹침 계산 해상도 (1280×720 → 640×360)


class Fitter:
    def __init__(self, calib):
        c = calib
        self.KL, self.DL, self.KR, self.DR = c["K_L"], c["D_L"], c["K_R"], c["D_R"]
        self.R, self.T, self.CM, self.A = c["R_LR"], c["T_LR"].reshape(3), c["C_M_in_L"].reshape(3), c["A_ML"]
        self.rvR = cv2.Rodrigues(self.R)[0]

    def project(self, P_M):  # M 좌표 점들 → (왼쪽 픽셀, 오른쪽 픽셀)
        XL = (self.A.T @ P_M.T).T + self.CM
        pl = cv2.projectPoints(XL, np.zeros(3), np.zeros(3), self.KL, self.DL)[0].reshape(-1, 2)
        pr = cv2.projectPoints(XL, self.rvR, self.T, self.KR, self.DR)[0].reshape(-1, 2)
        return pl, pr

    @staticmethod
    def verts(p):  # p = (x, y, z, yaw°)
        th = math.radians(p[3]); c, s = math.cos(th), math.sin(th)
        Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        return (Ry @ _V.T).T + np.array(p[:3])

    def score(self, p, ML, MR, box):
        pl, pr = self.project(self.verts(p))
        ious = []
        for pts, M, (x0, y0, x1, y1) in ((pl, ML, box[0]), (pr, MR, box[1])):
            r = np.zeros((y1 - y0, x1 - x0), np.uint8)
            q = (pts * SCALE - [x0, y0]).astype(np.int32)
            cv2.fillConvexPoly(r, cv2.convexHull(q), 1)
            m = M[y0:y1, x0:x1]
            inter = np.count_nonzero(r & m); uni = np.count_nonzero(r | m)
            ious.append(inter / uni if uni else 0.0)
        return min(ious), ious

    def fit(self, img_l, img_r, min_iou=0.85, init=None, fast=False):
        """init = 직전 자세(x, y, z, yaw°)가 있으면 각도 훑기 없이 거기서 작은 걸음으로 이어 찾음 (실시간 추적용, 계산 몇 분의 1)
        빨간 덩어리가 여럿이면(주황 테이프 등) 큰 것부터 짝지어 맞춰 보고 겹침 기준을 넘는 첫 짝을 씀 (실시간 추적은 가장 큰 것만)."""
        best = None
        for step in range(len(LIGHT_STEPS) if not fast else 1):        # 조명 단계: 평소(0)에서 못 맞으면 보정·채도 낮춤 차례로
            cl, cr = mask_candidates(img_l, small=fast, step=step), mask_candidates(img_r, small=fast, step=step)
            if not cl or not cr: continue
            if fast: return self._fit_masks(cl[0], cr[0], None, None, min_iou, init, True)
            for i, j in sorted(((i, j) for i in range(min(3, len(cl))) for j in range(min(3, len(cr)))), key=lambda t: t[0] + t[1]):
                r = self._fit_masks(None, None, cl[i], cr[j], min_iou, init, False)
                if r is not None and r.get("ok"):
                    r["light_step"] = step; return r
                if best is None or (r is not None and min(r["iou"]) > min(best["iou"])): best = r
        return best

    def _fit_masks(self, ML, MR, mL, mR, min_iou, init, fast):
        for m, k_ in ((ML, 2), (MR, 2)) if fast else ((mL, 4), (mR, 4)):  # 화면 테두리에 닿아 잘린 큐브는 모형이 억지로 맞음(10-29: 잘린 채 겹침 0.88, 각도 −29°) → 안 믿음
            ys, xs = np.nonzero(m)
            if xs.min() <= k_ - 1 or ys.min() <= k_ - 1 or xs.max() >= m.shape[1] - k_ or ys.max() >= m.shape[0] - k_:
                return dict(ok=False, cut=True, iou=[0.0, 0.0], p=[0, 0, 0, 0])
        if not fast:
            ML = cv2.resize(mL, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)
            MR = cv2.resize(mR, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)
        H, W = ML.shape
        box = []
        for M in (ML, MR):  # 겹침은 빨간 영역 둘레(여유 40 px)만 계산
            ys, xs = np.nonzero(M)
            box.append((max(0, xs.min() - 40), max(0, ys.min() - 40), min(W, xs.max() + 40), min(H, ys.max() + 40)))
        if init is not None:
            p = list(init); s_, _ = self.score(p, ML, MR, box); steps = [3.0, 3.0, 3.0, 3.0]
            return self._refine(p, s_, steps, ML, MR, box, min_iou)
        # 시작값: 두 화면 빨간 영역 무게중심을 삼각측량 (대략 몇 cm 안)
        if fast: cl = np.array(np.nonzero(ML)[::-1], float).mean(1) / SCALE; cr = np.array(np.nonzero(MR)[::-1], float).mean(1) / SCALE
        else: cl = np.array(np.nonzero(mL)[::-1], float).mean(1); cr = np.array(np.nonzero(mR)[::-1], float).mean(1)
        uL = cv2.undistortPoints(cl.reshape(1, 1, 2), self.KL, self.DL).reshape(2, 1)
        uR = cv2.undistortPoints(cr.reshape(1, 1, 2), self.KR, self.DR).reshape(2, 1)
        X = cv2.triangulatePoints(np.hstack([np.eye(3), np.zeros((3, 1))]), np.hstack([self.R, self.T.reshape(3, 1)]), uL, uR)
        P0 = self.A @ ((X[:3, 0] / X[3, 0]) - self.CM)
        coarse = []
        for yaw in range(-45, 45, 9):  # 각도는 거칠게 다 훑고
            p = [P0[0], P0[1], P0[2] + SIDE / 2, float(yaw)]
            coarse.append((self.score(p, ML, MR, box)[0], p))
        # 상위 3개 각도를 각각 끝까지 다듬어 가장 잘 맞는 것 (10-29: 위치 고정한 거친 비교만으로 하나 고르면 36° 큐브를
        # 0° 쪽 덜 맞는 봉우리(겹침 0.93 < 0.97)로 잡아 '반듯하다'로 판단 → 대각선으로 물음)
        # 각도 후보마다 위치만 대략(2 mm 까지) 맞춘 점수로 비교 → 상위 2개를 끝까지 (위치 고정 비교는 36° 를 놓침)
        rough = []
        for s_, p in coarse:
            q, sq = self._climb(list(p), s_, [8.0, 8.0, 8.0, 0.0], ML, MR, box, stop=2.0)
            rough.append((sq, q))
        rough.sort(key=lambda c_: -c_[0])
        outs = [self._refine(list(p), s_, [4.0, 4.0, 4.0, 6.0], ML, MR, box, min_iou) for s_, p in rough[:2]]
        return max(outs, key=lambda o: min(o["iou"]))

    def _climb(self, p, s_, steps, ML, MR, box, stop=0.4):
        steps = list(steps)
        while max(steps[:3]) > stop:
            improved = False
            for i in range(4):
                if steps[i] <= 0: continue
                for sgn in (1, -1):
                    q = list(p); q[i] += sgn * steps[i]
                    if i == 3: q[3] = (q[3] + 45) % 90 - 45
                    sq, _ = self.score(q, ML, MR, box)
                    if sq > s_: s_, p, improved = sq, q, True
            if not improved: steps = [x / 2 for x in steps]
        return p, s_

    def _refine(self, p, s_, steps, ML, MR, box, min_iou):
        # x, y, z(mm), yaw(°) — 좌표 하나씩 밀어 보며 좁힘
        while max(steps[:3]) > 0.4:
            improved = False
            for i in range(4):
                for sgn in (1, -1):
                    q = list(p); q[i] += sgn * steps[i]
                    if i == 3: q[3] = (q[3] + 45) % 90 - 45
                    sq, _ = self.score(q, ML, MR, box)
                    if sq > s_: s_, p, improved = sq, q, True
            if not improved: steps = [x / 2 for x in steps]
        s_, ious = self.score(p, ML, MR, box)
        if s_ < min_iou: return dict(ok=False, iou=ious, p=p)
        pl, pr = self.project(self.verts(p))
        return dict(ok=True, M=np.array(p[:3]), yaw=float(p[3]), iou=ious,
                    hull_l=cv2.convexHull(pl.astype(np.float32)).reshape(-1, 2), hull_r=cv2.convexHull(pr.astype(np.float32)).reshape(-1, 2))
