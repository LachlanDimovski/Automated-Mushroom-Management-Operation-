import json, math, os, re, subprocess, sys, time, csv
import numpy as np, rclpy, cv2
from PIL import Image as PI
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import Image
from ultralytics import YOLO
REPO, OUT = sys.argv[1], sys.argv[2]; W = 'river_moderate_aus_dt2'
meta = json.load(open(f'{REPO}/forest_worlds/worlds/{W}.meta.json'))
td = f'{REPO}/forest_worlds/world_assets/river_moderate_aus_terrain'
t = open(td + '/model.sdf').read()
sx, sy, sz = map(float, re.search(r'<heightmap>.*?<size>([^<]+)</size>', t, re.S).group(1).split())
px, py, pz = map(float, re.search(r'<heightmap>.*?<pos>([^<]+)</pos>', t, re.S).group(1).split())
img = np.asarray(PI.open(td + '/heightmap.png').convert('L'), float) / 255 * sz + pz
def tz(x, y):
    n, m = img.shape; fx = min(max((x - px + sx / 2) / sx * (m - 1), 0), m - 1); fy = min(max((sy / 2 - (y - py)) / sy * (n - 1), 0), n - 1)
    return float(img[int(round(fy)), int(round(fx))])
objs = [(o['x'], o['y'], o.get('radius', 0.5)) for o in meta.get('objects', []) if 'x' in o]
river = np.array(meta['river']['line'], float)
def river_d(x, y): return float(np.min(np.hypot(river[:, 0] - x, river[:, 1] - y)))
fords = meta.get('fords', [])
def blocked(ax, ay, bx, by):
    for ox, oy, r in objs:
        dx, dy = bx - ax, by - ay; L = dx * dx + dy * dy; u = max(0, min(1, ((ox - ax) * dx + (oy - ay) * dy) / L))
        if math.hypot(ax + u * dx - ox, ay + u * dy - oy) < r + 0.05: return True
    return False
class N(Node):
    def __init__(s):
        super().__init__('r1_test'); s.f = None; s.t = 0
        s.create_subscription(Image, 'husky1/camera/image', s.cb, Q)
    def cb(s, m):
        s.f = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1).copy(); s.t = time.time()
rclpy.init(); n = N(); model = YOLO(f'{REPO}/perception/perception/models/variants/amanita_river_v6_yolo11n.pt' if os.path.exists(f'{REPO}/perception/perception/models/variants/amanita_river_v6_yolo11n.pt') else f'{REPO}/perception/models/variants/amanita_river_v6_yolo11n.pt')
def spin(sec):
    e = time.time() + sec
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.05)
spin(5); print('camera', n.f is not None, n.f.shape if n.f is not None else None, flush=True)
def tp(x, y, z, yaw):
    q = f'name: "husky1" position {{x: {x} y: {y} z: {z}}} orientation {{x: 0 y: 0 z: {math.sin(yaw/2)} w: {math.cos(yaw/2)}}}'
    return subprocess.run(['ign', 'service', '-s', f'/world/{W}/set_pose', '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', q], capture_output=True, text=True).stdout
w = csv.writer(open(OUT + '/r1.csv', 'w', buffering=1)); w.writerow('mushroom,range,angle,x,y,blocked,best_conf,cx,cy,bw,bh,native_conf,nboxes'.split(','))
done = skipped = 0
for mu in meta['mushrooms']:
    for r in (1.0, 2.0, 3.0, 4.0):
        for k in range(6):
            a = k * math.pi / 3; d = r + 0.45
            x, y = mu['x'] + d * math.cos(a), mu['y'] + d * math.sin(a)
            if max(abs(x), abs(y)) > 26 or river_d(x, y) < 2.0 or any(math.hypot(x - ox, y - oy) < ro + 1.0 for ox, oy, ro in objs): skipped += 1; continue
            yaw = math.atan2(mu['y'] - y, mu['x'] - x); tp(x, y, tz(x, y) + 0.35, yaw); spin(1.6); t0 = time.time()
            while n.t < t0: spin(0.05)
            fr = np.ascontiguousarray(n.f[:, :, :3][:, :, ::-1]); b = model.predict(fr, imgsz=640, conf=0.25, agnostic_nms=True, verbose=False)[0].boxes
            best = (0, 0, 0, 0, 0); nat = 0
            for i in range(len(b)):
                c = model.names[int(b.cls[i])]; cf = float(b.conf[i]); cx, cy, bw, bh = (float(v) for v in b.xywh[i])
                if c.startswith('amanita') and abs(cx - fr.shape[1] / 2) <= 90 and cf > best[0]: best = (cf, cx, cy, bw, bh)
                if c.startswith('native') and abs(cx - fr.shape[1] / 2) <= 90: nat = max(nat, cf)
            bl = blocked(x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw), mu['x'], mu['y'])
            w.writerow([mu['name'], r, k, round(x, 2), round(y, 2), int(bl), round(best[0], 3), *[round(v, 1) for v in best[1:]], round(nat, 3), len(b)]); done += 1
            if done % 7 == 0: cv2.imwrite(f'{OUT}/frames/{mu["name"]}_{int(r)}m_{k}.jpg', fr)
            if done % 20 == 0: print(done, skipped, mu['name'], r, flush=True)
print('DONE', done, 'skipped', skipped, flush=True)
