import csv, math, subprocess, sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
OUT = sys.argv[1]; W = 'simple_trees'
def svc(s, req, rt='ignition.msgs.Boolean', q='ignition.msgs.EntityFactory'):
    return subprocess.run(['ign', 'service', '-s', f'/world/{W}/{s}', '--reqtype', q, '--reptype', rt, '--timeout', '4000', '--req', req], capture_output=True, text=True).stdout
def box(name, x, y, sx, sy, sz):
    g = f'<box><size>{sx} {sy} {sz}</size></box>'
    return f"<sdf version='1.8'><model name='{name}'><static>true</static><pose>{x} {y} {sz/2} 0 0 0</pose><link name='l'><collision name='c'><geometry>{g}</geometry></collision><visual name='v'><geometry>{g}</geometry></visual></link></model></sdf>"
def pole(name, x, y, r, h):
    g = f'<cylinder><radius>{r}</radius><length>{h}</length></cylinder>'
    return f"<sdf version='1.8'><model name='{name}'><static>true</static><pose>{x} {y} {h/2} 0 0 0</pose><link name='l'><collision name='c'><geometry>{g}</geometry></collision><visual name='v'><geometry>{g}</geometry></visual></link></model></sdf>"
# (name, y, sdf, nearest-surface distance from a point)
def d_rect(x, y, cx, cy, hx, hy): return math.hypot(max(abs(x - cx) - hx, 0), max(abs(y - cy) - hy, 0))
LANES = [('wall', -3.0, box('t_wall', 2.5, -3.0, 0.4, 3.0, 1.2), lambda x, y: d_rect(x, y, 2.5, -3.0, 0.2, 1.5)),
         ('pole', -6.0, pole('t_pole', 2.5, -6.0, 0.15, 1.5), lambda x, y: max(math.hypot(x - 2.5, y + 6.0) - 0.15, 0)),
         ('low_box', 5.0, box('t_low', 2.5, 5.0, 0.5, 0.5, 0.3), lambda x, y: d_rect(x, y, 2.5, 5.0, 0.25, 0.25))]
for n_, y, s, _f in LANES:
    print(n_, svc('create', 'sdf: "' + s.replace('"', '\\"') + '"').strip()[:20], flush=True)
class N(Node):
    def __init__(s):
        super().__init__('r12_test'); s.x = s.y = None
        s.create_subscription(Odometry, '/husky1/odometry', s.cb, Q)
        s.raw = s.create_publisher(Twist, '/husky1/cmd_vel_raw', 10); s.nav = s.create_publisher(Twist, '/husky1/cmd_vel_nav_guarded', 10)
        s.hb = s.create_publisher(Bool, '/husky1/teleop_active', 10)
    def cb(s, m): s.x, s.y = m.pose.pose.position.x, m.pose.pose.position.y
rclpy.init(); n = N()
def spin(t):
    e = time.time() + t
    while time.time() < e:
        n.hb.publish(Bool(data=True)); rclpy.spin_once(n, timeout_sec=0.05)
def tp(x, y):
    q = f'name: "husky1" position {{x: {x} y: {y} z: 0.2}} orientation {{x: 0 y: 0 z: 0 w: 1}}'
    subprocess.run(['ign', 'service', '-s', f'/world/{W}/set_pose', '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', q], capture_output=True)
spin(3); print('odom', n.x, n.y, flush=True)
w = csv.writer(open(OUT + '/r12.csv', 'w', buffering=1)); w.writerow('obstacle,path,speed,rep,final_x,final_y,min_base_gap,final_base_gap,peak_v,seconds'.split(','))
for name, y, _s, dist in LANES:
    for path, pub in (('raw', n.raw), ('nav', n.nav)):
        for v in (0.5, 1.0):
            for rep in (1, 2):
                pub.publish(Twist()); tp(-3.5, y); spin(2.5)
                t = Twist(); t.linear.x = v; t0 = time.time(); lx, ly, lt = n.x, n.y, t0; peak = 0; px = n.x; pt = t0; mn = 99
                while time.time() - t0 < 20:
                    pub.publish(t); spin(0.05); now = time.time(); mn = min(mn, dist(n.x, n.y))
                    if now - pt > 0.4: peak = max(peak, (n.x - px) / (now - pt)); px, pt = n.x, now
                    if math.hypot(n.x - lx, n.y - ly) > 0.03: lx, ly, lt = n.x, n.y, now
                    if now - lt > 3 and now - t0 > 4: break
                pub.publish(Twist()); fg = dist(n.x, n.y)
                w.writerow([name, path, v, rep, round(n.x, 3), round(n.y, 3), round(mn, 3), round(fg, 3), round(peak, 2), round(time.time() - t0, 1)])
                print(name, path, v, rep, 'min', round(mn, 2), 'end', round(fg, 2), 'at', round(n.x, 2), round(n.y, 2), 'peak', round(peak, 2), flush=True)
print('DONE', flush=True)
