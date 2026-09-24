// Renders synthetic depth images from the husky camera pose and checks the guard's depth logic:
// flat ground passes, a low log / a hiker / a drop-off stop it, a mushroom does not.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "skidsteer_controller/depth_check.hpp"

using depth_check::Geometry;

struct Box { double x0, x1, y0, y1, z0, z1; };

static Geometry camera(double x, double z, double pitch)
{
  const double c = std::cos(pitch), s = std::sin(pitch);
  return Geometry{{c, 0, s, 0, 1, 0, -s, 0, c}, {x, 0.0, z}, 2.0944, 0.1323};
}

// depth along camera x for each pixel; ground only where x < ground_end
static std::vector<float> render(const Geometry & g, const std::vector<Box> & boxes, double ground_end = 1e9, double k = 0.0)
{
  const int W = 720, H = 480;
  const double f = (W / 2.0) / std::tan(g.horizontal_fov / 2.0), cx = (W - 1) / 2.0, cy = (H - 1) / 2.0;
  std::vector<float> img(W * H, INFINITY);
  for (int v = 0; v < H; ++v) {
    for (int u = 0; u < W; ++u) {
      const double c[3] = {1.0, -(u - cx) / f, -(v - cy) / f};
      double r[3];
      for (int i = 0; i < 3; ++i) {
        r[i] = g.R[3 * i] * c[0] + g.R[3 * i + 1] * c[1] + g.R[3 * i + 2] * c[2];
      }
      double best = INFINITY;
      // ground z = -base_height + k * x (k = 0: the wheel plane)
      const double nr = r[2] - k * r[0];
      if (nr < 0) {
        const double t = (-g.base_height + k * g.t[0] - g.t[2]) / nr;
        if (g.t[0] + t * r[0] < ground_end) {best = t;}
      }
      for (const Box & b : boxes) {  // slab test
        double lo = 0, hi = INFINITY;
        const double o[3] = {g.t[0], g.t[1], g.t[2]}, mn[3] = {b.x0, b.y0, b.z0}, mx[3] = {b.x1, b.y1, b.z1};
        for (int i = 0; i < 3 && lo <= hi; ++i) {
          if (std::fabs(r[i]) < 1e-12) {
            if (o[i] < mn[i] || o[i] > mx[i]) {hi = -1;}
            continue;
          }
          double a = (mn[i] - o[i]) / r[i], z = (mx[i] - o[i]) / r[i];
          if (a > z) {std::swap(a, z);}
          lo = std::max(lo, a); hi = std::min(hi, z);
        }
        if (lo <= hi && lo > 0 && lo < best) {best = lo;}
      }
      img[v * W + u] = static_cast<float>(best);
    }
  }
  return img;
}

static depth_check::Result run(const Geometry & g, const std::vector<Box> & boxes, double ground_end = 1e9, double k = 0.0)
{
  auto img = render(g, boxes, ground_end, k);
  return depth_check::check(reinterpret_cast<const uint8_t *>(img.data()), 720, 480, 720 * sizeof(float), g);
}

int main()
{
  const double gz = -0.1323;  // wheel plane in base_link
  const Box log{1.60, 1.90, -1.3, 1.3, gz, gz + 0.30};        // 0.85 m ahead of the front
  const Box hiker{2.20, 2.50, -0.3, 0.3, gz, gz + 1.70};      // 1.45 m ahead
  const Box mushroom{1.60, 1.70, -0.05, 0.05, gz, gz + 0.12};
  const Box far_log{4.0, 4.3, -1.3, 1.3, gz, gz + 0.30};  // 3.25 m past the front

  // as mounted today, a level camera, and a camera moved low to the front of the gripper
  const Geometry cams[] = {camera(0.45, 0.25, 0.4363), camera(0.45, 0.25, 0.0), camera(0.80, 0.02, 0.4363)};
  for (const Geometry & g : cams) {
    auto flat = run(g, {});
    assert(!std::isfinite(flat.obstacle_dist) && !flat.cliff && flat.points.empty());
    auto l = run(g, {log});
    assert(std::fabs(l.obstacle_dist - 0.85) < 0.1);
    assert(!l.points.empty());
    for (const auto & p : l.points) {assert(p[0] > 1.55 && p[0] < 1.95 && p[2] > gz + 0.15);}  // on the log, not the ground
    auto h = run(g, {hiker});
    assert(h.obstacle_dist < 1.6 && h.obstacle_dist > 1.3);
    auto mu = run(g, {mushroom});
    assert(!std::isfinite(mu.obstacle_dist) && mu.points.empty());
    auto fl = run(g, {far_log});
    assert(fl.obstacle_dist > 3.0 && fl.points.empty());  // beyond mark_range: stop check sees it, costmap does not
    // ground rising ahead at 10 deg (edge of a hollow): not an obstacle; a log on that slope still is
    const double k = std::tan(10 * M_PI / 180);
    auto up = run(g, {}, 1e9, k);
    assert(!std::isfinite(up.obstacle_dist) && up.points.empty());
    const Box slog{1.60, 1.90, -1.3, 1.3, gz, gz + 1.9 * k + 0.30};
    assert(std::fabs(run(g, {slog}, 1e9, k).obstacle_dist - 0.85) < 0.1);
    auto drop = run(g, {}, 1.0);
    assert(drop.cliff && !std::isfinite(drop.obstacle_dist));
    std::printf("cam x=%.2f z=%.2f: log %.2f m, hiker %.2f m, flat clear, mushroom ignored, drop-off flagged\n",
      g.t[0], g.t[2], l.obstacle_dist, h.obstacle_dist);
  }
  std::printf("depth_check: all passed\n");
  return 0;
}
