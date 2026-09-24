#ifndef SKIDSTEER_CONTROLLER_DEPTH_CHECK_HPP
#define SKIDSTEER_CONTROLLER_DEPTH_CHECK_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

// Depth-image obstacle and cliff check in the robot frame. Every pixel is
// projected to a 3D point in base_link using the camera's pose (from TF), so
// the result is independent of where the camera sits or how far it is tilted.
// Heights are measured from a ground plane fitted to the points ahead, so a
// slope or the rim of a hollow reads as ground, not as an obstacle.
namespace depth_check
{

struct Geometry
{
  double R[9];                 // base_link <- camera_link rotation, row-major
  double t[3];                 // camera_link origin in base_link
  double horizontal_fov;       // rad
  double base_height;          // base_link above the ground under the wheels
};

struct Limits
{
  double min_obstacle_height = 0.15;  // above the wheel plane; mushrooms and grass sit below it
  double max_obstacle_height = 1.2;   // canopy above the robot is not in the way
  double self_front_x = 0.75;         // chassis + folded gripper end here (Nav2 footprint front is 0.7)
  double corridor_half_width = 0.5;   // footprint half-width 0.35 + margin
  double cliff_near_x = 1.2;          // floor is expected between these base_link x...
  double cliff_far_x = 1.8;
  double cliff_drop = 0.25;           // ...and a point this far below the wheel plane means no floor
  int min_pixels = 20;                // noise floor for both checks
  double ground_fit_range = 3.0;      // ground plane fitted to points this far past the front
  double max_ground_slope = 0.36;     // tan(20 deg); a steeper fit is distrusted -> wheel plane
  double mark_range = 3.0;            // points published for Nav2's costmap only this far past the front:
                                      // (= ground_fit_range: heights past the fitted plane are not trusted)
};

struct Result
{
  float obstacle_dist = std::numeric_limits<float>::infinity();  // ahead of self_front_x, m
  bool cliff = false;
  int floor_pixels = 0;   // cliff-band pixels that saw a floor
  int band_pixels = 0;    // cliff-band pixels (ray would hit flat ground in the band)
  std::vector<std::array<float, 3>> points;  // obstacle points in base_link (any bearing), for the costmap
  double ground_slope = 0.0;                 // fitted ground gradient ahead (0 = wheel plane)
};

// depth: 32FC1 metres along the camera's forward (x) axis, Gazebo camera_link
// convention (x forward, y left, z up). step is the row stride in bytes.
inline Result check(
  const uint8_t * data, uint32_t width, uint32_t height, uint32_t step,
  const Geometry & g, const Limits & lim = Limits(), uint32_t stride = 2)
{
  Result res;
  const double f = (width / 2.0) / std::tan(g.horizontal_fov / 2.0);
  const double cx = (width - 1) / 2.0, cy = (height - 1) / 2.0;
  const double ground = -g.base_height;
  std::vector<std::array<double, 3>> pts;  // every valid point ahead of the robot, base_link

  for (uint32_t v = 0; v < height; v += stride) {
    const uint8_t * row = data + static_cast<size_t>(v) * step;
    for (uint32_t u = 0; u < width; u += stride) {
      // ray for unit forward depth, camera frame -> base_link
      const double c[3] = {1.0, -(u - cx) / f, -(v - cy) / f};
      double r[3];
      for (int i = 0; i < 3; ++i) {
        r[i] = g.R[3 * i] * c[0] + g.R[3 * i + 1] * c[1] + g.R[3 * i + 2] * c[2];
      }
      float d;
      std::memcpy(&d, row + static_cast<size_t>(u) * sizeof(float), sizeof(float));
      const bool valid = std::isfinite(d) && d > 0.0f;

      // cliff band: rays that would meet the wheel plane just ahead of the robot
      if (r[2] < 0.0) {
        const double s = (ground - g.t[2]) / r[2];
        const double gx = g.t[0] + s * r[0], gy = g.t[1] + s * r[1];
        if (gx > lim.cliff_near_x && gx < lim.cliff_far_x && std::fabs(gy) < lim.corridor_half_width) {
          ++res.band_pixels;
          if (valid && g.t[2] + d * r[2] > ground - lim.cliff_drop) {
            ++res.floor_pixels;
          }
        }
      }
      if (valid) {
        const double px = g.t[0] + d * r[0];
        if (px > lim.self_front_x) {
          pts.push_back({px, g.t[1] + d * r[1], g.t[2] + d * r[2]});
        }
      }
    }
  }

  // Ground plane z = a x + b y + c, least squares over low points just ahead,
  // refitted without what sticks up. Slopes and hollows become the reference;
  // a log or trunk stands off it. Falls back to the wheel plane when unsure.
  double a = 0.0, b = 0.0, c0 = ground;
  for (int iter = 0; iter < 3; ++iter) {
    double n = 0, sx = 0, sy = 0, sz = 0, sxx = 0, sxy = 0, syy = 0, sxz = 0, syz = 0;
    for (const auto & p : pts) {
      const double dz = p[2] - (a * p[0] + b * p[1] + c0);
      if (p[0] - lim.self_front_x > lim.ground_fit_range || std::fabs(dz) > (iter == 0 ? 0.35 : 0.08)) {
        continue;
      }
      n += 1; sx += p[0]; sy += p[1]; sz += p[2];
      sxx += p[0] * p[0]; sxy += p[0] * p[1]; syy += p[1] * p[1]; sxz += p[0] * p[2]; syz += p[1] * p[2];
    }
    // normal equations [sxx sxy sx; sxy syy sy; sx sy n] [a b c] = [sxz syz sz], Cramer's rule
    const double det = sxx * (syy * n - sy * sy) - sxy * (sxy * n - sy * sx) + sx * (sxy * sy - syy * sx);
    if (n < 50 || std::fabs(det) < 1e-9) {
      break;
    }
    const double na = (sxz * (syy * n - sy * sy) - sxy * (syz * n - sy * sz) + sx * (syz * sy - syy * sz)) / det;
    const double nb = (sxx * (syz * n - sz * sy) - sxz * (sxy * n - sy * sx) + sx * (sxy * sz - syz * sx)) / det;
    const double nc = (sxx * (syy * sz - sy * syz) - sxy * (sxy * sz - sy * sxz) + sx * (sxy * syz - syy * sxz)) / det;
    if (std::fabs(na) > lim.max_ground_slope || std::fabs(nb) > lim.max_ground_slope) {
      a = b = 0.0; c0 = ground;
      break;
    }
    a = na; b = nb; c0 = nc;
  }
  res.ground_slope = std::hypot(a, b);

  std::vector<float> hits;
  for (const auto & p : pts) {
    const double h = p[2] - (a * p[0] + b * p[1] + c0);
    if (h <= lim.min_obstacle_height || h >= lim.max_obstacle_height) {
      continue;
    }
    if (std::fabs(p[1]) < lim.corridor_half_width) {
      hits.push_back(static_cast<float>(p[0] - lim.self_front_x));
    }
    if (p[0] - lim.self_front_x < lim.mark_range) {
      res.points.push_back({static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])});
    }
  }

  if (static_cast<int>(res.points.size()) < lim.min_pixels) {
    res.points.clear();  // a few stray pixels are noise, not an obstacle to map
  }
  if (static_cast<int>(hits.size()) >= lim.min_pixels) {
    // nearest distance that at least min_pixels agree on, not a single noisy pixel
    std::nth_element(hits.begin(), hits.begin() + (lim.min_pixels - 1), hits.end());
    res.obstacle_dist = hits[lim.min_pixels - 1];
  }
  res.cliff = res.band_pixels >= lim.min_pixels && res.floor_pixels < res.band_pixels / 2;
  return res;
}

}  // namespace depth_check

#endif  // SKIDSTEER_CONTROLLER_DEPTH_CHECK_HPP
