#include "quat.h"

#include <cmath>

Quat::Quat() : Quat(0, 0, 0, 1) {}

Quat::Quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

void Quat::normalize() {
  if (x == 0 && y == 0 && z == 0 && w == 0) {
    w = 1;
  }
  const auto norm = std::pow(x * x + y * y + z * z + w * w, 0.5f);
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
}

Quat Quat::conjugate() const {
  return Quat(-x, -y, -z, w);
}

Quat Quat::product(const Quat& q) const {
  return Quat(
    q.w * x - q.z * y + q.y * z + q.x * w,
    q.z * x + q.w * y - q.x * z + q.y * w,
    -q.y * x + q.x * y + q.w * z + q.z * w,
    -q.x * x - q.y * y - q.z * z + q.w * w
  );
}

float Quat::getYaw() const { // y p r
  return std::atan2(2 * (x * y + z * w) , -1 + 2 * (x * x + w * w));
}
