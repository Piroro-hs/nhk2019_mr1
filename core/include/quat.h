#if !defined(__QUAT__H__)
#define __QUAT__H__

class Quat {
  private:
    float x;
    float y;
    float z;
    float w;
  public:
    Quat();
    Quat(float x, float y, float z, float w);
    void normalize();
    Quat conjugate() const;
    Quat product(const Quat& q) const;
    float getYaw() const;
};

#endif // __QUAT__H__
