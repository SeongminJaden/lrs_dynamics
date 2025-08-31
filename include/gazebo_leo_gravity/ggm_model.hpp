#ifndef GGM_MODEL_HPP_
#define GGM_MODEL_HPP_

#include <string>
#include <vector>
#include <ignition/math/Vector3.hh>  // 필요한 헤더 포함

namespace gazebo_leo_gravity
{

struct Coefficient
{
  int n, m;
  double C, S;
};

class GGMModel
{
public:
  // 파일 로드 함수
  bool load(const std::string& filename, int nmax);

  // 중력 가속도 계산 함수
  ignition::math::Vector3d acceleration(const ignition::math::Vector3d& pos) const;

private:
  int nmax_;
  double GM_ = 0.0;
  double a_ = 0.0;
  bool normalized_ = false;

  std::vector<Coefficient> coeffs_;
};

} // namespace gazebo_leo_gravity

#endif  // GGM_MODEL_HPP_

