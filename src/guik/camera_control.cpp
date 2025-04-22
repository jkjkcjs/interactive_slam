 @@ -1,12 +1,3 @@
#include <guik/camera_control.hpp>

#include <memory>
#include <GL/gl3w.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>


namespace guik {

ArcCameraControl::ArcCameraControl() : center(0.0f, 0.0f, 0.0f), distance(10.0f), left_button_down(false), theta(0.0f), phi(-60.0f * M_PI / 180.0f) {
  left_button_down = false;
  middle_button_down = false;
}

ArcCameraControl::~ArcCameraControl() {}

// 鼠标输入触发函数
void ArcCameraControl::mouse(const Eigen::Vector2i& p, int button, bool down) {
  if (button == 0) {
    left_button_down = down;  // 左键是否按下
  }
  if (button == 2) {
    middle_button_down = down;  // 右键是否按下
  }
  drag_last_pos = p;  // 更新上一次鼠标位置
}

// 鼠标拖动函数
void ArcCameraControl::drag(const Eigen::Vector2i& p, int button) {
  Eigen::Vector2i rel = p - drag_last_pos;

  if (left_button_down) {  // 左键按下
    theta -= rel[0] * 0.01f;  // 左右旋转角度
    phi -= rel[1] * 0.01f;     // 
    // 限制俯仰角度在 -90°到90°之间
    phi = std::min(M_PI_2 - 0.01, std::max(-M_PI_2 + 0.01, phi));  
  }

  if (middle_button_down) {  // 中键按下
    center += Eigen::AngleAxisf(theta + M_PI_2, Eigen::Vector3f::UnitZ()) * Eigen::Vector3f(-rel[0], rel[1], 0.0f) * distance * 0.001f;
  }

  drag_last_pos = p;  // 更新拖动位置
}

// 鼠标滚动函数
void ArcCameraControl::scroll(const Eigen::Vector2f& rel) {
  if (rel[0] > 0) {
    distance = distance * 0.9f; // 鼠标上滚放大
  } else if (rel[0] < 0) {
    distance = distance * 1.1f; // 鼠标下滚缩小
  }

  distance = std::max(0.1, distance);  // 距离最小为0.1
}

// 获取相机的旋转
Eigen::Quaternionf ArcCameraControl::rotation() const { 
  // 返回以 Z 轴和 Y 轴为轴的旋转矩阵
  return Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY()); 
}

// 获取相机的观察矩阵
Eigen::Matrix4f ArcCameraControl::view_matrix() const {
  // 根据相机位置和观察点生成观察矩阵
  Eigen::Vector3f offset = rotation() * Eigen::Vector3f(distance, 0.0f, 0.0f);
  Eigen::Vector3f eye = center + offset;

  glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center[0], center[1], center[2]), glm::vec3(0.0f, 0.0f, 1.0f));
  return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat));  // 返回 Map 后的 OpenGL 观察矩阵
}

}  // namespace guik
