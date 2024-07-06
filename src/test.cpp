#include <cstddef>
#include <print>
#include <string>
#include <numbers>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/ext/vector_double3.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>



int main() {
    auto up = glm::dvec3(0, 0, 1);
    auto v = glm::dvec3(1, 1, -1);
    auto w = glm::proj(v, up);

    std::println("w: ({} {} {})", w.x, w.y, w.z);
}