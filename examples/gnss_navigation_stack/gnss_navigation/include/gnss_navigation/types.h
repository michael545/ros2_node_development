#pragma once

#include <stdint.h>

#include <memory>
#include <vector>
#include <array>
#include <deque>
#include <list>
#include <map>
#include <set>

#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

namespace Eigen {

typedef Matrix<int16_t,  3, 1> Vector3s;
typedef Matrix<uint32_t, 3, 1> Vector3ui;
typedef Matrix<int64_t,  3, 1> Vector3l;
typedef Matrix<uint64_t, 3, 1> Vector3ul;
typedef Matrix<double,   6, 1> Vector6d;

typedef Matrix<double, 6, 6> Matrix6d;

typedef std::deque<Vector2d, aligned_allocator<Vector2d>>  VectorVector2d;
typedef std::deque<Vector3d, aligned_allocator<Vector3d>>  VectorVector3d;
typedef std::deque<Vector3i, aligned_allocator<Vector3i>>  VectorVector3i;
typedef std::deque<Vector3ui,aligned_allocator<Vector3ui>> VectorVector3ui;
typedef std::deque<VectorXd, aligned_allocator<VectorXd>>  VectorVectorXd;

}

namespace gnss_navigation {

template<class T>
using List = std::deque<T>;

template<class T>
using LinkedList = std::list<T>;

template<class T>
using DynamicArray = std::vector<T>;

template<class T, std::size_t N>
using Array = std::array<T, N>;

template<class Key, class T>
using Dictionary = std::map<Key, T>;

using namespace Eigen;

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

} // namespace gnss_navigation
