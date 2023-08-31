/**
* @Function: Common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <Eigen/Core>
#include <deque>
#include <vector>
#include <glog/logging.h>

namespace gici {
  
// Math common ----------------------------------------------------
// Default precision for float type check
#define DEFAULT_PRECISION 1.0e-4

// Check equal for float types
template<typename FloatT>
inline bool checkEqual(FloatT x, FloatT y, 
                       float precision = DEFAULT_PRECISION) {
  return (fabs(x - y) < precision);
}

// Check float type equals zero
template<typename FloatT>
inline bool checkZero(FloatT x, 
                      float precision = DEFAULT_PRECISION) {
  return checkEqual<FloatT>(x, 0.0, precision);
}

// Check equal for float matrix
template<typename FloatT, int Rows, int Cols>
inline bool checkEqual(Eigen::Matrix<FloatT, Rows, Cols> mat_x,
                       Eigen::Matrix<FloatT, Rows, Cols> mat_y,
                       float precision = DEFAULT_PRECISION) {
  bool has_none_equal = false;
  for (size_t i = 0; i < mat_x.rows(); i++) {
    for (size_t j = 0; j < mat_x.cols(); j++) {
      if (!checkEqual(mat_x(i, j), mat_y(i, j), precision)) {
        has_none_equal = true; break;
      }
    }
  }
  return !has_none_equal;
}

// Check less than and equal to for float types
template<typename FloatT>
inline bool checkLessEqual(FloatT x, FloatT y, 
                       float precision = DEFAULT_PRECISION) {
  return (x <= (y + precision));
}

// Check larger than and equal to for float types
template<typename FloatT>
inline bool checkLargerEqual(FloatT x, FloatT y, 
                       float precision = DEFAULT_PRECISION) {
  return (x >= (y - precision));
}

// Check in bound
template<typename FloatT>
inline bool checkInBound(FloatT x, FloatT min, FloatT max,
                       float precision = DEFAULT_PRECISION) {
  return checkLargerEqual(x, min, precision) && 
         checkLessEqual(x, max, precision);
}

// Check float matrix equals zero
template<typename FloatT, int Rows, int Cols>
inline bool checkZero(Eigen::Matrix<FloatT, Rows, Cols> mat, 
                      float precision = DEFAULT_PRECISION) {
  Eigen::Matrix<FloatT, Rows, Cols> mat_y;
  mat_y.setZero();
  return checkEqual(mat, mat_y, precision);
}

// Square
template<typename T>
inline T square(T x) {
  return (x * x);
}

// Parameter wise square for Eigen
template<typename FloatT, int Rows, int Cols>
inline Eigen::Matrix<FloatT, Rows, Cols> 
  cwiseSquare(Eigen::Matrix<FloatT, Rows, Cols> mat) {
  Eigen::Matrix<FloatT, Rows, Cols> out;
  for (size_t i = 0; i < mat.rows(); i++) {
    for (size_t j = 0; j < mat.cols(); j++) {
      out(i, j) = square(mat(i, j));
    }
  }
  return out;
}

// Operation common -----------------------------------------------
// Get current variable from deque
template<typename T>
inline T& getCurrent(std::deque<T>& seq) {
  CHECK(seq.size() > 0);
  return seq.back();
}

// Get last variable from deque
template<typename T>
inline T& getLast(std::deque<T>& seq) {
  CHECK(seq.size() > 1);
  const size_t seq_size = seq.size();
  return seq.at(seq_size - 2);
}

// Get oldest (first) variable from deque
template<typename T>
inline T& getOldest(std::deque<T>& seq) {
  CHECK(seq.size() > 0);
  return seq.front();
}

// Push a batch of data back for deque
template<typename T>
inline void pushBatchBack(std::deque<T>& seq, const std::deque<T>& add) {
  for (size_t i = 0; i < add.size(); i++) {
    seq.push_back(add[i]);
  }
}

// Get current variable from vector
template<typename T>
inline T& getCurrent(std::vector<T>& seq) {
  CHECK(seq.size() > 0);
  return seq.back();
}

// Get last variable from vector
template<typename T>
inline T& getLast(std::vector<T>& seq) {
  CHECK(seq.size() > 1);
  const size_t seq_size = seq.size();
  return seq.at(seq_size - 2);
}

// Get oldest (first) variable from vector
template<typename T>
inline T& getOldest(std::vector<T>& seq) {
  CHECK(seq.size() > 0);
  return seq.front();
}

// Push a batch of data back for vector
template<typename T>
inline void pushBatchBack(std::vector<T>& seq, const std::vector<T>& add) {
  for (size_t i = 0; i < add.size(); i++) {
    seq.push_back(add[i]);
  }
}

// Get median value
template<typename T>
inline T getMedian(std::vector<T>& seq) {
  if (seq.size() == 0) return static_cast<T>(0);
  std::vector<T> temp = seq;
  typename std::vector<T>::iterator it = temp.begin()+std::floor(temp.size() / 2);
  std::nth_element(temp.begin(), it, temp.end());
  return *it;
}

// Get average value
template<typename T>
inline double getAverage(std::vector<T>& seq) {
  if (seq.size() == 0) return 0.0;
  double sum = 0.0;
  for (size_t i = 0; i < seq.size(); i++) {
    sum += static_cast<double>(seq[i]);
  }
  return sum / static_cast<double>(seq.size());
}

// Get standard deviation value
template<typename T>
inline double getStandardDeviation(std::vector<T>& seq, const T& ref) {
  if (seq.size() == 0) return 0.0;
  double sum2 = 0.0;
  for (size_t i = 0; i < seq.size(); i++) {
    sum2 += square(seq[i] - ref);
  }
  return sqrt(sum2 / static_cast<double>(seq.size()));
}

// Compute x/y
template<typename T>
inline double getDivide(T x, T y) {
  return static_cast<double>(x) / static_cast<double>(y);
}

}

