#pragma once

#include <inputs/DelegateKnapsack.h>
#include "GeneralRobust.h"

template <typename T>
struct MakeRobust : GeneralRobust<T> {
  Algorithm<T>* algorithm;
  const double mul;

  string name() const {
    return algorithm->name() + "(r)";
  }

  MakeRobust(Algorithm<T>* algorithm, double mul) : algorithm(algorithm), mul(mul) {}

  Result<T> solveBeforeRemovals(const vector<T>& input, const Knapsack<T>& knapsack) const {
    DelegateKnapsack<T> delegateKnapsack(vector_map<double, double>(knapsack.K, [=](double k) { return k * mul;}), knapsack);
    return algorithm->solveBeforeRemovals(input, delegateKnapsack);
  }

  virtual ~MakeRobust() {
    delete algorithm;
  }
};
