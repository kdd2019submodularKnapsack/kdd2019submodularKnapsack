#pragma once

#include <algorithms/Algorithm.h>
#include <algorithms/Greedy.h>
#include <algorithms/MultiDimensional.h>
#include <algorithms/MarginalRatioThresholding.h>

template <typename T>
struct GeneralRobust : Algorithm<T> {
  Result<T> finish(const Result<T>& input, const set<T>& exclude, const Knapsack<T> &knapsack, double lowerBound, double upperBound, double eps) const override {
    cout.flush();
    auto elements = filter<T>(input.set, [&](const T& x) { return exclude.count(x) == 0; });
    Result<T> res =
        knapsack.K.size() <= 1
            ? Greedy<T>().solve(elements, knapsack, false)
            : MultiDimensional<T>(lowerBound, upperBound, eps).solve(elements, knapsack, false);
//            : MarginalRatioThresholding<T>(lowerBound, upperBound, eps).solve(elements, knapsack, false);
    res.oracleCalls += input.oracleCalls;
    knapsack.filterResult(res, exclude);
    return res;
  }
};