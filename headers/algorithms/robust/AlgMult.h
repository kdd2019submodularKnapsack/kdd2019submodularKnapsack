#pragma once

#pragma once

#include "algorithms/Algorithm.h"
#include "GeneralRobust.h"

template<typename T>
struct AlgMult : GeneralRobust<T> {
  const double lowerBound;
  const double upperBound;
  const double eps;
  const double m;

  AlgMult(double lowerBound, double upperBound, double eps, double m) :
    lowerBound(lowerBound), upperBound(upperBound), eps(eps), m(m) {
    assert(lowerBound > 1e-9);
    assert(!isinf(upperBound));
    assert(eps > 1e-9);
  }

  string name() const {
    return "AlgMult";
  }

  Result<T> algorithmWithOptEstimation(double approxObj, const vector<T>& input, const Knapsack<T> &knapsack, bool prune) const {
    assert(approxObj >= 0);
    double K = knapsack.K[0];
    for (double k : knapsack.K) {
      assert(k == K);
    }
    int d = knapsack.constraintNum;
    int w = max(1, (int)ceil(4 * log2(2 * K) * m / K));
    int eight = 8;
    int ten = 10;
    int iUppedBound = (int)ceil(log2(2 * K));
//    cout << "l = " << iUppedBound << endl;
    vector<vector<Result<T>>> B(iUppedBound + 1);
    vector<vector<double>> space(iUppedBound + 1);
    for (int i = 0; i < iUppedBound + 1; ++i) {
      int bound = w * (int)ceil(K / (1 << i))  +  eight * iUppedBound;
      B[i] = vector<Result<T>>();
      for (int j = 0; j < bound + 1; ++j) {
        B[i].emplace_back(Result<T>(d));
      }
      space[i] = vector<double>(d);
    }
    map<T, double> maxCost;

    int oracleCalls = 0;
    for (const T& e : input) {
      vector<double> costs = knapsack.costs(e);
      double c = *max_element(costs.begin(), costs.end());
      maxCost[e] = c;
      for (int i = 0; i < iUppedBound + 1; ++i) {
        double maxAllowedCost = min((double)(1 << i), K);
        if (c > maxAllowedCost) {
          continue;
        }
        for (int j = 1; j < (int)B[i].size(); ++j) {
          if (existsIndex(d, [&](int a) { return B[i][j].c[a] + costs[a] >= 2 * maxAllowedCost; })) {
            continue;
          }
          oracleCalls++;
          double dif = knapsack.dif(B[i][j].set, e, B[i][j].objective);
          if (dif * (1 + 2 * d) * maxAllowedCost < c * approxObj) {
            continue;
          }
          knapsack.addToResult(B[i][j], e, dif);
          for (int a = 0; a < d; ++a) {
            space[i][a] += eight * iUppedBound * knapsack.c(a, e);
          }
          auto totalSize = [&]() { return vector_sum<int>(B[i], [](const auto& res) { return res.set.size(); }); };
          auto moreSpace = [&]() { return existsIndex(d, [&](int a) { return space[i][a] >= maxAllowedCost; }); };
          while (moreSpace() && totalSize() < ten * w * maxAllowedCost) {
            B[i].emplace_back(Result<T>(d));
            for (int a = 0; a < d; ++a) {
              space[i][a] = max(space[i][a] - maxAllowedCost, 0.);
            }
          }
          // Go to next elements in the stream
          goto goToNextElement;
        }
      }
      goToNextElement:;
    }
/*

    cout << "Bucket sizes: " << endl;
    int total = 0;
    for (const auto& Bi : B) {
      int sum = 0;
      for (const auto& res : Bi) {
        cout << res.set.size() << " ";
        sum += res.set.size();
      }
      cout << "| sum = " << sum << endl;
      total += sum;
    }
    cout << total << endl << endl;
*/

    vector<T> resSet;
    for (const auto& Bi : B) {
      for (const auto& res : Bi) {
        addRange<T>(resSet, res.set);
      }
    }
    removeDuplicates(resSet);
    if (prune) {
      sortBy(resSet, [&](const T &x) { return maxCost[x]; });
      return algorithmWithOptEstimation(approxObj, resSet, knapsack, false);
    } else {
      return Result<T>(resSet, oracleCalls, -1, vector<double>(d));
    }
  }

  Result<T> solveBeforeRemovals(const vector<T>& input, const Knapsack<T> &knapsack) const {
    int oracleCalls = 0;
    vector<T> elements;
    cout << "v = ";
    for (double v = lowerBound; v < upperBound * (1 + eps); v *= (1 + eps)) {
      cout << v << ", ";
      cout.flush();
      auto newRes = algorithmWithOptEstimation(v, input, knapsack, true);
      if (newRes.set.size() == 0) {
        break;
      }
      addRange(elements, newRes.set);
      oracleCalls += newRes.oracleCalls;
    }
    cout << endl;
    removeDuplicates(elements);
    Result<T> res(elements, oracleCalls, -1, vector<double>(knapsack.constraintNum));
    knapsack.recalculate(res);
    return res;
  }

};

