#pragma once

#include "Graph.h"
#include "utils.h"
#include "inputs/Knapsack.h"

struct CoverageKnapsack : Knapsack<int> {
  const Graph graph;
  vector<vector<float>> cost;
  mutable unordered_map<int64, vector<int>> mem;
  mutable set<int64> was;
  mutable long totalSize;
  const int n;

  const int64 emptySetKey = 5;

  CoverageKnapsack(const vector<double> &K, const Graph &graph) : Knapsack(K), graph(graph), n((int)graph.vertices.size()) {
    cost = vector<vector<float>>(K.size(), vector<float>(n));
    cout << "Costs:" << endl;
    for (int c = 0; c < (int) K.size(); ++c) {
      for (int i = 0; i < n; ++i) {
//        cost[c][i] = Rand::next(3) + 1;
        cost[c][i] = (float)Rand::nextRand() * 2 + 1;
        cout << cost[c][i] << " ";
      }
      cout << endl;
    }
    mem[emptySetKey] = vector<int>();
    totalSize = 0;
  }

  double c(int i, const int &e) const override {
    return cost[i][e];
  }

  void checkMem() const {
    if (totalSize > 4e9) {
      cerr << "clear ";
      mem.clear();
      mem[emptySetKey] = vector<int>();
      totalSize = 0;
    }
  }

  int bitNum(const vector<bool>& mask) const {
    int res = 0;
//#pragma omp parallel reduction (+:res)
    for (int i = 0; i < n; ++i) {
      if (mask[i]) {
        res ++;
      }
    }
    return res;
  }

  double compute(vector<int> &z, bool remember) const {
    if (z.empty()) {
      return 0;
    }
    int64 mul = 1343157;
    // without last element
    int64 prevKey = emptySetKey;
    for (int i = 0; i < (int) z.size() - 1; ++i) {
      prevKey = prevKey * mul + z[i];
    }
    int lastVertex = z[(int) z.size() - 1];
    int64 key = prevKey * mul + lastVertex;
    if (mem.count(key) > 0) {
      return (double)mem[key].size() / n;
    }
    if (mem.count(prevKey) == 0) {
      z.pop_back();
      compute(z, true);
      z.push_back(lastVertex);
    }
//    assert(mem.count(prevKey) > 0);
    const auto& edges = graph.vertices[lastVertex].edges;
    if (remember || was.count(key) > 0) {
      was.erase(key);
      auto newMask = sortedUnion(mem[prevKey], edges);
      checkMem();
      mem[key] = newMask;
      int cnt = (int)newMask.size();
      totalSize += cnt * 4 + 30;
      assert(cnt <= n);
      return (double)cnt / n;
    } else {
      was.insert(key);
      return (double)unionSize(mem[prevKey], edges) / n;
    }
  }

  double f(vector<int> &z) const override {
    return compute(z, false);
  }
};