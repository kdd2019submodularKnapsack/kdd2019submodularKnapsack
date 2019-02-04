#pragma once

#include <iostream>
#include "utils.h"
#include <sstream>

#include "algorithms/MarginalRatioThresholding.h"
#include "algorithms/Greedy.h"
#include "algorithms/robust/AlgMult.h"
#include "algorithms/MultiDimensional.h"

#include "Graph.h"
#include "inputs/CoverageKnapsack.h"
#include "inputs/NonrobustKiller.h"
#include "Dataset.h"

Result<int> runAlgorithmTestRemovals(const vector<int>& input, const Knapsack<int>& knapsack, const Algorithm<int>& algorithm) {
  return measureTime<Result<int>>("Algorithm " + algorithm.name(), [&]() {
    auto res = algorithm.solveBeforeRemovals(input, knapsack);
    cout << "Oracle calls: " << res.oracleCalls << endl;
    cout << "Objective before removals: " << res.objective << endl;
    cout << "Costs: " << vectorToStr(res.c, " ") << endl;
    cout << endl;
    cout << "Elements before removals: " << res.set.size() << endl;
//    cout << vectorToStr(res.set, " ") << endl;
    return res;
  });
}

void testWithRemovals() {
  int d = 2;
  for (double k : { 1.5, 5., 10., 20. }) {
//    Graph g = Graph::read("../data/graphs/wiki-vote.txt");
//    Graph g = Graph::read("../data/graphs/twitter.txt");
    vector<double> K(d, k);
    Knapsack<int>* knapsack;
    int size;
    double lowerBound, upperBound;
    double eps = 0.5;
    Dataset dataset = NONROBUST_KILLER;
    switch (dataset) {
      case COVERAGE: {
        string folder = "../data/graphs/";
        Graph g = Graph::read(folder + "facebook.txt");
        knapsack = new CoverageKnapsack(K, g);
        if (d == 1) {
          lowerBound = 0.1;
          upperBound = 0.7;
        } else {
          lowerBound = 0.05;
          upperBound = 0.3;
        }
        size = (int) g.vertices.size();;
        break;
      }
      case MOVIES: {
        string folder = "../data/ml-latest-small/";
        auto movies = readCsv<Movie>(folder + "/movies.csv", Movie::parse);
        auto ratings = readCsv<Rating>(folder + "/ratings.csv", Rating::parse);
        sort(begin(movies), end(movies), [](const Movie &a, const Movie &b) { return a.movie < b.movie; });
        int maxUser = 0;
        map<int, int> movieMap;
        for (int i = 0; i < (int) movies.size(); ++i) {
          movieMap[movies[i].movie] = i;
          movies[i].movie = i;
        }
        for (Rating &rating : ratings) {
          rating.user--;
          rating.movie = movieMap[rating.movie];
          maxUser = max(maxUser, rating.user);
        }
        int userCount = maxUser + 1;
        cout << "Movie count = " << movies.size() << "; User count = " << userCount << "; Ratings count = "
             << ratings.size() << ";\n";
//      vector<Genres> genres = { Genres({"Comedy", "Horror"}, {"Adventure", "Action"})};
        vector<Genres> genres = {Genres({"Comedy", "Horror"}, {"Adventure", "Action"}),
                                 Genres({"Drama", "Romance"}, {"Sci-Fy", "Fantasy"})};
        for (int i = 0; i < (int) genres.size(); ++i) {
          cout << "K = " << K[i] << "; bad = (" << vectorToStr(genres[i].bad, ", ");
          cout << "); good = (" << vectorToStr(genres[i].good, ", ") << ")\n";
        }
        knapsack = new SubtractAverageKnapsack(movies, ratings, genres, K, range(movies.size()));
        lowerBound = 1;
        upperBound = 7;
        size = movies.size();
        break;
      }
      case NONROBUST_KILLER: {
        size = 1000;
        lowerBound = upperBound = 100;
        knapsack = new NonrobustKiller(K, size);
        break;
      }
    }
    for (double m : {0, 5, 10, 15, 20, 25}) {
      vector<int> input = range(size);
      shuffle(input.begin(), input.end(), generator);
//      cout << "Obj: " << MultiDimensional<int>(lowerBound, upperBound, eps).solve(input, *knapsack, true).objective << endl;


      cout << "lower bound = " << lowerBound
           << "; upper bound = " << upperBound
           << "; eps = " << eps
           << "; K = " << vectorToStr(K, ", ")
           << "; M = " << m
           << "; d = " << d
           << ";\n\n";
      auto alg = AlgMult<int>(lowerBound, upperBound, eps, m);
      Result<int> result = runAlgorithmTestRemovals(input, *knapsack, alg);
      cout << endl;
//      auto greedy = Greedy<int>();

      cout << std::fixed;
      cout << "m = " << m << endl;
      cout << "Objective after removal of returned elements" << endl;
      cout << "    AlgMult,   Greedy(on entire set after removals),   OPT upper bound" << endl;
      set<int> exclude;
      cout << endl;
      while (true) {
        cout.precision(5);
        vector<int> nextExclude;
        vector<int> remaining = {};
        for (int x : input) {
          if (exclude.count(x) == 0) {
            remaining.push_back(x);
          }
        }
        auto res = alg.finish(result, exclude, *knapsack, lowerBound, upperBound, eps);
        cout << "    " << res.objective << " ";
        Result<int> rem(d);
        rem.set = remaining;
        auto greedySolution = alg.finish(rem, {}, * knapsack, lowerBound, upperBound, eps);
//        auto greedySolution = greedy.solve(remaining, *knapsack, false);
        double approx = d == 1 ? 1 - pow(exp(1), - greedySolution.c[0] / k) : 1. / (1 + 2. * d);
        cout << greedySolution.objective << " " << greedySolution.objective / approx << endl;
        cout.flush();
        addRange(nextExclude, res.set);
        if (nextExclude.empty()) {
          break;
        }
        for (int x : nextExclude) {
          exclude.insert(x);
        }
      }
      cout << endl;
    }
    delete knapsack;
  }
  // I don't delete pointers, but who cares
}
