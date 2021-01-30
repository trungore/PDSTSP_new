#ifndef TIGERSUGAR_UTILITIES
#define TIGERSUGAR_UTILITIES

#include "testlib.h"

#include<bits/stdc++.h>
#define FOR(i, a, b) for (int i = (a), _b = (b); i <= _b; i++)
#define FORD(i, b, a) for (int i = (b), _a = (a); i >= _a; i--)
#define REP(i, n) for (int i = 0, _n = (n); i < _n; i++)
#define FORE(i, v) for (__typeof((v).begin()) i = (v).begin(); i != (v).end(); i++)
#define ALL(v) (v).begin(), (v).end()
#define IS_INF(x)   (std::isinf(x))
#define IS_NAN(x)   (std::isnan(x))
#define fi   first
#define se   second
#define MASK(i) (1LL << (i))
#define BIT(x, i) (((x) >> (i)) & 1)
#define div   ___div
#define prev   ___prev
#define left   ___left
#define right   ___right
#define __builtin_popcount __builtin_popcountll
using namespace std;
template<class X, class Y>
    bool minimize(X &x, const Y &y) {
        X eps = 1e-9;
        if (x > y + eps) {
            x = y;
            return true;
        } else return false;
    }
template<class X, class Y>
    bool maximize(X &x, const Y &y) {
        X eps = 1e-9;
        if (x + eps < y) {
            x = y;
            return true;
        } else return false;
    }
template<class T>
    T Abs(const T &x) {
        return (x < 0 ? -x : x);
    }

namespace tigersugar {

typedef double Distance;
const Distance INF_DISTANCE = (double) 1e9 + 7;

struct Instance {
    double time_limit;
    int numPoint, numDrone, numDroneEligible;
    vector<pair<int, double> > lsDroneEligible;
    vector<double> drone_cost; 
    vector<vector<double> > distance;

    Instance(int numPoint, int numDrone, int numDroneEligible) {
        this->numPoint = numPoint;
        this->numDrone = numDrone;
        this->numDroneEligible = numDroneEligible;

        drone_cost.resize(numPoint+1, 0);
        distance.resize(numPoint+1);
        FOR(i, 0, numPoint) distance[i].resize(numPoint+1, 0);
    }
};

struct Tour {
    vector<int> points; // this include the depot.

    Tour(const vector<int> &points = vector<int>()) {
        this->points = points;
    }

    void add(int x) {
        points.push_back(x);
    }

    int& operator [] (int x) {
        return points[x];
    }
    int operator [] (int x) const {
        return points[x];
    }

    bool empty(void) const {
        return points.empty();
    }

    int size(void) const {
        return points.size();
    }

    Distance distance(const Instance &instance) const {
        if (points.empty()) return 0;
        // Distance res = instance.distance[0][points.front()] + instance.distance[points.back()][0];
        Distance res = 0;
        FOR(i, 0, (int)points.size()-2) res += instance.distance[points[i]][points[i + 1]];
        return res;
    }

    void reverse(void) {
        std::reverse(ALL(points));
    }

    void debug(const Instance &instance) {
        cerr << "VEHICLE TOUR: ";
        for (int x : points) cerr << x << " ";
        cerr << '\n';
        cerr << "VEHILCE COST: " << this->distance(instance) << "\n\n";
    }
};

struct Drone_Tour {
    vector<vector<int> > node;

    vector<int> operator [] (int x) const {
        return node[x];
    }

    bool empty(void) const {
        return node.empty();
    }

    int size(void) const {
        return node.size();
    }

    void add_drone() {
        node.push_back(vector<int>());
    }

    Distance distance(const Instance &instance) {
        Distance res = 0;
        for (vector<int> drone : node) {
            Distance val = 0;
            for (int x : drone) val += instance.drone_cost[x];
            maximize(res, val);
        }
        return res;
    }

    int numPointAssigned() {
        int res = 0;
        for (vector<int> vec : node) res += vec.size();
        return res;
    }

    void debug(const Instance &instance) {
        cerr << "DRONE TOUR: \n";
        REP(i, node.size()) {
            cerr << "drone " << i << ": ";
            for (int x : node[i]) cerr << x << " ";
            cerr << '\n'; 
        }
        cerr << "DRONE COST: " << this->distance(instance) << "\n\n";
    }
};

struct Solution {
    vector<Tour> tours;

    void add(const Tour &tour) {
        tours.push_back(tour);
    }

    Tour& operator [] (int x) {
        return tours[x];
    }
    Tour operator [] (int x) const {
        return tours[x];
    }

    bool empty(void) const {
        return tours.empty();
    }

    int size(void) const {
        return tours.size();
    }

    Distance distance(const Instance &instance) const {
        Distance res = 0;
        FORE(it, tours) res += it->distance(instance);
        return res;
    }

    // void print(void) const {
    //     REP(i, tours.size()) tours[i].print(format("Route #%d", i + 1));
    // }

    // void verify(const Instance &instance) const {
    //     set<int> used;
    //     FORE(it, tours) {
    //         it->verify(instance);
    //         FORE(jt, it->points) ensuref(used.insert(*jt).se, "%d is visited in two different tours.", *jt);
    //     }
    //     FOR(i, 1, instance.numPoint)
    //         ensuref(used.find(i) != used.end(), "%d has not been visited.", i);
    // }
};

} // tigersugar

#endif // TIGERSUGAR_UTILITIES
