#include "paper/utilities.cpp"
#include "problem.cpp"

namespace Input_Adaptation {
    tigersugar::Instance Process() {
        tigersugar::Instance instance(Problem::n, Problem::m, Problem::nD);
        instance.time_limit = Problem::time_limit;

        REP(i, Problem::dCost.size()) {
            instance.drone_cost[i] = Problem::dCost[i];
            if (instance.drone_cost[i] == Constant::INF) instance.drone_cost[i] = 0;
            else instance.lsDroneEligible.push_back( make_pair(i, instance.drone_cost[i]) );
        }

        REP(i, Problem::vCost.size()) REP(j, Problem::vCost[i].size()) 
            instance.distance[i][j] = Problem::vCost[i][j];

        sort(instance.lsDroneEligible.begin(), instance.lsDroneEligible.end(), [] (pair<int, double> u, pair<int, double> v){
            return u.second > v.second;
        });

        return instance;
    }
}