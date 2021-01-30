#include "utilities.cpp"

namespace pms_optimizer {
    void optimizeTour(const tigersugar::Instance &instance, tigersugar::Drone_Tour &tour) {
        if (tour.node.empty()) {
            tour.node.resize(1);
            return;
        }

        vector<pair<double, int> > ls;
        for (int x : tour.node[0]) ls.push_back( make_pair(-instance.drone_cost[x], x) );
        sort(ls.begin(), ls.end());

        multiset<pair<double, int> > Set;
        REP(i, instance.numDrone) {
            if ( i < (int) tour.node.size() ) tour.node[i].clear();
            Set.insert(make_pair(0, i)); 
        }

        for (auto foo : ls) {
            int x = foo.second;

            auto it = Set.begin();
            int id = it -> second;
            double nTime = it->first + instance.drone_cost[x];
            Set.erase(it);
            Set.insert( make_pair(nTime, id) );

            while ( id >= (int) tour.size() ) tour.add_drone();
            tour.node[id].push_back(x);
        }   
    }
}