#include "paper/utilities.cpp"
#include "paper/tsp_optimizer.cpp"
#include "paper/pms_optimizer.cpp"

namespace dynamic_programming_split_tour {
    #define limitPer 100

    #ifndef ORE_INFINITY
    #define ORE_INFINITY 1e9+7
    #endif

    bool Minimize(double &a, double b) {
        if (a > b) { a = b; return true; }
        return false;
    }

    void Build_Graph(const tigersugar::Instance &instance, vector<int> &V, vector<vector<int> > &adj, vector<double> &S2) {
        int n = V.size();
        adj.clear(); adj.resize(n);
        S2.clear(); S2.resize(n);
        
        REP(i, V.size()) {
            bool flag_eligible = true;
            S2[i] = (i == 0) ? instance.drone_cost[V[i]] : S2[i-1] + instance.drone_cost[V[i]];

            FOR(j, i+1, V.size()-1) {
                if ( !flag_eligible ) break;
                if ( !instance.drone_cost[V[j]] ) flag_eligible = false;
                adj[i].push_back(j);
            }
        }
    }

    double get_S2(const vector<double> &S2, int numDrone, int u, int v) {
        ++u; --v;
        if (u > v) return 0;
        if (u == 0) return S2[v];
        return (S2[v] - S2[u-1]) / (double) numDrone; 
    }

    void split(tigersugar::Instance &instance, tigersugar::Tour &vehicle, tigersugar::Drone_Tour &drone, vector<pair<double, int> > &drone_keeping, double best_cost) {  
        assert( drone.node.empty() );
        drone.node.resize(1);

        vector<int> V;
        for (int x : vehicle.points) V.push_back(x);

        vector<double> S2;
        vector<vector<int> > adj;
        Build_Graph(instance, V, adj, S2);

        vector<vector<double> > dp;
        vector<vector<pair<int, int> > > Trace;
        dp.resize(V.size());
        Trace.resize(V.size());
        FOR(i, 0, (int) V.size()-1) {
            dp[i].resize(limitPer+1, ORE_INFINITY + 1);
            Trace[i].resize(limitPer+1);
        }

        dp[0][0] = 0; 
        FOR(i, 0, (int) V.size()-1) FOR(per, 0, limitPer) {
            if (dp[i][per] > ORE_INFINITY) continue;
            for (int j : adj[i]) {
                double vehicle_cost = instance.distance[ V[i] ][ V[j] ];
                double drone_cost = get_S2(S2, instance.numDrone, i, j);
                drone_cost += best_cost * (double) per / (double) limitPer;
                
                int nPer = round( drone_cost / best_cost * (double) limitPer );
                if (nPer > limitPer) continue;

                if ( Minimize( dp[j][nPer], dp[i][per] + vehicle_cost ) ) Trace[j][nPer] = make_pair(i, per); 
            }
        }

        tigersugar::Tour ore_vehicle = vehicle;
        tigersugar::Drone_Tour ore_drone = drone;
        for (auto x : drone_keeping) ore_drone.node[0].push_back(x.second);
        pms_optimizer::optimizeTour(instance, ore_drone);
        double ore_cost = max( vehicle.distance(instance), ore_drone.distance(instance) );

        FOR(last_per, 0, limitPer) {    
            int per = last_per, i = (int) V.size()-1;
            if (dp[i][per] >= ORE_INFINITY + 1e-9) continue;

            tigersugar::Tour cur_vehicle;
            tigersugar::Drone_Tour cur_drone;
            cur_vehicle.points.push_back(0);
            cur_drone.node.resize(1);
            for (auto x : drone_keeping) cur_drone.node[0].push_back(x.second);

            /// trace back
            while (i > 0) { 
                if (V[i] != 0) cur_vehicle.points.push_back(V[i]);
                pair<int, int> foo = Trace[i][per];
                FOR(p, foo.first+1, i-1) if (V[p] != 0) cur_drone.node[0].push_back(V[p]);
                i = foo.first; per = foo.second;
            }

            cur_vehicle.points.push_back(0);
            assert(cur_vehicle.points[0] == 0 && cur_vehicle.points.back() == 0);

            /// optimize tour for drone
            pms_optimizer::optimizeTour(instance, cur_drone);
            
            double drone_cost = cur_drone.distance(instance);
            if (drone_cost >= ore_cost + 1e-9) continue;
            
            /// optimize tour for vehicle
            //tsp_optimizer::optimizeTour(instance, cur_vehicle);
            double vehicle_cost = cur_vehicle.distance(instance);

            /// update result
            if ( max(drone_cost, vehicle_cost) + 1e-9 < ore_cost ) {
                ore_cost = max(drone_cost, vehicle_cost);
                ore_vehicle = cur_vehicle;
                ore_drone = cur_drone;
            }
        }

        vehicle = ore_vehicle;
        drone = ore_drone;
    }
}