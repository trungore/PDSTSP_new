#include "paper/utilities.cpp"

namespace local_search {
    bool optimize3(tigersugar::Instance &instance, tigersugar::Tour &vehicle, tigersugar::Drone_Tour &drone) {
        if ( (int) drone.numPointAssigned() < 2 ) return false;

        bool flag_optimize = false;
        tigersugar::Tour bestVehicle = vehicle;
        tigersugar::Drone_Tour bestDrone = drone;
        double vehicle_cost = vehicle.distance(instance);
        double drone_cost = drone.distance(instance);
        double bestCost = max( vehicle.distance(instance), drone.distance(instance) );

        multiset<double> Set; /// save 3 max drone
        vector<double> save_drone;
        save_drone.resize( drone.node.size() );
        
        double maxDrone[3] = { -1, -1, -1 };
        FOR(i, 0, drone.node.size()-1) {
            for (int x : drone.node[i]) save_drone[i] += instance.drone_cost[x];
            if (save_drone[i] >= maxDrone[0]) {
                maxDrone[2] = maxDrone[1];
                maxDrone[1] = maxDrone[0];
                maxDrone[0] = save_drone[i]; 
            }
            else if (save_drone[i] >= maxDrone[1]) {
                maxDrone[2] = maxDrone[1];
                maxDrone[1] = save_drone[i];
            }
            else if (save_drone[i] >= maxDrone[2]) {
                maxDrone[2] = save_drone[i];
            }
        }

        FOR(x, 0, vehicle.points.size()-1)  
        FOR(y, 0, drone.node.size()-1) FOR(i, 0, drone.node[y].size()-1)
        FOR(z, y, drone.node.size()-1) FOR(j, 0, drone.node[z].size()-1) {
            if (y == z && i >= j) continue;
            if ( instance.drone_cost[ vehicle.points[x] ] <= 0 ) continue;
            assert( vehicle.points[x] != 0 );
        
            /// before shuffle
            double nVehicle_cost = vehicle_cost;    
            nVehicle_cost -= instance.distance[ vehicle.points[x-1] ][ vehicle.points[x] ];
            nVehicle_cost -= instance.distance[ vehicle.points[x+1] ][ vehicle.points[x] ];

            double nDrone1_cost = save_drone[y];
            nDrone1_cost -= instance.drone_cost[ drone.node[y][i] ];
            double nDrone2_cost = save_drone[z];
            nDrone2_cost -= instance.drone_cost[ drone.node[z][j] ];

            /// permutation shuffle
            int per[3] = { vehicle.points[x], drone.node[y][i], drone.node[z][j] };
            sort(per, per+3);
            
            do {
                
                double save_vehicle = nVehicle_cost, save_drone1 = nDrone1_cost, save_drone2 = nDrone2_cost;

                nVehicle_cost += instance.distance[ vehicle.points[x-1] ][ per[0] ];
                nVehicle_cost += instance.distance[ vehicle.points[x+1] ][ per[0] ];
                
                nDrone1_cost += instance.drone_cost[ per[1] ];
                nDrone2_cost += instance.drone_cost[ per[2] ];
                
                double newMaxDrone[5] = { maxDrone[0], maxDrone[1], maxDrone[2], nDrone1_cost, nDrone2_cost };
                FOR(id, 0, 2) if (newMaxDrone[id] > 1e-9 && newMaxDrone[id] <= save_drone[y]) { newMaxDrone[id] = -1; break; }
                if (save_drone[y] != save_drone[z])
                    FOR(id, 0, 2) if (newMaxDrone[id] > 1e-9 && newMaxDrone[id] <= save_drone[z]) { newMaxDrone[id] = -1; break; }
                   
                double nDrone_cost = -2;
                FOR(id, 0, 4) if (nDrone_cost + 1e-9 < newMaxDrone[id]) nDrone_cost = newMaxDrone[id];

                if ( max(nVehicle_cost, nDrone_cost) + 1e-9 < bestCost ) {
                        //cerr << "\n-----------------LEVEL-UP-----------------------\n";
                        //cerr << max(nVehicle_cost, nDrone_cost) <<  " " << bestCost << '\n';

                    flag_optimize = true;
                    bestCost = max(nVehicle_cost, nDrone_cost);

                    bestVehicle = vehicle;
                    bestVehicle.points[x] = per[0];

                    bestDrone = drone;
                    bestDrone.node[y][i] = per[1];
                    bestDrone.node[z][j] = per[2];

                        // FOR(id, 0, drone.node.size()-1) cerr << save_drone[id] << '\n';

                        // cerr << "maxDrone[]: ";
                        // FOR(id, 0, 2) cerr << maxDrone[id] << " ";
                        // cerr << '\n';

                        // cerr << "save_drone y, z: " << save_drone[y] << " " << save_drone[z] << '\n';

                        // cerr << "newMaxDrone[]: ";
                        // FOR(id, 0, 4) cerr << newMaxDrone[id] << " ";
                        // cerr << '\n';

                        // cerr << "nDrone_cost: " << nDrone_cost << '\n';

                        // cerr << "YASUOL HASAGI ===========> " << bestVehicle.distance(instance) << " " 
                        //     << bestDrone.distance(instance) << " " << nVehicle_cost << " " << nDrone_cost << '\n';

                }

                nVehicle_cost = save_vehicle;
                nDrone1_cost = save_drone1;
                nDrone2_cost = save_drone2;

            } while (next_permutation(per, per+3));
        }

        vehicle = bestVehicle;
        drone = bestDrone;
        return flag_optimize;
    }

    bool optimize_drone(tigersugar::Instance &instance, tigersugar::Drone_Tour &drone) {
        if ( (int) drone.node.size() <= 1 ) return false;

        vector<double> save_drone;
        save_drone.resize( drone.node.size() );
        
        double maxDrone[3] = { -1, -1, -1 };
        FOR(i, 0, drone.node.size()-1) {
            for (int x : drone.node[i]) save_drone[i] += instance.drone_cost[x];
            if (save_drone[i] >= maxDrone[0]) {
                maxDrone[2] = maxDrone[1];
                maxDrone[1] = maxDrone[0];
                maxDrone[0] = save_drone[i]; 
            }
            else if (save_drone[i] >= maxDrone[1]) {
                maxDrone[2] = maxDrone[1];
                maxDrone[1] = save_drone[i];
            }
            else if (save_drone[i] >= maxDrone[2]) {
                maxDrone[2] = save_drone[i];
            }
        }

        bool flag_optimize = false;
        tigersugar::Drone_Tour best_drone = drone;
        double best_cost = drone.distance(instance);

        FOR(x, 0, (int) drone.node.size()-1) FOR(y, x+1, (int) drone.node.size()-1) 
            FOR(i, 0, drone.node[x].size()-1) FOR(j, 0, drone.node[y].size()-1) {
                double nDrone1_cost = save_drone[x];
                nDrone1_cost -= instance.drone_cost[ drone.node[x][i] ];
                double nDrone2_cost = save_drone[y];
                nDrone2_cost -= instance.drone_cost[ drone.node[y][j] ];

                nDrone1_cost += instance.drone_cost[ drone.node[y][j] ];
                nDrone2_cost += instance.drone_cost[ drone.node[x][i] ];

                double newMaxDrone[5] = { maxDrone[0], maxDrone[1], maxDrone[2], nDrone1_cost, nDrone2_cost };
                FOR(id, 0, 2) if (newMaxDrone[id] > 1e-9 && newMaxDrone[id] <= save_drone[x]) { newMaxDrone[id] = -1; break; }
                if (save_drone[x] != save_drone[y])
                    FOR(id, 0, 2) if (newMaxDrone[id] > 1e-9 && newMaxDrone[id] <= save_drone[y]) { newMaxDrone[id] = -1; break; }
                   
                double nDrone_cost = -2;
                FOR(id, 0, 4) if (nDrone_cost + 1e-9 < newMaxDrone[id]) nDrone_cost = newMaxDrone[id];

                if (best_cost > nDrone_cost + 1e-9) {
                    best_cost = nDrone_cost;
                    best_drone = drone;
                    swap(best_drone.node[x][i], best_drone.node[y][j]);
                    flag_optimize = true;
                }
            }

        drone = best_drone;
        return flag_optimize;
    }

    bool optimize(tigersugar::Instance &instance, tigersugar::Tour &vehicle, tigersugar::Drone_Tour &drone) {
        if ( vehicle.points.empty() || drone.node.empty() || drone.node[1].empty() ) return false;  

        bool flag_optimize = false;
        tigersugar::Tour bestVehicle = vehicle;
        tigersugar::Drone_Tour bestDrone = drone;
        double vehicle_cost = vehicle.distance(instance);
        double drone_cost = drone.distance(instance);
        double bestCost = max( vehicle.distance(instance), drone.distance(instance) );

        vector<double> save_drone;
        save_drone.resize( drone.node.size() );
        double maxDrone0 = -1, maxDrone1 = -1;
        FOR(i, 0, drone.node.size()-1) {
            for (int x : drone.node[i]) save_drone[i] += instance.drone_cost[x];
            if (maxDrone0 + 1e-9 < save_drone[i]) {
                maxDrone1 = maxDrone0;
                maxDrone0 = save_drone[i];
            }
            else if (maxDrone1 + 1e-9 < save_drone[i]) maxDrone1 = save_drone[i];
        }

        FOR(x, 0, vehicle.points.size()-1)  
        FOR(y, 0, drone.node.size()-1) FOR(i, 0, drone.node[y].size()-1) {
            if (vehicle.points[x] == 0) continue;
            if ( instance.drone_cost[ vehicle.points[x] ] == 0 ) continue;

            double nVehicle_cost = vehicle_cost;
            nVehicle_cost -= instance.distance[ vehicle.points[x-1] ][ vehicle.points[x] ];
            nVehicle_cost -= instance.distance[ vehicle.points[x+1] ][ vehicle.points[x] ];
            
            double nDrone_cost = save_drone[y];
            nDrone_cost -= instance.drone_cost[ drone.node[y][i] ];

            swap( vehicle.points[x], drone.node[y][i] );
            nVehicle_cost += instance.distance[ vehicle.points[x-1] ][ vehicle.points[x] ];
            nVehicle_cost += instance.distance[ vehicle.points[x+1] ][ vehicle.points[x] ];

            nDrone_cost += instance.drone_cost[ drone.node[y][i] ];
            if (nDrone_cost + 1e-9 < maxDrone0) {
                if (save_drone[y] + 1e-9 >= maxDrone0) nDrone_cost = max( maxDrone1, nDrone_cost ); /// y la maxDrone0
                else nDrone_cost = maxDrone0;
            }

            // cerr << "?? " << nDrone_cost << " " << drone.distance(instance) << '\n';
            // cerr << "@@ " << nVehicle_cost << " " << vehicle.distance(instance) << '\n';

            // if (nVehicle_cost != vehicle.distance(instance)) {
            //     cerr << "?? " << vehicle_cost << " " << nVehicle_cost << '\n';
            //     cerr << "??? " << vehicle.distance(instance) << '\n';
            //     exit(0);
            // }
            // assert( nVehicle_cost == vehicle.distance(instance) );
            // assert( nDrone_cost == drone.distance(instance) );

            double nCost = max( nVehicle_cost, nDrone_cost );

            if (bestCost > nCost + 1e-9) {
                    // cerr << "LOCAL SEARCH: " << nCost << '\n';
                flag_optimize = true;
                bestCost = nCost;
                bestVehicle = vehicle;
                bestDrone = drone;

                // cerr << "??? " << bestVehicle.distance(instance) << " " << bestDrone.distance(instance) << " " << bestCost << '\n';
                // cerr << "@@@ " << nVehicle_cost << " " << nDrone_cost << '\n';
                // cerr << "NEW/OLD: " << instance.drone_cost[ drone.node[y][i] ] << " " << instance.drone_cost[ vehicle.points[x] ] << '\n';
                // cerr << "LAG: " << vehicle_cost << " " << drone_cost << " " << max(vehicle_cost, drone_cost) << '\n'; 
            }

            swap( vehicle.points[x], drone.node[y][i] );
        }

        vehicle = bestVehicle;
        drone = bestDrone;
        return flag_optimize;
    }
}