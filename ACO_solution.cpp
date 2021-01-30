#include "paper/utilities.cpp"
#include "paper/NNTourBuilder.cpp"

#include "paper/tsp_optimizer.cpp"
#include "problem.cpp"
#include "local_search_optimize.cpp"
#include "dynamic_programming_split_tour.cpp"

namespace ACO_solution {
    #ifndef ORE_INFINITY
    #define ORE_INFINITY 1e9+7
    #endif

    double maxPhe, minPhe, tLimit, rho, best_time, best_loop, tsp_rho;
    int nAnts, iLimit, num_drone_keeping, percent_drone_keeping;

    void Assign_Parameter(tigersugar::Instance &instance) {
        rho = 0.9;
        tsp_rho = 0.9;
        maxPhe = 1.0;
        minPhe = maxPhe / (double) (2*instance.numPoint);
        
        nAnts = instance.numPoint;
        iLimit = 10000;
        tLimit = instance.time_limit;
        percent_drone_keeping = 50;

        cerr<<"rho                  "<<rho<<"\n";
        cerr<<"tsp_rho              "<<tsp_rho<<"\n";
        cerr<<"nAnts                "<<nAnts<<"\n";
        cerr<<"iLimit               "<<iLimit<<"\n";
        cerr<<"tLimit               "<<tLimit<<"\n";

        cout<<"rho                  "<<rho<<"\n";
        cout<<"tsp_rho              "<<tsp_rho<<"\n";
        cout<<"nAnts                "<<nAnts<<"\n";
        cout<<"iLimit               "<<iLimit<<"\n";
        cout<<"tLimit               "<<tLimit<<"\n";
    }

    void process(tigersugar::Instance &instance) {
    	Assign_Parameter(instance);

        /// create phe array
        vector<double> phe;
        phe.resize(instance.numPoint+1, 0);
        FOR(i, 1, instance.numPoint) phe[i] = (instance.drone_cost[i] <= 1e-9) ? 0 : maxPhe; 
        
        /// create tsp_phe array
        vector<vector<double> > tsp_phe;
        tsp_phe.resize(instance.numPoint+1);
        FOR(i, 0, instance.numPoint) tsp_phe[i].resize(instance.numPoint+1, maxPhe);

        /// main algorithm process
        tigersugar::Tour best_vehicle;
        tigersugar::Drone_Tour best_drone;
        double best_vehicle_cost = ORE_INFINITY+1, best_drone_cost = ORE_INFINITY+1;

        clock_t startTime = clock();
        int numOutLoop = 0;
        FOR(loop, 1, iLimit) {
            if ((double)(clock() - startTime) / CLOCKS_PER_SEC > tLimit) break;
            ++numOutLoop;

            tigersugar::Tour best_inLoop_vehicle;
            tigersugar::Drone_Tour best_inLoop_drone;
            double best_inLoop_vehicle_cost, best_inLoop_drone_cost;

            if (best_vehicle_cost < ORE_INFINITY) {
            	num_drone_keeping = (instance.numPoint - (int) best_vehicle.size() + 2) / 2;
            }
           	else {
           		num_drone_keeping = 0;
           	}
            
            FOR(ant, 1, nAnts) {
                /// build always drone
                vector<pair<double, int> > drone_keeping;
                FOR(x, 1, instance.numPoint) {
                    if (instance.drone_cost[x] <= 1e-9) continue;
                    int keep = rnd.next(100000000) % 100;
                    
                    if (keep <= percent_drone_keeping) drone_keeping.push_back( make_pair(phe[x], x) );
                }

                /// get points which always assigned for drone tour 
                sort(drone_keeping.begin(), drone_keeping.end());
                reverse(drone_keeping.begin(), drone_keeping.end());
                while ((int) drone_keeping.size() > num_drone_keeping) drone_keeping.pop_back(); 

                /*for(int i=0; i < drone_keeping.size(); i++)
                	cerr<<"("<<drone_keeping[i].second<<", "<<drone_keeping[i].first<<") ";
                cerr<<"\n";*/

                /// build tsp tour with remain vertice
                tigersugar::Tour vehicle = NNTourBuilder::ACO_Build_Sub_Tour2(instance, drone_keeping, 0, tsp_phe);

                //tigersugar::Tour vehicle = NNTourBuilder::Build_Sub_Tour(instance, drone_keeping, 0);
                tigersugar::Drone_Tour drone;
                tsp_optimizer::optimizeTour(instance, vehicle);

                /// dynamic programming
                double cost;
                if (loop == 1 && ant == 1) cost = vehicle.distance(instance);
                else cost = max(best_drone_cost, best_vehicle_cost);

                dynamic_programming_split_tour::split(instance, vehicle, drone, drone_keeping, cost);

                //cerr << "DP " <<vehicle.distance(instance)<<"(" <<vehicle.size() <<") - "
                //						<<drone.distance(instance)<<"(" <<drone.size() <<")"<<'\n';

                tsp_optimizer::optimizeTour(instance, vehicle);
                /// local search

                while (true) {
                    bool ok = false;
                    while ( local_search::optimize_drone(instance, drone) ) { ok = true; }
                    while ( local_search::optimize(instance, vehicle, drone) ) { ok = true; }
                    if (!ok) break;
                }

                /// update inLoop tour
                double vehicle_cost = vehicle.distance(instance);
                double drone_cost = drone.distance(instance);
                if ( ant == 1 || max(best_inLoop_vehicle_cost, best_inLoop_drone_cost) > max(vehicle_cost, drone_cost) ) {
                    best_inLoop_vehicle_cost = vehicle_cost;
                    best_inLoop_drone_cost = drone_cost;

                    best_inLoop_vehicle = vehicle;
                    best_inLoop_drone = drone;
                } 

            }

            /// local search
            // while ( local_search::optimize(instance, best_inLoop_vehicle, best_inLoop_drone) ) {cerr<<"*";}

            /// update outLoop tour
            if ( max(best_vehicle_cost, best_drone_cost) > max(best_inLoop_vehicle_cost, best_inLoop_drone_cost) ) {
                best_vehicle_cost = best_inLoop_vehicle_cost;
                best_drone_cost = best_inLoop_drone_cost;

                best_vehicle = best_inLoop_vehicle;
                best_drone = best_inLoop_drone;
                cerr<<"--- NEW BEST --- at  " << (double)(clock() - startTime) / CLOCKS_PER_SEC << "\n";
                cout<<"--- NEW BEST --- at  " << (double)(clock() - startTime) / CLOCKS_PER_SEC << "\n";
                best_time = (double)(clock() - startTime) / CLOCKS_PER_SEC;
                best_loop = loop;
            }

            cerr << "#" << loop << ": " <<best_inLoop_vehicle_cost<<"(" <<best_inLoop_vehicle.size() <<") - "
                						<<best_inLoop_drone_cost<<"(" <<best_inLoop_drone.size() <<") - best "
                						<<max(best_vehicle_cost, best_drone_cost)<<'\n';
            cout << "#" << loop << ": " <<best_inLoop_vehicle_cost<<"(" <<best_inLoop_vehicle.size() <<") - "
                						<<best_inLoop_drone_cost<<"(" <<best_inLoop_drone.size() <<") - best "
                						<<max(best_vehicle_cost, best_drone_cost)<<'\n';

            /// update phe[]
            for (int x : best_inLoop_vehicle.points) {
                phe[x] = phe[x] * rho + minPhe * (1-rho); /// x is in vehicle tour
            }
            for (vector<int> vec : best_inLoop_drone.node) 
                for (int x : vec) {
                	phe[x] = phe[x] * rho + maxPhe * (1-rho); /// x is in drone tour
                	 //cerr<<"("<<x<<","<<phe[x]<<") ";
                	 //cout<<"("<<x<<","<<phe[x]<<") ";
            	}
            //cerr<<endl;
            //cout<<endl;

            FOR(i, 0, (int) best_inLoop_vehicle.points.size()-2) {
                int u = best_inLoop_vehicle.points[i], v = best_inLoop_vehicle.points[i+1];
                if (u > v) swap(u,v);

                tsp_phe[u][v] = tsp_phe[u][v] * tsp_rho + maxPhe * (1-tsp_rho);
                tsp_phe[v][u] = -1;
            }  

            FOR(u, 0, instance.numPoint - 1)  
                FOR(v, u+1, instance.numPoint - 1) { 
                    if (tsp_phe[v][u] < 0) tsp_phe[v][u] = tsp_phe[u][v];
                    else {
                        tsp_phe[u][v] = tsp_phe[u][v] * tsp_rho + minPhe * (1-tsp_rho);
                        tsp_phe[v][u] = tsp_phe[u][v];
                    }
                }

            /*
            double p = 0;
            for (int step=0; step<100; step++) {
                p += 0.01;
                int cnt = 0;
                for (int u = 0; u < instance.numPoint; u++)
                    for (int v = u+1; v < instance.numPoint; v++)
                        if ((tsp_phe[u][v] > p-0.01) && (tsp_phe[u][v] <= p)) cnt++;

                cerr<< cnt << " ";
                cout<< cnt << " ";
            }
            cerr<<"\n";
            cout<<"\n";
            */

        } 

        /// Output
        Problem::result = max( best_vehicle.distance(instance), best_drone.distance(instance) );
        Problem::vTour = best_vehicle.points;
        Problem::dTour = best_drone.node;
        Problem::numOutLoop = best_loop;
        Problem::excutionTime = best_time;
        cerr << "\n\nbest loop: " << best_loop<<" best time: "<<best_time<<" Excution time: " << (double)(clock() - startTime) / CLOCKS_PER_SEC << "\n\n";
        cout << "\n\nbest loop: " << best_loop<<" best time: "<<best_time<<" Excution time: " << (double)(clock() - startTime) / CLOCKS_PER_SEC << "\n\n";
    }
}