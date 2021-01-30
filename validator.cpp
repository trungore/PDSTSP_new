#include "problem.cpp"

namespace winter_validator {
    const double eps = 1e-9;
            
    bool checking() {
        /// check vTour 
        if ( (int) Problem::vTour.size() >= 2 && (Problem::vTour[0] != 0 || Problem::vTour.back() != 0) ) {
            cerr << "ERROR: vTour must have depot in both START and FINISH\n";
            exit(0); 
        }

        /// check duplicated 
        vector<bool> seen;
        seen.resize(Problem::n+1, 0);

        for (int x : Problem::vTour) {
            if (x == 0) continue;
            if (seen[x]) { 
                cerr << "ERROR: " << x << " is duplicated 1\n";
                cout << "ERROR: " << x << " is duplicated 1\n";
                exit(0); 
            } 
            seen[x] = true;
        } 

        for (vector<int> drone : Problem::dTour) for (int x : drone) {
            if (Problem::dCost[x] == Constant::INF) {
                cerr << "ERROR: " << x << " is not drone eligible\n";
                cout << "ERROR: " << x << " is not drone eligible\n";
                exit(0);
            }

            if (x == 0) {
                cerr << "ERROR: depot is in drone tour \n";
                cout << "ERROR: depot is in drone tour \n";
                exit(0);
            }
            if (seen[x]) {
                cerr << "ERROR: " << x << " is duplicated 2\n";
                cout << "ERROR: " << x << " is duplicated 2\n";
                exit(0);
            }
            seen[x] = true;
        }

        /// check missing
        for (int i = 1; i <= Problem::n; ++i) 
            if (!seen[i]) {
                cerr << "ERROR: missing " << i << '\n';
                cout << "ERROR: missing " << i << '\n';
                exit(0); 
            }

        /// check result
        double veh_cost = 0;
        for (int i = 0; i <= (int) Problem::vTour.size()-2; ++i) {
            int u = Problem::vTour[i], v = Problem::vTour[i+1];
            veh_cost += Problem::vCost[u][v];
        }

        double drone_cost = 0;
        bool flag = false;
        for (vector<int> drone : Problem::dTour) {
            double cost = 0;
            for (int x : drone) cost += Problem::dCost[x];
            if (!flag) { drone_cost = cost; flag = true; }
            if (drone_cost + eps < cost) drone_cost = cost;
        }

        double total_cost = max(veh_cost, drone_cost);
        if ( abs(total_cost - Problem::result) > eps ) {
            cerr << "ERROR: result doesn't match for tour\n";
            cout << "ERROR: result doesn't match for tour\n";
            exit(0); 
        }
        
        cerr << "VALIDATOR PASS: Congratulation!!!!\n\n";
        cout << "VALIDATOR PASS: Congratulation!!!!\n\n";

        exit(1);
    }
}