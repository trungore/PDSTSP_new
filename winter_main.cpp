#include "template.cpp"
#include "config.cpp"
#include "problem.cpp"
#include "validator.cpp"
#include "paper_Input_adaptation.cpp"

#include "ACO_solution.cpp"

int main(int argc, char *argv[])
{
    ios_base::sync_with_stdio(0);
    cin.tie(0);
    cout.tie(0);

    Config::parse_arguments(argc, argv);

    Problem::import_data_from_tsplib_instance(
        Config::input,
        Config::customer,
        Config::drone,
        Config::vehicle_speed,
        Config::drone_speed,
        Config::time_limit,
        Config::loop_limit,
        Config::output 
    );

    freopen(Problem::output.c_str(), "w", stdout);
    string sat_file = Problem::output + ".sat";
    ofstream sat(sat_file);

    tigersugar::Instance instance = Input_Adaptation::Process();

    double avgRes = 0, avgTime = 0, avgLoop = 0, maxRes = -1, minRes = -1;
    FOR(Time, 1, 10) {
    	cerr<<Time<<"\n";
        ACO_solution::process(instance);

        cerr << "Number of customers: " << Problem::n << "\n";
        cerr << "Number of drones: " << Problem::m << "\n";
        cerr << "Number of drone_eligible: " << Problem::nD-1 << "\n\n";
        
            cerr << "RESULT: " << Problem::result << "\n\n";
            
            cerr << "VEHICLE TOUR: ";
            for (int x : Problem::vTour) cerr << x << " ";
            cerr << "\n\n";
            
            cerr << "DRONE TOUR:\n";
            for (int i = 0; i < (int) Problem::dTour.size(); ++i) {
                cerr << "drone " << i+1 << ": ";
                for (int x : Problem::dTour[i]) cerr << x << " ";
                cerr << '\n';
            }
            cerr << '\n';

        cout << "Number of customers: " << Problem::n << "\n";
        cout << "Number of drones: " << Problem::m << "\n";
        cout << "Number of drone_eligible: " << Problem::nD-1 << "\n\n";
        
            cout << "RESULT: " << Problem::result << "\n\n";
            
            cout << "VEHICLE TOUR: ";
            for (int x : Problem::vTour) cout << x << " ";
            cout << "\n\n";
            
            cout << "DRONE TOUR:\n";
            for (int i = 0; i < (int) Problem::dTour.size(); ++i) {
                cout << "drone " << i+1 << ": ";
                for (int x : Problem::dTour[i]) cout << x << " ";
                cout << '\n';
            }
            cout << '\n';

        //cerr<<"Check "<<winter_validator::checking()<<"\n";

        sat << Problem::result << " " << Problem::excutionTime << " " << Problem::numOutLoop << endl;
        
        minRes = (Time == 1) ? Problem::result : min(minRes, Problem::result);
        maxRes = (Time == 1) ? Problem::result : max(maxRes, Problem::result);
        
        avgRes += Problem::result;
        avgTime += Problem::excutionTime;
        avgLoop += Problem::numOutLoop;
    }

    cerr <<"\nbest-cost\t\tavg-cost\t\tavg-time\t\tworst-cost\t\tavg-loop\n";
    cerr << minRes << "\t\t\t" << avgRes/10 << "\t\t\t" << avgTime/10 << "\t\t\t" << maxRes << "\t\t\t" << avgLoop/10 << '\n'; 

    sat <<"\nbest-cost\t\tavg-cost\t\tavg-time\t\tworst\t\tavg-loop\n";
    sat << minRes << "\t\t\t" << avgRes/10 << "\t\t\t" << avgTime/10 << "\t\t\t" << maxRes << "\t\t\t" << avgLoop/10 << '\n';  

    cout <<"\nbest-cost\t\tavg-cost\t\tavg-time\t\tworst-cost\t\tavg-loop\n";
    cout << minRes << "\t\t\t" << avgRes/10 << "\t\t\t" << avgTime/10 << "\t\t\t" << maxRes << "\t\t\t" << avgLoop/10 << '\n';      

}
