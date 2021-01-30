#ifndef PDSTSP_PROBLEM
#define PDSTSP_PROBLEM

#include <bits/stdc++.h>

#include "point.cpp"
#include "constant.cpp"

using namespace std;

namespace Problem
{
    int n; // Number of customers
    int m; // Number of drones
    int depot = 0;
    vector<double> dCost; // cost of traveling by a drone from depot to customer and return
    vector<vector<double> > vCost; // cost of traveling by the vehicle from this customer to another

    int nD; // Number of drone-eligible customers

    double result;
    vector<int> vTour; // tour of the vehicle, the first and the last must be depot
    vector<vector<int>> dTour; // list of customers assigned of drones, not containing the depot

    double time_limit, excutionTime;
    double loop_limit, numOutLoop;

    string output;

    void import_data_from_tsplib_instance(string input_file, int _n, int _m, double vSpeed, double dSpeed, double _time_limit, int _loop_limit, string _output)
    {
        // sV: speed of vehicle
        // sD: speed of drone

        cerr << "Reading data from " << input_file << "...\n";

        n = _n;
        m = _m;
        time_limit = _time_limit;
        loop_limit = _loop_limit;
        output = _output;
        
	    input_file = "data/" + input_file;
        ifstream input(input_file);
        vector<Point::point> points;

        for(int i = 0; i <= n; ++i)
        {
            char temp;
            int id, drone_eligible;
            double x, y;
            input >> id >> temp >> x >> temp >> y >> temp >> drone_eligible;
            drone_eligible = 1 - drone_eligible;
            points.push_back(Point::point(x, y));
            if (drone_eligible)
            {
                dCost.push_back(2 * Point::euclidean_distance(points[0], points[i]) / dSpeed);
                nD++;
            }
            else 
                dCost.push_back(Constant::INF);
        }

        for(int i = 0; i <= n; ++i)
        {
            vector<double> a = vector<double>();
            vCost.push_back(vector<double>());
            for(int j = 0; j <= n; ++j)
                vCost[i].push_back(Point::manhattan_distance(points[i], points[j]) / vSpeed);
        }
    }

    void print_result_to_file(string output_file)
    {
        freopen(output_file.c_str(), "w", stdout);

        cout << "Result: " << result << "\n";

        cout << "Vehicle tour: ";
        for(auto id : vTour) cout << id << " ";
        cout << "\n";

        for(int i = 0; i < m; ++i)
        {
            cout << "Customers of drone #" << i+1 << ": ";
            for(auto id : dTour[i]) cout << id << " ";
            cout << "\n";
        }
    }

}

#endif

