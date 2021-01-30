#ifndef PSDTSP_CONFIG
#define PSDTSP_CONFIG

#include "template.cpp"
#include "constant.cpp"

namespace Config
{
    #define to_string   ore_to_string
    string input;
    bool found_input = false;

    int customer = 0;
    bool found_customer = false;

    int drone = 1;

    double vehicle_speed = Constant::DEFAULT_VEHICLE_SPEED;
    double drone_speed = Constant::DEFAULT_DRONE_SPEED;

    double time_limit = 300.0;
    int loop_limit = 10000;

    string output = "output/";

    string to_string(double val) {
        int tmp = (int) val;
        string ans = "";
        while (tmp > 0) ans += (char) ('0' + tmp%10), tmp /= 10;
        reverse(ans.begin(), ans.end());
        return ans;
    } 

    void parse_arguments(int argc, char *argv[])
    {
        for(int i = 1; i < argc; ++i)
        {
            string key = argv[i];

            if (key == "--input")
            {
                string value = argv[++i];
                input = value;
                found_input = true;
                output += value;
            }
            else
            if (key == "--drone")
            {
                int value = stoi(argv[++i]);
                drone = value;
                output += " Num drone " + to_string(value);
            }
            else
            if (key == "--vehicle-speed")
            {
                double value = stof(argv[++i]);
                vehicle_speed = value;
                output += " Vehicle speed " + to_string(value);
            }
            else
            if (key == "--drone-speed")
            {
                double value = stof(argv[++i]);
                drone_speed = value;
                output += " Drone speed " + to_string(value);
            }
            else 
            if (key == "--time-limit") {
                double val = stof(argv[++i]);
                time_limit = val;
                output += " Time limit " + to_string(val);
            }
            else if (key == "--loop-limit") {
                int val = stoi(argv[++i]);
                loop_limit = val;
                output += " Loop limit " + to_string(val);
            }
            else
            {
                cerr << "Unknow argument " << argv[i] << "!\n";
                exit(0);
            }
        }

        if (!found_input)
            {
                cerr << "Input is required!\n";
                exit(0);
            }
            // if (!found_customer)
            // {
            //     cerr << "Number of customers is required!\n";
            //     exit(0);
            // }
        
        for (int i = 0; i < (int) input.size(); ++i)
            if ( isdigit(input[i]) ) {
                while ( isdigit(input[i]) ) customer = customer * 10 + input[i] - '0', ++i;
                break;
            }
    }
}

#endif
