# Parrallel Drone Scheduling Traveling Salesman Problem

A new dynamic programming based algorithm for the Parrallel Drone Scheduling Traveling Salesman Problem

## How to run this code

Compile file <i>main.cpp</i>:
```
g++ main.cpp -o main
```

Run file <i>main</i> with config:

Config:

<i>--input</i> &nbsp; Name of input file. The input file must be in folder <i>/instances</i>. This param is required.

<i>--output</i> &nbsp; Name of output file.

<i>--customer</i> &nbsp; Number of customer. This param is required.

<i>--drone</i> &nbsp; Number of drone. The default of this value is 1.

<i>--vehicle-speed</i> &nbsp; Speed of the vehicle. The default of this value is 25.

<i>--drone-speed</i> &nbsp; Speed of drones. The default of this value is 25.


Example:
```
./main --input input.txt --output output.txt --customer 100 --drone 5 --vehicle-speed 20 --drone-speed 60
```
