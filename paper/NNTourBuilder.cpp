#include "utilities.cpp"

/*
 * Build a giant tour with nearest-neighbor method.
 */
namespace NNTourBuilder {
    #define ORE_INFINITY        1000000007

    tigersugar::Tour build(tigersugar::Instance &instance, int srcNode, int type = 0) {
        int limit_heap_size;
        if (type == 0) limit_heap_size = 3;
        else limit_heap_size = rnd.next(1000000) % 2 + 1; 

        tigersugar::Tour tour;
        vector<bool> added(instance.numPoint+1, false);

        tour.add(srcNode);
        added[srcNode] = true;

        int x = srcNode;
        int depotPos = 0;
        for (int i = 1; i <= instance.numPoint; ++i) {
            priority_queue<pair<double, int> > heap;

            for (int y = 0; y <= instance.numPoint; ++y) {
                if (added[y]) continue;

                if ( (int)heap.size() < limit_heap_size ) {
                    heap.push( make_pair(instance.distance[x][y], y) );
                    continue;
                }

                pair<double, int> top = heap.top();
                if (instance.distance[x][y] < top.first) {
                    heap.pop();
                    heap.push( make_pair(instance.distance[x][y], y) );
                }
            }

            int id = rnd.next(111000) % (int) heap.size();
            FOR(Time, 1, id) heap.pop();

            tour.add( heap.top().second );
            x = heap.top().second;
            added[x] = true;

            if (x == 0) depotPos = i;
        }

        // Shift depot to first position
        assert(depotPos >= 0);
        rotate(tour.points.begin(), tour.points.begin() + depotPos, tour.points.end());
        assert(tour.points.front() == 0);
        tour.add(0);

        return tour;
    }

    tigersugar::Tour Build_Sub_Tour(tigersugar::Instance &instance, vector<pair<double, int> > &drone_keeping, int srcNode) {
        int limit_heap_size = 10;

        tigersugar::Tour tour;
        vector<bool> added(instance.numPoint+1, false);
        
        for (auto x : drone_keeping) added[x.second] = true;

        tour.add(srcNode);
        added[srcNode] = true;

        int x = srcNode;
        int depotPos = 0;
        for (int i = 1; i <= instance.numPoint - (int) drone_keeping.size(); ++i) {
            priority_queue<pair<double, int> > heap;

            for (int y = 0; y <= instance.numPoint; ++y) {
                if (added[y]) continue;

                if ( (int)heap.size() < limit_heap_size ) {
                    heap.push( make_pair(instance.distance[x][y], y) );
                    continue;
                }

                pair<double, int> top = heap.top();
                if (instance.distance[x][y] < top.first) {
                    heap.pop();
                    heap.push( make_pair(instance.distance[x][y], y) );
                }
            }

            int id = rnd.next(111000) % (int) heap.size();
            FOR(Time, 1, id) heap.pop();

            tour.add( heap.top().second );
            x = heap.top().second;
            added[x] = true;

            if (x == 0) depotPos = i;
        }

        // Shift depot to first position
        assert(depotPos >= 0);
        rotate(tour.points.begin(), tour.points.begin() + depotPos, tour.points.end());
        assert(tour.points.front() == 0);
        tour.add(0);

        return tour;

    }

    tigersugar::Tour ACO_Build_Sub_Tour2(tigersugar::Instance &instance, vector<pair<double, int> > &drone_keeping, int srcNode, vector<vector<double> > &tsp_phe) {
        int limit_heap_size = 10;
        
        tigersugar::Tour tour;
        vector<bool> added(instance.numPoint+1, false);

        for (auto x : drone_keeping) added[x.second] = true;

        tour.add(srcNode);
        added[srcNode] = true;

        int x = srcNode;
        int depotPos = 0;

        for (int i = 1; i <= instance.numPoint - (int) drone_keeping.size(); ++i) {
            priority_queue<pair<double, int> > heap;

            for (int y = 0; y <= instance.numPoint; ++y) {
                if (added[y]) continue;

                if ( (int)heap.size() < limit_heap_size ) {
                    heap.push( make_pair(-tsp_phe[x][y] / (instance.distance[x][y] + 1), y) );
                    continue;
                }

                pair<double, int> top = heap.top();
                if (-tsp_phe[x][y] / (instance.distance[x][y] + 1) < top.first) {
                    heap.pop();
                    heap.push( make_pair(-tsp_phe[x][y] / (instance.distance[x][y] + 1), y) );
                }
            }

            double phe_total = 0;
            vector<pair<double, int> > candList;
            while ((int)heap.size() > 0) {                 
                candList.push_back( heap.top() );
                phe_total += heap.top().first;

                heap.pop();
            }
            shuffle ( candList.begin(), candList.end() );

            double phe_part = -phe_total * (rnd.next(1000000) / 1000000.0);
            phe_total = 0;
            
            for (int j = 0; j < candList.size(); j++) {                  
                x = candList[j].second;
                phe_total += -candList[j].first;
                if ( phe_total > phe_part ) break;
            };
            
            tour.add( x ); 
            added[x] = true;

            if (x == 0) depotPos = i;
        }

        // Shift depot to first position
        assert(depotPos >= 0);
        rotate(tour.points.begin(), tour.points.begin() + depotPos, tour.points.end());
        assert(tour.points.front() == 0);
        tour.add(0);

        return tour;

    }

    tigersugar::Tour ACO_Build_Sub_Tour(tigersugar::Instance &instance, vector<pair<double, int> > &drone_keeping, int srcNode, vector<vector<double> > &tsp_phe) {
        tigersugar::Tour tour;
        vector<bool> added(instance.numPoint+1, false);
        
        for (auto x : drone_keeping) added[x.second] = true;

        tour.add(srcNode);
        added[srcNode] = true;

        int x = srcNode;
        int depotPos = 0;
        for (int i = 1; i <= instance.numPoint - (int) drone_keeping.size(); ++i) {
            double best_val = 0, plus_val = 0;
            int nextVertice = -1, plus_nextVertice = -1;
            for (int y = 0; y <= instance.numPoint; ++y) {
                if (added[y]) continue;
                int take = rnd.next(10000000) % 100;
                
                if (take <= 90) {
                    double cmp_val = tsp_phe[x][y] / (instance.distance[x][y]+1);
                    if (cmp_val > best_val) {
                        nextVertice = y;
                        best_val = cmp_val;
                    }
                }
                
                double cmp_val = tsp_phe[x][y] / (instance.distance[x][y]+1);
                if (cmp_val > plus_val) {
                    plus_nextVertice = y;
                    plus_val = cmp_val;
                }
            }

            if (nextVertice == -1) nextVertice = plus_nextVertice;

            x = nextVertice;
            tour.add(x);
            added[x] = true;

            if (x == 0) depotPos = i;
        }

        // Shift depot to first position
        assert(depotPos >= 0);
        rotate(tour.points.begin(), tour.points.begin() + depotPos, tour.points.end());
        assert(tour.points.front() == 0);
        tour.add(0);

        return tour;
    }
}
