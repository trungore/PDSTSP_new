#ifndef TSP_OPTIMIZER
#define TSP_OPTIMIZER

#include "utilities.cpp"

namespace tsp_optimizer {
    struct Tour {
        vector<int> nodes; // This vector contains 0 (starting/ending point) at both ends.

        Tour(){}

        Tour(const vector<int> &v) {
            nodes.push_back(0);
            FORE(it, v) {
                nodes.push_back(*it);
            }
            nodes.push_back(0);
        }

        void append(int x) {
            nodes.push_back(x);
        }

        int operator[](int x) const { return nodes[x]; }

        int length() const {
            return nodes.size();
        }

        double calculateCost(const tigersugar::Instance &instance) const {
            double sum = 0;
            for (int i = 1; i < (int)nodes.size(); ++i) {
                sum += instance.distance[nodes[i - 1]][nodes[i]];
            }
            return sum;
        }
    };

    struct TspProblem {
        vector<vector<double>> distance;
        vector<vector<int>> nn_list;

        TspProblem(const tigersugar::Instance &instance, const Tour &tour) {
            auto n = tour.length() - 1;
            distance.resize(n, vector<double>(n));
            nn_list.resize(n, vector<int>(n - 1));

            vector<pair<double, int>> tmp;
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    distance[i][j] = instance.distance[tour[i]][tour[j]];
                }

                for (int j = 0; j < n; ++j) {
                    if (j != i) {
                        tmp.emplace_back(distance[i][j], j);
                    }
                }
                sort(tmp.begin(), tmp.end());
                for (int j = 0; j < n - 1; ++j) {
                    nn_list[i][j] = tmp[j].second;
                }
                tmp.clear();
            }
        }
    };

    static const int nn_ls = 20;
    static const double EPS = 0.0001;

    enum OptimizeMethod {
        twoOptsMethod,
        threeOptsMethod
    };

    void two_opt_first(vector<int> &tour, const TspProblem &instance)
    /*
      FUNCTION:       2-opt a tour
      INPUT:          pointer to the tour that undergoes local optimization
      OUTPUT:         none
      (SIDE)EFFECTS:  tour is 2-opt
      COMMENTS:       the neighbourhood is scanned in random order (this need
                      not be the best possible choice). Concerning the speed-ups used
              here consult, for example, Chapter 8 of
              Holger H. Hoos and Thomas Stuetzle,
              Stochastic Local Search---Foundations and Applications,
              Morgan Kaufmann Publishers, 2004.
              or some of the papers online available from David S. Johnson.
    */
    {
        int n = tour.size() - 1;
        int c1, c2;             /* cities considered for an exchange */
        int s_c1, s_c2;         /* successor cities of c1 and c2     */
        int p_c1, p_c2;         /* predecessor cities of c1 and c2   */
        int pos_c1, pos_c2;     /* positions of cities c1, c2        */
        int i, j, h, l;
        int improvement_flag, improve_node, help, n_improves = 0, n_exchanges=0;
        int h1=0, h2=0, h3=0, h4=0;
        double radius;          /* radius of nn-search */
        double gain = 0;
        vector<int> random_vector;
        vector<int> pos(n);     /* positions of cities in tour */
        vector<int> dlb(n);     /* vector containing don't look bits */

        for ( i = 0 ; i < n ; i++ ) {
            pos[tour[i]] = i;
            dlb[i] = false;
        }

        improvement_flag = true;
        random_vector = rnd.perm(n);

        while ( improvement_flag ) {

            improvement_flag = false;

            for (l = 0 ; l < n; l++) {

                c1 = random_vector[l];
                if ( dlb[c1] )
                    continue;
                improve_node = false;
                pos_c1 = pos[c1];
                s_c1 = tour[pos_c1+1];
                radius = instance.distance[c1][s_c1];

                /* First search for c1's nearest neighbours, use successor of c1 */
                for ( h = 0 ; h < nn_ls && h < n - 1 ; h++ ) {
                    c2 = instance.nn_list[c1][h]; /* exchange partner, determine its position */
                    if ( radius > instance.distance[c1][c2] ) {
                        s_c2 = tour[pos[c2]+1];
                        gain =  - radius + instance.distance[c1][c2] +
                                instance.distance[s_c1][s_c2] - instance.distance[c2][s_c2];
                        if ( gain < -EPS ) {
                            h1 = c1; h2 = s_c1; h3 = c2; h4 = s_c2;
                            improve_node = true;
                            goto exchange2opt;
                        }
                    }
                    else
                        break;
                }
                /* Search one for next c1's h-nearest neighbours, use predecessor c1 */
                if (pos_c1 > 0)
                    p_c1 = tour[pos_c1-1];
                else
                    p_c1 = tour[n-1];
                radius = instance.distance[p_c1][c1];
                for ( h = 0 ; h < nn_ls && h < n - 1 ; h++ ) {
                    c2 = instance.nn_list[c1][h];  /* exchange partner, determine its position */
                    if ( radius > instance.distance[c1][c2] ) {
                        pos_c2 = pos[c2];
                        if (pos_c2 > 0)
                            p_c2 = tour[pos_c2-1];
                        else
                            p_c2 = tour[n-1];
                        if ( p_c2 == c1 )
                            continue;
                        if ( p_c1 == c2 )
                            continue;
                        gain =  - radius + instance.distance[c1][c2] +
                                instance.distance[p_c1][p_c2] - instance.distance[p_c2][c2];
                        if ( gain < -EPS ) {
                            h1 = p_c1; h2 = c1; h3 = p_c2; h4 = c2;
                            improve_node = true;
                            goto exchange2opt;
                        }
                    }
                    else
                        break;
                }
                if (improve_node) {
                    exchange2opt:
                    n_exchanges++;
                    improvement_flag = true;
                    dlb[h1] = false; dlb[h2] = false;
                    dlb[h3] = false; dlb[h4] = false;
                    /* Now perform move */
                    if ( pos[h3] < pos[h1] ) {
                        help = h1; h1 = h3; h3 = help;
                        help = h2; h2 = h4; h4 = help;
                    }
                    if ( pos[h3] - pos[h2] < n / 2 + 1) {
                        /* reverse inner part from pos[h2] to pos[h3] */
                        i = pos[h2]; j = pos[h3];
                        while (i < j) {
                            c1 = tour[i];
                            c2 = tour[j];
                            tour[i] = c2;
                            tour[j] = c1;
                            pos[c1] = j;
                            pos[c2] = i;
                            i++; j--;
                        }
                    }
                    else {
                        /* reverse outer part from pos[h4] to pos[h1] */
                        i = pos[h1]; j = pos[h4];
                        if ( j > i )
                            help = n - (j - i) + 1;
                        else
                            help = (i - j) + 1;
                        help = help / 2;
                        for ( h = 0 ; h < help ; h++ ) {
                            c1 = tour[i];
                            c2 = tour[j];
                            tour[i] = c2;
                            tour[j] = c1;
                            pos[c1] = j;
                            pos[c2] = i;
                            i--; j++;
                            if ( i < 0 )
                                i = n-1;
                            if ( j >= n )
                                j = 0;
                        }
                        tour[n] = tour[0];
                    }
                } else {
                    dlb[c1] = true;
                }

            }
            if ( improvement_flag ) {
                n_improves++;
            }
        }
    }

    void three_opt_first(vector<int> &tour, const TspProblem &instance)
    /*
      FUNCTION:       3-opt the tour
      INPUT:          pointer to the tour that is to optimize
      OUTPUT:         none
      (SIDE)EFFECTS:  tour is 3-opt
      COMMENT:        this is certainly not the best possible implementation of a 3-opt
                      local search algorithm. In addition, it is very lengthy; the main
		      reason herefore is that awkward way of making an exchange, where
		      it is tried to copy only the shortest possible part of a tour.
		      Whoever improves the code regarding speed or solution quality, please
		      drop me the code at stuetzle no@spam informatik.tu-darmstadt.de
		      The neighbourhood is scanned in random order (this need
                      not be the best possible choice). Concerning the speed-ups used
		      here consult, for example, Chapter 8 of
		      Holger H. Hoos and Thomas Stuetzle,
		      Stochastic Local Search---Foundations and Applications,
		      Morgan Kaufmann Publishers, 2004.
		      or some of the papers online available from David S. Johnson.
    */
    {
        int n = tour.size() - 1;

        /* In case a 2-opt move should be performed, we only need to store opt2_move = true,
           as h1, .. h4 are used in such a way that they store the indices of the correct move */

        int   c1, c2, c3;           /* cities considered for an exchange */
        int   s_c1, s_c2, s_c3;     /* successors of these cities        */
        int   p_c1, p_c2, p_c3;     /* predecessors of these cities      */
        int   pos_c1, pos_c2, pos_c3;     /* positions of cities c1, c2, c3    */
        int   i, j, h, g, l;
        int   improvement_flag, help;
        int   h1=0, h2=0, h3=0, h4=0, h5=0, h6=0; /* memorize cities involved in a move */
        double   diffs, diffp;
        int   between = false;
        int   opt2_flag;  /* = true: perform 2-opt move, otherwise none or 3-opt move */
        int   move_flag;  /*
			      move_flag = 0 --> no 3-opt move
			      move_flag = 1 --> between_move (c3 between c1 and c2)
			      move_flag = 2 --> not_between with successors of c2 and c3
			      move_flag = 3 --> not_between with predecessors of c2 and c3
			      move_flag = 4 --> cyclic move
			   */
        double gain, move_value, radius, add1, add2;
        double decrease_breaks;    /* Stores decrease by breaking two edges (a,b) (c,d) */
        int val[3];
        int n1, n2, n3;
        vector<int> pos(n);        /* positions of cities in tour */
        vector<int> dlb(n);        /* vector containing don't look bits */
        vector<int> h_tour(n);     /* help vector for performing exchange move */
        vector<int> hh_tour(n);    /* help vector for performing exchange move */
        vector<int> random_vector;

        for ( i = 0 ; i < n ; i++ ) {
            pos[tour[i]] = i;
            dlb[i] = false;
        }
        improvement_flag = true;
        random_vector = rnd.perm(n);

        while ( improvement_flag ) {
            move_value = 0;
            improvement_flag = false;

            for ( l = 0 ; l < n ; l++ ) {

                c1 = random_vector[l];
                if ( dlb[c1] )
                    continue;
                opt2_flag = false;

                move_flag = 0;
                pos_c1 = pos[c1];
                s_c1 = tour[pos_c1+1];
                if (pos_c1 > 0)
                    p_c1 = tour[pos_c1-1];
                else
                    p_c1 = tour[n-1];

                h = 0;    /* Search for one of the h-nearest neighbours */
                while ( h < nn_ls && h < n - 1 ) {

                    c2   = instance.nn_list[c1][h];  /* second city, determine its position */
                    pos_c2 = pos[c2];
                    s_c2 = tour[pos_c2+1];
                    if (pos_c2 > 0)
                        p_c2 = tour[pos_c2-1];
                    else
                        p_c2 = tour[n-1];

                    diffs = 0; diffp = 0;

                    radius = instance.distance[c1][s_c1];
                    add1   = instance.distance[c1][c2];

                    /* Here a fixed radius neighbour search is performed */
                    if ( radius > add1 ) {
                        decrease_breaks = - radius - instance.distance[c2][s_c2];
                        diffs =  decrease_breaks + add1 + instance.distance[s_c1][s_c2];
                        diffp =  - radius - instance.distance[c2][p_c2] +
                                 instance.distance[c1][p_c2] + instance.distance[s_c1][c2];
                    }
                    else
                        break;
                    if ( p_c2 == c1 )  /* in case p_c2 == c1 no exchange is possible */
                        diffp = 0;
                    if ( (diffs < move_value - EPS) || (diffp < move_value - EPS) ) {
                        improvement_flag = true;
                        if (diffs <= diffp) {
                            h1 = c1; h2 = s_c1; h3 = c2; h4 = s_c2;
                            move_value = diffs;
                            opt2_flag = true; move_flag = 0;
                            /*     	    goto exchange; */
                        } else {
                            h1 = c1; h2 = s_c1; h3 = p_c2; h4 = c2;
                            move_value = diffp;
                            opt2_flag = true; move_flag = 0;
                            /*     	    goto exchange; */
                        }
                    }
                    /* Now perform the innermost search */
                    g = 0;
                    while (g < nn_ls && g < n - 1) {

                        c3   = instance.nn_list[s_c1][g];
                        pos_c3 = pos[c3];
                        s_c3 = tour[pos_c3+1];
                        if (pos_c3 > 0)
                            p_c3 = tour[pos_c3-1];
                        else
                            p_c3 = tour[n-1];

                        if ( c3 == c1 ) {
                            g++;
                            continue;
                        }
                        else {
                            add2 = instance.distance[s_c1][c3];
                            /* Perform fixed radius neighbour search for innermost search */
                            if ( decrease_breaks + add1 < add2 ) {

                                if ( pos_c2 > pos_c1 ) {
                                    if ( pos_c3 <= pos_c2 && pos_c3 > pos_c1 )
                                        between = true;
                                    else
                                        between = false;
                                }
                                else if ( pos_c2 < pos_c1 )
                                    if ( pos_c3 > pos_c1 || pos_c3 < pos_c2 )
                                        between = true;
                                    else
                                        between = false;
                                else {
                                    assert(false);
                                    printf(" Strange !!, pos_1 %d == pos_2 %d, \n",pos_c1,pos_c2);
                                }

                                if ( between ) {
                                    /* We have to add edges (c1,c2), (c3,s_c1), (p_c3,s_c2) to get
                                       valid tour; it's the only possibility */

                                    gain = decrease_breaks - instance.distance[c3][p_c3] +
                                           add1 + add2 +
                                           instance.distance[p_c3][s_c2];

                                    /* check for improvement by move */
                                    if ( gain < move_value - EPS ) {
                                        improvement_flag = true; /* g = neigh_ls + 1; */
                                        move_value = gain;
                                        opt2_flag = false;
                                        move_flag = 1;
                                        /* store nodes involved in move */
                                        h1 = c1; h2 = s_c1; h3 = c2; h4 = s_c2; h5 = p_c3; h6 = c3;
                                        goto exchange;
                                    }
                                }
                                else {   /* not between(pos_c1,pos_c2,pos_c3) */

                                    /* We have to add edges (c1,c2), (s_c1,c3), (s_c2,s_c3) */

                                    gain = decrease_breaks - instance.distance[c3][s_c3] +
                                           add1 + add2 +
                                           instance.distance[s_c2][s_c3];

                                    if ( pos_c2 == pos_c3 ) {
                                        gain = 20000;
                                    }

                                    /* check for improvement by move */
                                    if ( gain < move_value - EPS ) {
                                        improvement_flag = true; /* g = neigh_ls + 1; */
                                        move_value = gain;
                                        opt2_flag = false;
                                        move_flag = 2;
                                        /* store nodes involved in move */
                                        h1 = c1; h2 = s_c1; h3 = c2; h4 = s_c2; h5 = c3; h6 = s_c3;
                                        goto exchange;
                                    }

                                    /* or add edges (c1,c2), (s_c1,c3), (p_c2,p_c3) */
                                    gain = - radius - instance.distance[p_c2][c2]
                                           - instance.distance[p_c3][c3] +
                                           add1 + add2 +
                                           instance.distance[p_c2][p_c3];

                                    if ( c3 == c2 || c2 == c1 || c1 == c3 || p_c2 == c1 ) {
                                        gain = 2000000;
                                    }

                                    if ( gain < move_value - EPS ) {
                                        improvement_flag = true;
                                        move_value = gain;
                                        opt2_flag = false;
                                        move_flag = 3;
                                        h1 = c1; h2 = s_c1; h3 = p_c2; h4 = c2; h5 = p_c3; h6 = c3;
                                        goto exchange;
                                    }

                                    /* Or perform the 3-opt move where no subtour inversion is necessary
                                       i.e. delete edges (c1,s_c1), (c2,p_c2), (c3,s_c3) and
                                       add edges (c1,c2), (c3,s_c1), (p_c2,s_c3) */

                                    gain = - radius - instance.distance[p_c2][c2] -
                                           instance.distance[c3][s_c3]
                                           + add1 + add2 + instance.distance[p_c2][s_c3];

                                    /* check for improvement */
                                    if ( gain < move_value - EPS ) {
                                        improvement_flag = true;
                                        move_value = gain;
                                        opt2_flag = false;
                                        move_flag = 4;
                                        improvement_flag = true;
                                        /* store nodes involved in move */
                                        h1 = c1; h2 = s_c1; h3 = p_c2; h4 = c2; h5 = c3; h6 = s_c3;
                                        goto exchange;
                                    }
                                }
                            }
                            else
                                g = nn_ls + 1;
                        }
                        g++;
                    }
                    h++;
                }
                if ( move_flag || opt2_flag ) {
                    exchange:
                    move_value = 0;

                    /* Now make the exchange */
                    if ( move_flag ) {
                        dlb[h1] = false; dlb[h2] = false; dlb[h3] = false;
                        dlb[h4] = false; dlb[h5] = false; dlb[h6] = false;
                        pos_c1 = pos[h1]; pos_c2 = pos[h3]; pos_c3 = pos[h5];

                        if ( move_flag == 4 ) {

                            if ( pos_c2 > pos_c1 )
                                n1 = pos_c2 - pos_c1;
                            else
                                n1 = n - (pos_c1 - pos_c2);
                            if ( pos_c3 > pos_c2 )
                                n2 = pos_c3 - pos_c2;
                            else
                                n2 = n - (pos_c2 - pos_c3);
                            if ( pos_c1 > pos_c3 )
                                n3 = pos_c1 - pos_c3;
                            else
                                n3 = n - (pos_c3 - pos_c1);

                            /* n1: length h2 - h3, n2: length h4 - h5, n3: length h6 - h1 */
                            val[0] = n1; val[1] = n2; val[2] = n3;
                            /* Now order the partial tours */
                            h = 0;
                            help = INT_MIN;
                            for ( g = 0; g <= 2; g++) {
                                if ( help < val[g] ) {
                                    help = val[g];
                                    h = g;
                                }
                            }

                            /* order partial tours according length */
                            if ( h == 0 ) {
                                /* copy part from pos[h4] to pos[h5]
                                   direkt kopiert: Teil von pos[h6] to pos[h1], it
                                   remains the part from pos[h2] to pos[h3] */
                                j = pos[h4];
                                h = pos[h5];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h) {
                                    i++;
                                    j++;
                                    if ( j  >= n )
                                        j = 0;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                /* First copy partial tour 3 in new position */
                                j = pos[h4];
                                i = pos[h6];
                                tour[j] = tour[i];
                                pos[tour[i]] = j;
                                while ( i != pos_c1) {
                                    i++;
                                    if ( i >= n )
                                        i = 0;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                    tour[j] = tour[i];
                                    pos[tour[i]] = j;
                                }

                                /* Now copy stored part from h_tour */
                                j++;
                                if ( j >= n )
                                    j = 0;
                                for ( i = 0; i<n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }
                            else if ( h == 1 ) {

                                /* copy part from pos[h6] to pos[h1]
                                   direkt kopiert: Teil von pos[h2] to pos[h3], it
                                   remains the part from pos[h4] to pos[h5] */
                                j = pos[h6];
                                h = pos[h1];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h) {
                                    i++;
                                    j++;
                                    if ( j  >= n )
                                        j = 0;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                /* First copy partial tour 3 in new position */
                                j = pos[h6];
                                i = pos[h2];
                                tour[j] = tour[i];
                                pos[tour[i]] = j;
                                while ( i != pos_c2) {
                                    i++;
                                    if ( i >= n )
                                        i = 0;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                    tour[j] = tour[i];
                                    pos[tour[i]] = j;
                                }

                                /* Now copy stored part from h_tour */
                                j++;
                                if ( j >= n )
                                    j = 0;
                                for ( i = 0; i<n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }
                            else if ( h == 2 ) {
                                /* copy part from pos[h2] to pos[h3]
                                   direkt kopiert: Teil von pos[h4] to pos[h5], it
                                   remains the part from pos[h6] to pos[h1] */
                                j = pos[h2];
                                h = pos[h3];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h) {
                                    i++;
                                    j++;
                                    if ( j  >= n )
                                        j = 0;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                /* First copy partial tour 3 in new position */
                                j = pos[h2];
                                i = pos[h4];
                                tour[j] = tour[i];
                                pos[tour[i]] = j;
                                while ( i != pos_c3) {
                                    i++;
                                    if ( i >= n )
                                        i = 0;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                    tour[j] = tour[i];
                                    pos[tour[i]] = j;
                                }

                                /* Now copy stored part from h_tour */
                                j++;
                                if ( j >= n )
                                    j = 0;
                                for ( i = 0; i<n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }
                        }
                        else if ( move_flag == 1 ) {

                            if ( pos_c3 < pos_c2 )
                                n1 = pos_c2 - pos_c3;
                            else
                                n1 = n - (pos_c3 - pos_c2);
                            if ( pos_c3 > pos_c1 )
                                n2 = pos_c3 - pos_c1 + 1;
                            else
                                n2 = n - (pos_c1 - pos_c3 + 1);
                            if ( pos_c2 > pos_c1 )
                                n3 = n - (pos_c2 - pos_c1 + 1);
                            else
                                n3 = pos_c1 - pos_c2 + 1;

                            /* n1: length h6 - h3, n2: length h5 - h2, n2: length h1 - h3 */
                            val[0] = n1; val[1] = n2; val[2] = n3;
                            /* Now order the partial tours */
                            h = 0;
                            help = INT_MIN;
                            for ( g = 0; g <= 2; g++) {
                                if ( help < val[g] ) {
                                    help = val[g];
                                    h = g;
                                }
                            }
                            /* order partial tours according length */

                            if ( h == 0 ) {

                                /* copy part from pos[h5] to pos[h2]
                                   (inverted) and from pos[h4] to pos[h1] (inverted)
                                   it remains the part from pos[h6] to pos[h3] */
                                j = pos[h5];
                                h = pos[h2];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h ) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j = n-1;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                j = pos[h1];
                                h = pos[h4];
                                i = 0;
                                hh_tour[i] = tour[j];
                                n2 = 1;
                                while ( j != h) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j = n-1;
                                    hh_tour[i] = tour[j];
                                    n2++;
                                }

                                j = pos[h4];
                                for ( i = 0; i< n2 ; i++ ) {
                                    tour[j] = hh_tour[i];
                                    pos[hh_tour[i]] = j;
                                    j++;
                                    if (j >= n)
                                        j = 0;
                                }

                                /* Now copy stored part from h_tour */
                                for ( i = 0; i< n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }
                            else if ( h == 1 ) {

                                /* copy part from h3 to h6 (wird inverted) erstellen : */
                                j = pos[h3];
                                h = pos[h6];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h) {
                                    i++;
                                    j--;
                                    if ( j  < 0 )
                                        j = n-1;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                j = pos[h6];
                                i = pos[h4];

                                tour[j] = tour[i];
                                pos[tour[i]] = j;
                                while ( i != pos_c1) {
                                    i++;
                                    j++;
                                    if ( j >= n)
                                        j = 0;
                                    if ( i >= n)
                                        i = 0;
                                    tour[j] = tour[i];
                                    pos[tour[i]] = j;
                                }

                                /* Now copy stored part from h_tour */
                                j++;
                                if ( j >= n )
                                    j = 0;
                                i = 0;
                                tour[j] = h_tour[i];
                                pos[h_tour[i]] = j;
                                while ( j != pos_c1 ) {
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                    i++;
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                }
                                tour[n] = tour[0];
                            }

                            else if ( h == 2 ) {

                                /* copy part from pos[h2] to pos[h5] and
                                   from pos[h3] to pos[h6] (inverted), it
                                   remains the part from pos[h4] to pos[h1] */
                                j = pos[h2];
                                h = pos[h5];
                                i = 0;
                                h_tour[i] =  tour[j];
                                n1 = 1;
                                while ( j != h ) {
                                    i++;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }
                                j = pos_c2;
                                h = pos[h6];
                                i = 0;
                                hh_tour[i] = tour[j];
                                n2 = 1;
                                while ( j != h) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j = n-1;
                                    hh_tour[i] = tour[j];
                                    n2++;
                                }

                                j = pos[h2];
                                for ( i = 0; i< n2 ; i++ ) {
                                    tour[j] = hh_tour[i];
                                    pos[hh_tour[i]] = j;
                                    j++;
                                    if ( j >= n)
                                        j = 0;
                                }

                                /* Now copy stored part from h_tour */
                                for ( i = 0; i< n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }
                        }
                        else if ( move_flag == 2 ) {

                            if ( pos_c3 < pos_c1 )
                                n1 = pos_c1 - pos_c3;
                            else
                                n1 = n - (pos_c3 - pos_c1);
                            if ( pos_c3 > pos_c2 )
                                n2 = pos_c3 - pos_c2;
                            else
                                n2 = n - (pos_c2 - pos_c3);
                            if ( pos_c2 > pos_c1 )
                                n3 = pos_c2 - pos_c1;
                            else
                                n3 = n - (pos_c1 - pos_c2);

                            val[0] = n1; val[1] = n2; val[2] = n3;
                            /* Determine which is the longest part */
                            h = 0;
                            help = INT_MIN;
                            for ( g = 0; g <= 2; g++) {
                                if ( help < val[g] ) {
                                    help = val[g];
                                    h = g;
                                }
                            }
                            /* order partial tours according length */

                            if ( h == 0 ) {

                                /* copy part from pos[h3] to pos[h2]
                                   (inverted) and from pos[h5] to pos[h4], it
                                   remains the part from pos[h6] to pos[h1] */
                                j = pos[h3];
                                h = pos[h2];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h ) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j = n-1;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                j = pos[h5];
                                h = pos[h4];
                                i = 0;
                                hh_tour[i] = tour[j];
                                n2 = 1;
                                while ( j != h ) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j = n-1;
                                    hh_tour[i] = tour[j];
                                    n2++;
                                }

                                j = pos[h2];
                                for ( i = 0; i<n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }

                                for ( i = 0; i < n2 ; i++ ) {
                                    tour[j] = hh_tour[i];
                                    pos[hh_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                                /*  	      getchar(); */
                            }
                            else if ( h == 1 ) {

                                /* copy part from pos[h2] to pos[h3] and
                                   from pos[h1] to pos[h6] (inverted), it
                                   remains the part from pos[h4] to pos[h5] */
                                j = pos[h2];
                                h = pos[h3];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h ) {
                                    i++;
                                    j++;
                                    if ( j >= n  )
                                        j = 0;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                j = pos[h1];
                                h = pos[h6];
                                i = 0;
                                hh_tour[i] = tour[j];
                                n2 = 1;
                                while ( j != h ) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j =  n-1;
                                    hh_tour[i] = tour[j];
                                    n2++;
                                }
                                j = pos[h6];
                                for ( i = 0; i<n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                for ( i = 0; i < n2 ; i++ ) {
                                    tour[j] = hh_tour[i];
                                    pos[hh_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }

                            else if ( h == 2 ) {

                                /* copy part from pos[h1] to pos[h6]
                                   (inverted) and from pos[h4] to pos[h5],
                                   it remains the part from pos[h2] to
                                   pos[h3] */
                                j = pos[h1];
                                h = pos[h6];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h ) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j = n-1;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                j = pos[h4];
                                h = pos[h5];
                                i = 0;
                                hh_tour[i] = tour[j];
                                n2 = 1;
                                while ( j != h ) {
                                    i++;
                                    j++;
                                    if ( j >= n  )
                                        j = 0;
                                    hh_tour[i] = tour[j];
                                    n2++;
                                }

                                j = pos[h4];
                                /* Now copy stored part from h_tour */
                                for ( i = 0; i<n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }

                                /* Now copy stored part from h_tour */
                                for ( i = 0; i < n2 ; i++ ) {
                                    tour[j] = hh_tour[i];
                                    pos[hh_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }
                        }
                        else if ( move_flag == 3 ) {

                            if ( pos_c3 < pos_c1 )
                                n1 = pos_c1 - pos_c3;
                            else
                                n1 = n - (pos_c3 - pos_c1);
                            if ( pos_c3 > pos_c2 )
                                n2 = pos_c3 - pos_c2;
                            else
                                n2 = n - (pos_c2 - pos_c3);
                            if ( pos_c2 > pos_c1 )
                                n3 = pos_c2 - pos_c1;
                            else
                                n3 = n - (pos_c1 - pos_c2);
                            /* n1: length h6 - h1, n2: length h4 - h5, n2: length h2 - h3 */

                            val[0] = n1; val[1] = n2; val[2] = n3;
                            /* Determine which is the longest part */
                            h = 0;
                            help = INT_MIN;
                            for ( g = 0; g <= 2; g++) {
                                if ( help < val[g] ) {
                                    help = val[g];
                                    h = g;
                                }
                            }
                            /* order partial tours according length */

                            if ( h == 0 ) {

                                /* copy part from pos[h2] to pos[h3]
                                   (inverted) and from pos[h4] to pos[h5]
                                   it remains the part from pos[h6] to pos[h1] */
                                j = pos[h3];
                                h = pos[h2];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h ) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j = n-1;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                j = pos[h2];
                                h = pos[h5];
                                i = pos[h4];
                                tour[j] = h4;
                                pos[h4] = j;
                                while ( i != h ) {
                                    i++;
                                    if ( i >= n )
                                        i = 0;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                    tour[j] = tour[i];
                                    pos[tour[i]] = j;
                                }
                                j++;
                                if ( j >= n )
                                    j = 0;
                                for ( i = 0; i < n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }
                            else if ( h == 1 ) {

                                /* copy part from pos[h3] to pos[h2]
                                   (inverted) and from  pos[h6] to pos[h1],
                                   it remains the part from pos[h4] to pos[h5] */
                                j = pos[h3];
                                h = pos[h2];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h ) {
                                    i++;
                                    j--;
                                    if ( j < 0  )
                                        j = n-1;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                j = pos[h6];
                                h = pos[h1];
                                i = 0;
                                hh_tour[i] = tour[j];
                                n2 = 1;
                                while ( j != h ) {
                                    i++;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                    hh_tour[i] = tour[j];
                                    n2++;
                                }

                                j = pos[h6];
                                for ( i = 0; i<n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }

                                for ( i = 0 ; i < n2 ; i++ ) {
                                    tour[j] = hh_tour[i];
                                    pos[hh_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }

                            else if ( h == 2 ) {

                                /* copy part from pos[h4] to pos[h5]
                                   (inverted) and from pos[h6] to pos[h1] (inverted)
                                   it remains the part from pos[h2] to pos[h3] */
                                j = pos[h5];
                                h = pos[h4];
                                i = 0;
                                h_tour[i] = tour[j];
                                n1 = 1;
                                while ( j != h ) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j = n-1;
                                    h_tour[i] = tour[j];
                                    n1++;
                                }

                                j = pos[h1];
                                h = pos[h6];
                                i = 0;
                                hh_tour[i] = tour[j];
                                n2 = 1;
                                while ( j != h ) {
                                    i++;
                                    j--;
                                    if ( j < 0 )
                                        j = n-1;
                                    hh_tour[i] = tour[j];
                                    n2++;
                                }

                                j = pos[h4];
                                /* Now copy stored part from h_tour */
                                for ( i = 0; i< n1 ; i++ ) {
                                    tour[j] = h_tour[i];
                                    pos[h_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                /* Now copy stored part from h_tour */
                                for ( i = 0; i< n2 ; i++ ) {
                                    tour[j] = hh_tour[i];
                                    pos[hh_tour[i]] = j;
                                    j++;
                                    if ( j >= n )
                                        j = 0;
                                }
                                tour[n] = tour[0];
                            }
                        }
                        else {
                            printf(" Some very strange error must have occurred !!!\n\n");
                            exit(0);
                        }
                    }
                    if (opt2_flag) {

                        /* Now perform move */
                        dlb[h1] = false; dlb[h2] = false;
                        dlb[h3] = false; dlb[h4] = false;
                        if ( pos[h3] < pos[h1] ) {
                            help = h1; h1 = h3; h3 = help;
                            help = h2; h2 = h4; h4 = help;
                        }
                        if ( pos[h3]-pos[h2] < n / 2 + 1) {
                            /* reverse inner part from pos[h2] to pos[h3] */
                            i = pos[h2]; j = pos[h3];
                            while (i < j) {
                                c1 = tour[i];
                                c2 = tour[j];
                                tour[i] = c2;
                                tour[j] = c1;
                                pos[c1] = j;
                                pos[c2] = i;
                                i++; j--;
                            }
                        }
                        else {
                            /* reverse outer part from pos[h4] to pos[h1] */
                            i = pos[h1]; j = pos[h4];
                            if ( j > i )
                                help = n - (j - i) + 1;
                            else
                                help = (i - j) + 1;
                            help = help / 2;
                            for ( h = 0 ; h < help ; h++ ) {
                                c1 = tour[i];
                                c2 = tour[j];
                                tour[i] = c2;
                                tour[j] = c1;
                                pos[c1] = j;
                                pos[c2] = i;
                                i--; j++;
                                if ( i < 0 )
                                    i = n - 1;
                                if ( j >= n )
                                    j = 0;
                            }
                            tour[n] = tour[0];
                        }
                    }
                }
                else {
                    dlb[c1] = true;
                }
            }
        }
    }

    Tour optimize(const tigersugar::Instance &instance, const Tour &originalTour, OptimizeMethod method) {
//        printf("Original: ");
//        FORE(it, originalTour.nodes) printf("%d ", *it); printf("\n");

        // Map to TSP problem
        TspProblem problem(instance, originalTour);
        vector<int> nodes;
        for (int i = 0; i < originalTour.length(); ++i) {
            if (i + 1 < originalTour.length()) {
                nodes.push_back(i);
            } else {
                nodes.push_back(0);
            }
        }

        // local search
        if (method == twoOptsMethod) {
            two_opt_first(nodes, problem);
        } else if (method == threeOptsMethod) {
            three_opt_first(nodes, problem);
        }

        // put the depot back to start position
        nodes.pop_back();
        rotate(nodes.begin(), find(nodes.begin(), nodes.end(), 0), nodes.end());
        nodes.push_back(0);

        // build new tour
        Tour tour;
        for (int x : nodes) {
            tour.append(originalTour[x]);
        }
//        printf("After: ");
//        FORE(it, tour.nodes) printf("%d ", *it); printf("\n");

        return tour;
    }

    void optimizeTour(const tigersugar::Instance &instance, tigersugar::Tour &tour,
                      OptimizeMethod method = threeOptsMethod) {
        tigersugar::Distance oldDistance = tour.distance(instance);
        vector<int> org = tour.points;

        tour.points.pop_back(); 
        tour.points.erase(tour.points.begin());

        Tour tmp = optimize(instance, Tour(tour.points), method);
        ensure(tmp.nodes.size() >= 2 && tmp.nodes.front() == 0 && tmp.nodes.back() == 0);
        tour.points = vector<int>(tmp.nodes.begin(), tmp.nodes.end());
        tigersugar::Distance newDistance = tour.distance(instance);

        if (newDistance > oldDistance + 1e-9) {
            for (int x : org) cerr << x << " ";
            cerr << '\n';
            for (int x : tour.points) cerr << x << " ";
            cerr << '\n';

            cerr << "???? " << newDistance << " " << oldDistance << '\n';
            exit(0);
        }

        ensure(newDistance <= oldDistance + 1e-9);
        
        // if (newDistance < oldDistance)
        //     fprintf(stderr, "TSP Optimize from %lf to %lf (%lf)\n",
        //             oldDistance, newDistance, newDistance - oldDistance);
    }
} // tsp_optimizer

#endif // TSP_OPTIMIZER
