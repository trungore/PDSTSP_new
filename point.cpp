#ifndef DHK_LIB_POINT
#define DHK_LIB_POINT

#include "template.cpp"

namespace Point
{
    const double RRR = 6378.388;
    const double PI = 3.14159265359;

    struct point {
        double x = 0;
        double y = 0;

        point(const double& _x, const double& _y)
        {
            x = _x;
            y = _y;
        }

        friend istream& operator >> (istream& is, point& p)
        {
            is >> p.x >> p.y;
            return is;

        }

    };

    int euc_2d_distance(const point& p1, const point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return (int) round(sqrt(dx * dx + dy * dy));

    }

    int ceil_2d_distance(const point& p1, const point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return (int) ceil(sqrt(dx * dx + dy * dy));

    }

    int geo_distance(const point& p1, const point& p2)
    {
        double d, m;

        d = trunc(p1.x);
        m = p1.x - d;
        double latitude1 = PI * (d + 5.0 * m / 3.0) / 180.0;

        d = trunc(p2.x);
        m = p2.x - d;
        double latitude2 = PI * (d + 5.0 * m / 3.0) / 180.0;

        d = trunc(p1.y);
        m = p1.y - d;
        double longtitude1 = PI * (d + 5.0 * m / 3.0) / 180.0;

        d = trunc(p2.y);
        m = p2.y - d;
        double longtitude2 = PI * (d + 5.0 * m / 3.0) / 180.0;

        double q1 = cos(longtitude1 - longtitude2);
        double q2 = cos(latitude1 - latitude2);
        double q3 = cos(latitude1 + latitude2);
        return (int) (RRR * acos(0.5 * ((1.0 + q1) * q2 - (1.0 - q1) * q3)) + 1.0);

    }

    int att_distance(const point& p1, const point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return ceil(sqrt((dx * dx + dy * dy) / 10.0));

    }

    double euclidean_distance(const point& p1, const point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return sqrt(dx*dx + dy*dy);
    }

    double manhattan_distance(const point& p1, const point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return abs(dx) + abs(dy);
    }

}

#endif