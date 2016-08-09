#ifndef GETROUTE_HPP
#define GETROUTE_HPP
#include <math.h>
class point{
    public:
        float x;
        float y;
};

class routedata
{
	public:
    float dist_x;
    float dist_y;
    float dist_total;
};
void set_route_data(point*);

routedata  route(point* map_geo,int) ;
#endif
