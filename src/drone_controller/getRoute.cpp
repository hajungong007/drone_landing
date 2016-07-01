#include "getRoute.hpp"


void set_route_data(point * map_indoor){
//map_indoor[0].y = 0 ;map_indoor[0].x = 0;
map_indoor[0].y = - 1.82;map_indoor[0].x = 1.83;
map_indoor[1].y=  3.22 ;map_indoor[1].x= 0.14 ;
//map_indoor[1].y= 0;map_indoor[1].x=2.37; 
map_indoor[2].y = - 1.62 ;map_indoor[2].x = 0.60;
map_indoor[3].y = - 1.55 ;map_indoor[3].x = 1.87;
map_indoor[4].y = 2.33   ;map_indoor[4].x = 2.90;
map_indoor[5].y = 0.61   ;map_indoor[5].x = 2.805;
map_indoor[6].y = 0.12   ;map_indoor[6].x = 4.85;
map_indoor[7].y = -2.7   ;map_indoor[7].x = -0.49;
map_indoor[8].y = 1.11 ;map_indoor[8].x = -3.50;
//map_indoor[9].y = - 2.85 ;map_indoor[9].x = 5.49;
    
}
routedata route(point* map_geo,int a){ //getRoute from 2 points.
       routedata route_calculated;
       route_calculated.dist_x=map_geo[a].x;
       route_calculated.dist_y=map_geo[a].y;
        route_calculated.dist_total=sqrt(route_calculated.dist_x*route_calculated.dist_x+route_calculated.dist_y*route_calculated.dist_y);
        return route_calculated;

}
