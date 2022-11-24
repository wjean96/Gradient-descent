#ifndef __STRUCT_TYPE_HPP__
#define __STRUCT_TYPE_HPP__

#include <vector>

typedef struct _POINT{
    float x;
    float y;
    float z;
}_POINT;

typedef struct ST_RAMDOMPOINT{
    std::vector<_POINT> st_point;
    unsigned int nPoint;
}ST_RAMDOMPOINT;



#endif