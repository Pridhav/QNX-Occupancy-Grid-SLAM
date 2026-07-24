#ifndef MAP_COMMON_H
#define MAP_COMMON_H

#include <stdint.h>

#define MAP_SIZE 200    // 200x200 grid
#define RESOLUTION 0.05 // 5cm per cell
#define OCCUPIED 100
#define FREE 0

typedef struct {
    int8_t grid[MAP_SIZE][MAP_SIZE];
    float origin_x;
    float origin_y;
} occupancy_map_t;

#endif
