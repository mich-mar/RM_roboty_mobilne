#ifndef POINTS_HPP
#define POINTS_HPP

struct Point2D {
    int x;
    int y;
};

// Robot bases: points from (-8, -8) to (-8, 7)
const Point2D robot_bases[16] = {
    {-8, -8}, {-8, -7}, {-8, -6}, {-8, -5}, {-8, -4}, {-8, -3}, {-8, -2}, {-8, -1},
    {-8,  0}, {-8,  1}, {-8,  2}, {-8,  3}, {-8,  4}, {-8,  5}, {-8,  6}, {-8,  7}
};

// Pickup stations: points from (-2, -9) to (7, -9)
const Point2D pickup_stations[10] = {
    {-2, -9}, {-1, -9}, { 0, -9}, { 1, -9}, { 2, -9},
    { 3, -9}, { 4, -9}, { 5, -9}, { 6, -9}, { 7, -9}
};

// Delivery stations: points from (-2, 9) to (7, 9)
const Point2D delivery_stations[10] = {
    {-2, 9}, {-1, 9}, { 0, 9}, { 1, 9}, { 2, 9},
    { 3, 9}, { 4, 9}, { 5, 9}, { 6, 9}, { 7, 9}
};

#endif // POINTS_HPP
