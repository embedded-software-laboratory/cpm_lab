#include "calibration_floor_plane.hpp"



void calibration_floor_plane(
    double image_x,
    double image_y,
    double& world_x,
    double& world_y
)
{ 
    image_x /= 2048;
    image_y /= 2048;

    double ix1 = image_x;
    double ix2 = ix1 * image_x;
    double ix3 = ix2 * image_x;
    double ix4 = ix3 * image_x;

    double iy1 = image_y;
    double iy2 = iy1 * image_y;
    double iy3 = iy2 * image_y;
    double iy4 = iy3 * image_y;

    double features[] = {  
        1, ix1, iy1, ix2, ix1 * iy1, iy2, ix3, ix2 * iy1, ix1 * iy2, iy3,
        ix4, ix3 * iy1, ix2 * iy2, ix1 * iy3, iy4};

    double calibration_x[] = {4.351151e+00, -4.863260e+00, -1.183271e-01, 1.094383e+00, 4.861627e-01, 1.418471e-01, -6.700942e-01, 3.049470e-01, -4.362105e-01, 1.852765e-01, -7.676803e-02, -7.289893e-02, -1.313038e-01, -8.945004e-02, -5.608838e-02};
    double calibration_y[] = {-3.409322e-01, 5.226075e-01, 4.909991e+00, -3.901540e-01, -5.209485e-01, -1.093096e+00, 6.016393e-02, 5.101132e-01, -3.097603e-01, 5.542172e-01, -2.875253e-02, 5.073364e-02, 1.783163e-01, 1.005458e-01, 1.114947e-01};


    world_x = 0;
    world_y = 0;
    for (int i = 0; i < 15; ++i)
    {
        world_x += features[i] * calibration_x[i];
        world_y += features[i] * calibration_y[i];
    }
}
