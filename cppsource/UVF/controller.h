#pragma once

#include "uvf.h"

int sign(float x);

tuple<float, float> control(node point, node target, vector<tuple<float, float>> obstacle_vector,
    float de, float kr, float d_min, float delta, float rho_min, float  v_max, float k_error);