#pragma once

#include <iostream>
#include <math.h>
#include <tuple>
#include <vector>
#include <chrono>
#include "pch.h"

#ifndef M_PI
#define M_PI 3.1416
#endif

using namespace std;
typedef tuple<float, float, float> node;

float gauss(float r, float delta);

float HS(node point, float tx, float ty, float de, float kr, float ccw);

float TUF(node point, node target, float de, float kr);

tuple<float, tuple<float, float>, float> AUF(node point, vector<tuple<float, float>> obstacle_vector);

float UVF(node point, node target, vector<tuple<float, float>> obstacle_vector,
    float de, float kr, float d_min, float delta);
