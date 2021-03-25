/**
 * @file   CircularTrackSimParams.hpp
 * @author authaldo
 * @date   20.03.2021
 *
 * @brief  Parameters for the simulation of a circular track.
 */

#ifndef CIRCULAR_TRACK_SIM_PARAMS_HPP
#define CIRCULAR_TRACK_SIM_PARAMS_HPP

struct CircularTrackSimParams {
    unsigned int numConesPerCircle = 2;
    float innerRadius = 1.0f;
    float outerRadius = 2.0f;

    float deltaT = 1.0f;
    unsigned int historySize = 1;

    float fovAngle = 10;
    float fovRange = 10;

    float coneMeasStddev = 0;
    float coneDropProb = 0;

    float vehicleMeasStddev = 0;
};

#endif // CIRCULAR_TRACK_SIM_PARAMS_HPP
