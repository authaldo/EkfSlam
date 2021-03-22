/**
 * @file   SingleTrackModel.hpp
 * @author authaldo
 * @date   19.03.2021
 *
 * @brief  Definition of the linear single track model.
 */

#ifndef MODELS_SINGLE_TRACK_MODEL_HPP
#define MODELS_SINGLE_TRACK_MODEL_HPP

#include "../sim/util.hpp"

template<typename T>
class SingleTrackModel {
    using State = Eigen::Matrix<T, 5, 1>;

  public:
    /**
     * Initializes the internal state vector to zeros.
     */
    SingleTrackModel();

    /**
     * Initializes the single track model with the given pose, velocity and yaw rate.
     * @param initialPose    Initial pose (x, y, psi (in radian)).
     * @param initialVel     Initial velocity.
     * @param initialYawRate Initial yaw rate.
     */
    SingleTrackModel(Pose<T> initialPose, T initialVel, T initialYawRate);

    /**
     * Prediction of the internal state at time t + dt.
     * @param dt                  Time increment.
     * @param acceleration        Acceleration (applied before prediction).
     * @param yawRateAcceleration Yaw rate acceleration (applied before prediction).
     */
    void update(T dt, T acceleration = 0, T yawRateAcceleration = 0);

    /**
     * Getter for the current state vector.
     * @return Current model state (x, y, v, psi, dpsi).
     *
     * @note Angle psi is given in radian.
     */
    State getCurrentState() const;

    /**
     * Getter for the current pose.
     * @return Pose (x, y, psi).
     */
    Pose<T> getCurrentPose() const;

  private:
    State state;

    static constexpr T EPSILON = 1e-4;
    static constexpr unsigned int X = 0;
    static constexpr unsigned int Y = 1;
    static constexpr unsigned int V = 2;
    static constexpr unsigned int PSI = 3;      // given in radian
    static constexpr unsigned int D_PSI = 4;
};

template<typename T>
SingleTrackModel<T>::SingleTrackModel() {
    for (auto i = 0U; i < 5; i++) {
        state(i) = 0;
    }
}

template<typename T>
SingleTrackModel<T>::SingleTrackModel(Pose<T> initialPose, T initialVel, T initialYawRate) {
    state(X) = initialPose.pos.x;
    state(Y) = initialPose.pos.y;
    state(V) = initialVel;
    state(PSI) = initialPose.psi;
    state(D_PSI) = initialYawRate;
}

template<typename T>
void SingleTrackModel<T>::update(T dt, T acceleration, T yawRateAcceleration) {
    // apply acceleration and yaw rate acceleration
    state(V) += acceleration * dt;
    state(D_PSI) += yawRateAcceleration * dt;

    // case distinction based on yaw rate:
    if (abs(state(D_PSI)) > EPSILON) {
        state(X) += (state(V) / state(D_PSI)) * (std::sin(normalizeAngle(state(PSI) + dt * state(D_PSI))) - std::sin(state(PSI)));
        state(Y) += (state(V) / state(D_PSI)) * (std::cos(state(PSI)) - std::cos(normalizeAngle(state(PSI) + dt * state(D_PSI))));
        state(PSI) += state(D_PSI) * dt;
    } else {
        state(X) += state(V) * dt * std::cos(state(PSI));
        state(Y) += state(V) * dt * std::sin(state(PSI));
        state(PSI) += state(D_PSI) * dt;
    }

    state(PSI) = normalizeAngle(state(PSI));
}

template<typename T>
typename SingleTrackModel<T>::State SingleTrackModel<T>::getCurrentState() const {
    return state;
}

template<typename T>
Pose<T> SingleTrackModel<T>::getCurrentPose() const {
    Pose<T> pose(state(X), state(Y), state(PSI));

    return pose;
}


#endif // MODELS_SINGLE_TRACK_MODEL_HPP
