/**
 * @file SingleTrackModel.hpp.h
 * @author paul
 * @date 17.03.21
 * Description here TODO
 */
#ifndef EKFSLAM_SINGLETRACKMODEL_HPP
#define EKFSLAM_SINGLETRACKMODEL_HPP

#include "Dynamic.hpp"

namespace ekf_slam::single_track_model {
    template<typename T>
    struct State {
        T xPos, yPos;
        T v;
        T psi, dPsi;

        static constexpr std::size_t DIM = 5;
        using Vec = Eigen::Matrix<T, DIM, 1>;
        using Mat = Eigen::Matrix<T, DIM, DIM>;

        State(T xPos, T yPos, T v, T psi, T dPsi) : xPos{xPos}, yPos{yPos}, v{v}, psi{psi}, dPsi{dPsi} {};

        explicit State(Vec x) : xPos{x(0)}, yPos{x(1)}, v{x(2)}, psi{x(3)}, dPsi{x(4)} {};

        explicit operator Vec() {
            Vec ret{};
            ret(0) = xPos;
            ret(1) = yPos;
            ret(2) = v;
            ret(3) = psi;
            ret(4) = dPsi;
            return ret;
        }
    };

    template<typename T>
    struct Meas {
        T v, dPsi;

        static constexpr std::size_t DIM = 2;
        using Vec = Eigen::Matrix<T, DIM, 1>;
        using Mat = Eigen::Matrix<T, DIM, DIM>;

        Meas(T v, T dPsi) : v{v}, dPsi{dPsi} {};

        explicit Meas(Vec z) : v(z(0)), dPsi(z(1)){};

        explicit operator Vec() {
            Vec ret{};
            ret(0) = v;
            ret(1) = dPsi;
            return ret;
        }
    };

    template<typename T>
    auto make(const T &dt, T sigmaA2, T sigmaDDPsi2, T sigmaV2, T sigmaDPsi2) {
        auto f = [&dt](auto x) -> typename State<T>::Vec {
            State<T> state{x};
            // clang-format off
            State<T> newState{state.xPos + std::cos(state.psi) * state.v * dt,
                              state.yPos + std::sin(state.psi) * state.v * dt,
                              state.v,
                              state.psi + state.dPsi * dt,
                              state.dPsi};
            // clang-format on
            return static_cast<typename State<T>::Vec>(newState);
        };

        auto J_F = [&dt](auto x) -> typename State<T>::Mat {
            State<T> state{x};
            typename State<T>::Mat j_f;
            // clang-format off
            j_f <<
                1, 0, std::cos(state.psi) * dt, -std::sin(state.psi) * state.v *dt, 0,
                0, 1, std::sin(state.psi) * dt, std::cos(state.psi) * state.v * dt , 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, dt,
                0, 0, 0, 0, 1;
            // clang-format on
            return j_f;
        };

        auto Q_func = [&dt, sigmaA2, sigmaDDPsi2](auto x) -> typename State<T>::Mat {
            State<T> state{x};
            Eigen::Vector3d GammaA;
            // clang-format off
            GammaA <<
                    0.5 * dt * dt * std::cos(state.psi),
                    0.5 * dt * dt * std::sin(state.psi),
                    dt;
            // clang-format on
            Eigen::Vector2d GammaDDPsi;
            // clang-format off
            GammaDDPsi <<
                    0.5 * dt * dt,
                    dt;
            // clang-format on
            auto Q_a = GammaA * GammaA.transpose() * sigmaA2;
            auto Q_DDPsi = GammaDDPsi * GammaDDPsi.transpose() * sigmaDDPsi2;
            Eigen::Matrix<T, 5, 5> Q;
            Q.block(0, 0, 3, 3) = Q_a;
            Q.block(3, 3, 2, 2) = Q_DDPsi;

            return Q;
        };

        auto h = [](auto x, auto /*empty*/) -> typename Meas<T>::Vec {
            State<T> state{x};
            Meas<T> meas{state.v, state.psi};
            return static_cast<typename Meas<T>::Vec>(meas);
        };

        auto J_H = [](auto x, auto /*empty*/) -> Eigen::Matrix<T, Meas<T>::DIM, State<T>::DIM> {
            Eigen::Matrix<T, Meas<T>::DIM, State<T>::DIM> c = Eigen::Matrix<T, Meas<T>::DIM, State<T>::DIM>::Zero();
            c <<
                    // clang-format off
                    0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1;
            // clang-format on
            return c;
        };

        auto R_func = [sigmaV2, sigmaDPsi2]() -> typename Meas<T>::Mat {
            Eigen::Matrix<T, Meas<T>::DIM, Meas<T>::DIM> R = Eigen::Matrix<T, Meas<T>::DIM, Meas<T>::DIM>::Zero();
            R <<
                    // clang-format off
                    sigmaV2, 0,
                    0, sigmaDPsi2;
            // clang-format on
            return R;
        };

        return Dynamic<State<T>::DIM, Meas<T>::DIM, EmptyType, T>{f, J_F, Q_func, h, J_H, R_func};
    }
} // namespace ekf_slam::single_track_model

#endif // EKFSLAM_SINGLETRACKMODEL_HPP
