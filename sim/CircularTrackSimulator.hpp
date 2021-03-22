/**
 * @author authaldo
 * @date   19.03.2021
 *
 * @brief  Simulates a circular track.
 */

#ifndef CIRCULAR_TRACK_SIMULATOR_H
#define CIRCULAR_TRACK_SIMULATOR_H

#include <cmath>
#include <vector>
#include <deque>

#include "../models/SingleTrackModel.hpp"
#include "CircularTrackSimParams.hpp"
#include "util.hpp"

namespace sim {
    template<typename T>
    class CircularTrackSimulator {
      public:
        explicit CircularTrackSimulator(CircularTrackSimParams params);

        [[nodiscard]] std::vector<Coord<T>> getCones() const;
        [[nodiscard]] std::vector<Coord<T>> getInnerCones() const;
        [[nodiscard]] std::vector<Coord<T>> getOuterCones() const;

        void nextSimulationStep();
        [[nodiscard]] Pose<T> getCurrentVehiclePose() const;
        [[nodiscard]] std::vector<Coord<T>> getCurrentFoVPolygon() const;
        [[nodiscard]] std::vector<Coord<T>> getPositionHistory() const;

      private:
        std::vector<Coord<T>> innerCones;
        std::vector<Coord<T>> outerCones;

        CircularTrackSimParams params;

        SingleTrackModel<T> model;

        std::deque<Coord<T>> positionHistory;

        static constexpr T DEFAULT_VELOCITY = 3;
    };

    template<typename T>
    CircularTrackSimulator<T>::CircularTrackSimulator(CircularTrackSimParams params) : params(params) {
        auto inc = static_cast<float>(2 * M_PI / params.numConesPerCircle);

        // calulation of the yaw rate necessary to drive a circle with given velocity
        // and radius
        auto y = (params.outerRadius + params.innerRadius) / 2;
        T n = (2 * M_PI * y) / (DEFAULT_VELOCITY * params.deltaT);
        auto dpsi = -2*M_PI / (n * params.deltaT);

        Pose<T> initialPose = {0, y, 0};

        model = SingleTrackModel<T>(initialPose, DEFAULT_VELOCITY, dpsi);

        innerCones.reserve(params.numConesPerCircle);
        outerCones.reserve(params.numConesPerCircle);

        for (auto i = 0; i < params.numConesPerCircle; i++) {
            float angle = i * inc;

            // inner circle
            float x_in = params.innerRadius * std::sin(angle);
            float y_in = params.innerRadius * std::cos(angle);
            innerCones.emplace_back(x_in, y_in);

            // outer circle
            float x_out = params.outerRadius * std::sin(angle);
            float y_out = params.outerRadius * std::cos(angle);
            outerCones.emplace_back(x_out, y_out);
        }

        positionHistory.push_front(initialPose.pos);
    }

    template<typename T>
    std::vector<Coord<T>> CircularTrackSimulator<T>::getCones() const {
        std::vector<Coord<T>> cones;
        cones.reserve(innerCones.size() + outerCones.size());

        cones.insert(cones.end(), innerCones.begin(), innerCones.end());
        cones.insert(cones.end(), outerCones.begin(), outerCones.end());

        return cones;
    }

    template<typename T>
    std::vector<Coord<T>> CircularTrackSimulator<T>::getInnerCones() const {
        return innerCones;
    }

    template<typename T>
    std::vector<Coord<T>> CircularTrackSimulator<T>::getOuterCones() const {
        return outerCones;
    }

    template<typename T>
    void CircularTrackSimulator<T>::nextSimulationStep() {
        model.update(params.deltaT);

        // keep track of the position history
        positionHistory.push_front(model.getCurrentPose().pos);
        if (positionHistory.size() > params.historySize) {
            positionHistory.pop_back();
        }
    }

    template<typename T>
    Pose<T> CircularTrackSimulator<T>::getCurrentVehiclePose() const {
        return model.getCurrentPose();
    }

    template<typename T>
    std::vector<Coord<T>> CircularTrackSimulator<T>::getCurrentFoVPolygon() const {
        // TODO: implement
        return std::vector<Coord<T>>();
    }

    template<typename T>
    std::vector<Coord<T>> CircularTrackSimulator<T>::getPositionHistory() const {
        return std::vector<Coord<T>>(positionHistory.begin(), positionHistory.end());
    }
}


#endif // CIRCULAR_TRACK_SIMULATOR_H
