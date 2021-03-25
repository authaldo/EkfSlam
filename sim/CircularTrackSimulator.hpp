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
#include <random>

#include "../models/SingleTrackModel.hpp"
#include "CircularTrackSimParams.hpp"
#include "util.hpp"

namespace sim {
    template<typename T>
    float sign(Coord<T> p1,  Coord<T> p2, Coord<T> p3) {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    }

    template<typename T>
    bool isPointInTriangle(Coord<T> pt,  Coord<T> v1, Coord<T> v2, Coord<T> v3) {
        float d1, d2, d3;
        bool has_neg, has_pos;

        d1 = sign(pt, v1, v2);
        d2 = sign(pt, v2, v3);
        d3 = sign(pt, v3, v1);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    template<typename T>
    class CircularTrackSimulator {
      public:
        explicit CircularTrackSimulator(CircularTrackSimParams params);

        /**
         * Getter for all simulated cones.
         * @return Vector with the positions of all simulated cones.
         */
        [[nodiscard]] std::vector<Coord<T>> getCones() const;

        /**
         * Getter for all cones of the inner circle.
         * @return Vector with the positions of the inner cones.
         */
        [[nodiscard]] std::vector<Coord<T>> getInnerCones() const;

        /**
         * Getter for all cones of the outer circle.
         * @return Vector with the positions of the outer cones.
         */
        [[nodiscard]] std::vector<Coord<T>> getOuterCones() const;

        /**
         * Proceeds with the next simulation step.
         */
        void nextSimulationStep();

        /**
         * Getter for the current vehicle pose.
         * @return Current vehicle pose.
         */
        [[nodiscard]] Pose<T> getCurrentVehiclePose() const;

        /**
         * Getter for the FoV triangle.
         * @return Triangle points (1st point is doubled, to close the polygon).
         */
        [[nodiscard]] std::vector<Coord<T>> getCurrentFoVPolygon() const;

        /**
         * Getter for the position history.
         * @return Vector with last n previous vehicle positions (n specified in params).
         */
        [[nodiscard]] std::vector<Coord<T>> getPositionHistory() const;

        /**
         * Getter for the positions of the currently visible cones.
         * @return Vector with the positions of currently visible cones.
         */
        [[nodiscard]] std::vector<Coord<T>> getVisibleCones() const;

        [[nodiscard]] std::vector<Coord<T>> getConeMeasurements() const;

        [[nodiscard]] Coord<T> getPositionMeasurement() const;

        [[nodiscard]] T getYawRateMeasurement() const;

      private:
        std::vector<Coord<T>> innerCones;
        std::vector<Coord<T>> outerCones;

        CircularTrackSimParams params;

        SingleTrackModel<T> model;

        std::deque<Coord<T>> positionHistory;

        std::default_random_engine coneNoiseGenerator;
        std::default_random_engine vehicleNoiseGenerator;

        static constexpr T DEFAULT_VELOCITY = 3;
    };

    template<typename T>
    CircularTrackSimulator<T>::CircularTrackSimulator(CircularTrackSimParams params) : params(params) {
        auto inc = static_cast<float>(2 * M_PI / params.numConesPerCircle);

        // calculation of the yaw rate necessary to drive a circle with given velocity and radius
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
        std::vector<Coord<T>> points;
        auto curPose = model.getCurrentPose();

        points.reserve(4);
        points.push_back(curPose.pos);

        points.push_back({std::cos(curPose.psi - degree2Radian(params.fovAngle)) * params.fovRange + curPose.pos.x,
                          std::sin(curPose.psi - degree2Radian(params.fovAngle)) * params.fovRange + curPose.pos.y});

        points.push_back({std::cos(curPose.psi + degree2Radian(params.fovAngle)) * params.fovRange + curPose.pos.x,
                          std::sin(curPose.psi + degree2Radian(params.fovAngle)) * params.fovRange + curPose.pos.y});

        points.push_back(curPose.pos);

        return points;
    }

    template<typename T>
    std::vector<Coord<T>> CircularTrackSimulator<T>::getPositionHistory() const {
        return std::vector<Coord<T>>(positionHistory.begin(), positionHistory.end());
    }

    template<typename T>
    std::vector<Coord<T>> CircularTrackSimulator<T>::getVisibleCones() const {
        auto limit = std::pow(params.fovRange, 2);
        auto curPos = model.getCurrentPose().pos;
        auto foVPoints = getCurrentFoVPolygon();
        std::vector<Coord<T>> visibleCones;

        for (const auto &c : getCones()) {
            if (isPointInTriangle(c, foVPoints.at(0), foVPoints.at(1), foVPoints.at(2))) {
                visibleCones.push_back(c);
            }
        }

        return visibleCones;
    }

    template<typename T>
    std::vector<Coord<T>> CircularTrackSimulator<T>::getConeMeasurements() const {
        auto cones = getVisibleCones();
        auto pos = model.getCurrentPose().pos;

        if (params.coneDropProb != 0) {
            // TODO: is this necessary (-> implement)
        }

        if (params.coneMeasStddev != 0) {
            std::normal_distribution<T> coneNoise(0, params.coneMeasStddev);

            for (auto &c : cones) {
                c += coneNoise(coneNoiseGenerator);
            }
        }

        for (auto &c : cones) {
            c -= pos;
        }

        return cones;
    }

    template<typename T>
    Coord<T> CircularTrackSimulator<T>::getPositionMeasurement() const {
        auto pos = getCurrentVehiclePose().pos;

        if (params.vehicleMeasStddev != 0) {
            std::normal_distribution<double> vehicleNoise(0, params.vehicleMeasStddev);

            pos.x += vehicleNoise(vehicleNoiseGenerator);
            pos.y += vehicleNoise(vehicleNoiseGenerator);
        }

        return pos;
    }
    template<typename T>
    T CircularTrackSimulator<T>::getYawRateMeasurement() const {
        std::normal_distribution<double> vehicleNoise(0, params.vehicleMeasStddev);

        auto yawRate = model.getCurrentState()[5];

        if (params.vehicleMeasStddev != 0) {
            std::normal_distribution<double> vehicleNoise(0, params.vehicleMeasStddev);

            yawRate += vehicleNoise(vehicleNoiseGenerator);
        }

        return yawRate;
    }
}


#endif // CIRCULAR_TRACK_SIMULATOR_H
