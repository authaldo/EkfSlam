#include <cfenv>
#include <chrono>
#include <iostream>
#include <matplotlibcpp.h>
#include <thread>

#include "EkfSlamManager.hpp"
#include "sim/CircularTrackSimParams.hpp"
#include "sim/CircularTrackSimulator.hpp"

template<typename T>
std::pair<std::vector<T>, std::vector<T>> separateCoords(const std::vector<Coord<T>> &input) {
    std::vector<T> coordsX, coordsY;
    coordsX.reserve(input.size());
    coordsY.reserve(input.size());

    for (auto &[x, y] : input) {
        coordsX.push_back(x);
        coordsY.push_back(y);
    }

    return std::make_pair(coordsX, coordsY);
}

int main() {
    namespace plt = matplotlibcpp;

    CircularTrackSimParams params;
    params.numConesPerCircle = 16;
    params.innerRadius = 4.0f;
    params.outerRadius = 8.0f;
    params.deltaT = 0.1f;
    params.historySize = 30;
    params.fovAngle = 50;
    params.fovRange = 5;

    sim::CircularTrackSimulator<float> sim(params);

    auto [innerConesX, innerConesY] = separateCoords(sim.getInnerCones());
    auto [outerConesX, outerConesY] = separateCoords(sim.getOuterCones());

    constexpr float CONE_DRAW_SIZE = 10.0f;
    constexpr float VEHICLE_DRAW_SIZE = 30.0f;

    for (unsigned int i = 0; i < 1000; i++) {
        sim.nextSimulationStep();

        auto vehiclePose = sim.getCurrentVehiclePose();

        auto [histX, histY] = separateCoords(sim.getPositionHistory());

        auto [fovX, fovY] = separateCoords(sim.getCurrentFoVPolygon());

        auto [visConesX, visConesY] = separateCoords(sim.getVisibleCones());

        // Visualization
        plt::clf();
        plt::xlim(-10, 10);
        plt::ylim(-10, 10);

        plt::title("EKF SLAM Simulation");

        plt::plot(fovX, fovY, {{"c", "cyan"}});

        plt::scatter(std::vector<float>{vehiclePose.pos.x}, std::vector<float>{vehiclePose.pos.y}, VEHICLE_DRAW_SIZE, {{"c", "red"}});

        plt::scatter(innerConesX, innerConesY, CONE_DRAW_SIZE, {{"c", "orange"}, {"marker", "^"}});
        plt::scatter(outerConesX, outerConesY, CONE_DRAW_SIZE, {{"c", "blue"}, {"marker", "^"}});
        plt::scatter(visConesX, visConesY, CONE_DRAW_SIZE, {{"c", "black"}, {"facecolors", "none"}, {"edgecolors", "black"}});

        plt::plot(histX, histY);

        plt::draw();
        plt::pause(0.1);

        // do not reopen the figure if the user has closed it
        if (!plt::fignum_exists(1)) {
            break;
        }
    }

    /*
    feenableexcept(FE_INVALID | FE_OVERFLOW | FE_DIVBYZERO); // Floating point exceptions
    auto dt = 0.1;

    ekf_slam::VehicleParams vehicleParams{1, 1, 1, 1};
    ekf_slam::ObjectParams objectParams{1, 1};
    ekf_slam::Manager manager{vehicleParams, objectParams};

    std::vector<ekf_slam::Manager::ObjectState> cones;
    cones.emplace_back(1, 1);
    cones.emplace_back(5, 5);
    ekf_slam::Manager::VehicleState vehicleState{0, 0, 0, 0, 0};
    auto f = ekf_slam::single_track_model::make<double>(dt, 0, 0, 0, 0).f;
    auto coneH = ekf_slam::constant_position_model::make<double>(0, 0).h;

    auto ddPsi = [](auto t) -> double {
        if (t < 1) {
            return 0;
        } else if (t < 2) {
            return 0.1;
        } else {
            return -0 - 1;
        }
    };

    auto a = [](auto t) -> double {
        if (t < 1) {
            return 1;
        } else {
            return 0;
        }
    };

    for (std::size_t c = 0; c < 1000; ++c) {
        vehicleState.dPsi += ddPsi(c * dt);
        vehicleState.v += a(c * dt);
        vehicleState =
                ekf_slam::Manager::VehicleState{f(static_cast<ekf_slam::Manager::VehicleState::Vec>(vehicleState))};
        ekf_slam::Manager::VehicleMeas vehicleMeas{vehicleState.v, vehicleState.dPsi};

        std::vector<ekf_slam::Manager::ObjectMeas> conesMeasured;
        for (auto cone : cones) {
            auto coneLocal = ekf_slam::Manager::ObjectMeas{
                    coneH(static_cast<ekf_slam::Manager::ObjectState::Vec>(cone),
                          static_cast<ekf_slam::Manager::VehicleState::Vec>(vehicleState))};
            if (coneLocal.xPos > 0) {
                conesMeasured.emplace_back(coneLocal);
            }
        }

        auto [vehicle, cones] = manager.update(vehicleMeas, conesMeasured, dt);
        std::cout << "State:" << static_cast<ekf_slam::Manager::VehicleState::Vec>(vehicleState).transpose()
                  << "\tMeas:" << static_cast<ekf_slam::Manager::VehicleMeas::Vec>(vehicleMeas).transpose()
                  << "\tEst:" << static_cast<ekf_slam::Manager::VehicleState::Vec>(vehicle).transpose()
                  << "\tNumber of Objects\t"
                  << cones.size() << std::endl;

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
    }*/
}
