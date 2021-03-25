/**
 * @file   structs.hpp
 * @author authaldo
 * @date   20.03.21
 *
 * Some utility structs for position and coords.
 */

#ifndef SIM_UTIL_HPP
#define SIM_UTIL_HPP

template<typename T>
struct Coord {
    T x;
    T y;

    Coord(T x, T y) : x(x), y(y) {}

    friend std::ostream& operator<< (std::ostream &out, const Coord &coord) {
        out << "[" << coord.x << ", " << coord.y << "]";

        return out;
    }

    void operator+= (const Coord<T> &c1) {
        x += c1.x;
        y += c1.x;
    }

    void operator-= (const Coord<T> &c1) {
        x -= c1.x;
        y -= c1.y;
    }
};

template<typename T>
struct Pose {
    Coord<T> pos;
    T psi;

    Pose(Coord<T> pos, T psi) : pos(pos), psi(psi) {}

    Pose(T x, T y, T psi) : pos({x, y}), psi(psi) {}

    friend std::ostream& operator<< (std::ostream &out, const Pose &pose) {
        out << "[" << pose.pos.x << ", " << pose.pos.y << ", " << pose.psi << "]";

        return out;
    }
};

template<typename T>
static T radian2Degree(T radian) {
    return static_cast<T>((radian / M_PI) * 180.0f);
}

template<typename T>
static T degree2Radian(T degree) {
    return static_cast<T>((degree / 180.0f) * M_PI);
}

template<typename T>
T normalizeAngle(T angle) {
    if (angle > 2*M_PI) {
        angle -= 2*M_PI;
    } else if (angle < 0) {
        angle += 2*M_PI;
    }

    return angle;
}

#endif // SIM_UTIL_HPP
