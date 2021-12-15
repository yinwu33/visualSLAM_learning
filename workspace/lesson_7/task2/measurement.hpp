#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP


struct Measurement {
    Measurement(int camera_index_, int point_index_,
                double u_, double v_)
                 : camera_index(camera_index_), point_index(point_index_), u(u_), v(v) {}

    int camera_index;
    int point_index;

    double u;
    double v;

};

#endif // MEASUREMENT_HPP