#ifndef SMORPHI_HPP
#define SMORPHI_HPP

#include <iostream>
#include <vector>
#include <cmath>

class SmorphiShape
{
private:
    std::string current_shape;

    // Shape vertices
    std::vector<std::vector<double>> i_shape;
    std::vector<std::vector<double>> o_shape;

    // Laser offsets
    double i_offset_laser_x;
    double i_offset_laser_y;
    double i_offset_laser_theta;
    double o_offset_laser_x;
    double o_offset_laser_y;
    double o_offset_laser_theta;

    // Shape offsets
public:
    double i_shape_x;
    double i_shape_y;
    double o_shape_x;
    double o_shape_y;

public:
    SmorphiShape()
        : current_shape("i"),
          i_shape{{-0.35, 0.09}, {0.35, 0.09}, {0.35, -0.09}, {-0.35, -0.09}, {-0.35, 0.09}},
          o_shape{{-0.18, 0.18}, {0.18, 0.18}, {0.18, -0.18}, {-0.18, -0.18}, {-0.18, 0.18}},
          i_offset_laser_x(-0.18), i_offset_laser_y(0.0), i_offset_laser_theta(0.0),
          o_offset_laser_x(0.0), o_offset_laser_y(0.18), o_offset_laser_theta(M_PI),
          i_shape_x(-0.09), i_shape_y(0.0),
          o_shape_x(0.09), o_shape_y(0.09)
    {
    }

    void setShape(const std::string &shape)
    {
        if (shape == "i" || shape == "o")
        {
            current_shape = shape;
        }
        else
        {
            std::cerr << "Invalid shape specified. Using default shape: " << current_shape << std::endl;
        }
    }

    std::string getCurrentShape() const
    {
        return current_shape;
    }

    std::vector<std::vector<double>> getShapeScaled() const
    {
        std::vector<std::vector<double>> shape = current_shape == "i" ? i_shape : o_shape;
        double offset_x = current_shape == "i" ? i_shape_x : o_shape_x;
        double offset_y = current_shape == "i" ? i_shape_y : o_shape_y;

        for (auto &vertex : shape)
        {
            vertex[0] *= 0.8;
            vertex[1] *= 0.8;
        }

        return shape;
    }

    std::vector<std::vector<double>> getShape() const
    {
        std::vector<std::vector<double>> shape = current_shape == "i" ? i_shape : o_shape;
        double offset_x = current_shape == "i" ? i_shape_x : o_shape_x;
        double offset_y = current_shape == "i" ? i_shape_y : o_shape_y;

        // for (auto &vertex : shape)
        // {
        //     vertex[0] += offset_x;
        //     vertex[1] += offset_y;
        // }

        return shape;
    }

    std::vector<double> getLaserOffset() const
    {
        if (current_shape == "i")
        {
            return {i_offset_laser_x, i_offset_laser_y, i_offset_laser_theta};
        }
        else
        {
            return {o_offset_laser_x, o_offset_laser_y, o_offset_laser_theta};
        }
    }
    double x_offset()
    {
        if (current_shape == "i")
        {
            return i_shape_x;
        }
        else
        {
            return o_shape_x;
        }
    }
    double y_offset()
    {
        if (current_shape == "i")
        {
            return i_shape_y;
        }
        else
        {
            return o_shape_y;
        }
    }
};

#endif
