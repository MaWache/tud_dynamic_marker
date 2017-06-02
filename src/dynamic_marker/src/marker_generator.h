#ifndef MARKER_GENERATOR_H
#define MARKER_GENERATOR_H
#include <opencv2/opencv.hpp>

class marker_generator
{
public:
    marker_generator();
    void generate_markers(std::string pathtofolder, int xresolution, int yresolution);
};

#endif // MARKER_GENERATOR_H
