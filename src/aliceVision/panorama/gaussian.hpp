#pragma once

#include <aliceVision/image/all.hpp>

namespace aliceVision
{

class GaussianPyramidNoMask
{
public:
    GaussianPyramidNoMask(const size_t width_base, const size_t height_base, const size_t limit_scales = 64);

    bool process(const image::Image<image::RGBfColor>& input);

    bool downscale(image::Image<image::RGBfColor>& output, const image::Image<image::RGBfColor>& input);

    const size_t getScalesCount() const { return _scales; }

    const std::vector<image::Image<image::RGBfColor>>& getPyramidColor() const { return _pyramid_color; }

    std::vector<image::Image<image::RGBfColor>>& getPyramidColor() { return _pyramid_color; }

protected:
    std::vector<image::Image<image::RGBfColor>> _pyramid_color;
    std::vector<image::Image<image::RGBfColor>> _filter_buffer;
    size_t _width_base;
    size_t _height_base;
    size_t _scales;
};

template <class T>
inline void convolveRow(typename image::Image<T>::RowXpr output_row, typename image::Image<T>::ConstRowXpr input_row,
                        const Eigen::Matrix<float, 5, 1>& kernel, bool loop)
{

    const int radius = 2;

    for(int j = 0; j < input_row.cols(); j++)
    {

        T sum = T();
        float sumw = 0.0f;

        for(int k = 0; k < kernel.size(); k++)
        {

            float w = kernel(k);
            int col = j + k - radius;

            /* mirror 5432 | 123456 | 5432 */

            if(!loop)
            {
                if(col < 0)
                {
                    col = -col;
                }

                if(col >= input_row.cols())
                {
                    col = input_row.cols() - 1 - (col + 1 - input_row.cols());
                }
            }
            else
            {
                if(col < 0)
                {
                    col = input_row.cols() + col;
                }

                if(col >= input_row.cols())
                {
                    col = col - input_row.cols();
                }
            }

            sum += w * input_row(col);
            sumw += w;
        }

        output_row(j) = sum / sumw;
    }
}

template <class T>
inline void convolveColumns(typename image::Image<T>::RowXpr output_row, const image::Image<T>& input_rows,
                            const Eigen::Matrix<float, 5, 1>& kernel)
{

    for(int j = 0; j < output_row.cols(); j++)
    {

        T sum = T();
        float sumw = 0.0f;

        for(int k = 0; k < kernel.size(); k++)
        {

            float w = kernel(k);
            sum += w * input_rows(k, j);
            sumw += w;
        }

        output_row(j) = sum / sumw;
    }
}

template <class T>
bool convolveGaussian5x5(image::Image<T>& output, const image::Image<T>& input, bool loop = false)
{

    if(output.size() != input.size())
    {
        return false;
    }

    Eigen::Matrix<float, 5, 1> kernel;
    kernel[0] = 1.0f;
    kernel[1] = 4.0f;
    kernel[2] = 6.0f;
    kernel[3] = 4.0f;
    kernel[4] = 1.0f;
    kernel = kernel / kernel.sum();

    image::Image<T> buf(output.Width(), 5);

    int radius = 2;

    convolveRow<T>(buf.row(0), input.row(2), kernel, loop);
    convolveRow<T>(buf.row(1), input.row(1), kernel, loop);
    convolveRow<T>(buf.row(2), input.row(0), kernel, loop);
    convolveRow<T>(buf.row(3), input.row(1), kernel, loop);
    convolveRow<T>(buf.row(4), input.row(2), kernel, loop);

    for(int i = 0; i < output.Height() - 3; i++)
    {

        convolveColumns<T>(output.row(i), buf, kernel);

        buf.row(0) = buf.row(1);
        buf.row(1) = buf.row(2);
        buf.row(2) = buf.row(3);
        buf.row(3) = buf.row(4);
        convolveRow<T>(buf.row(4), input.row(i + 3), kernel, loop);
    }

    /**
    current row : -5 -4 -3 -2 -1
    next 1 : -4 -3 -2 -1 -2
    next 2 : -3 -2 -1 -2 -3
    */
    convolveColumns<T>(output.row(output.Height() - 3), buf, kernel);

    buf.row(0) = buf.row(1);
    buf.row(1) = buf.row(2);
    buf.row(2) = buf.row(3);
    buf.row(3) = buf.row(4);
    convolveRow<T>(buf.row(4), input.row(output.Height() - 2), kernel, loop);
    convolveColumns<T>(output.row(output.Height() - 2), buf, kernel);

    buf.row(0) = buf.row(1);
    buf.row(1) = buf.row(2);
    buf.row(2) = buf.row(3);
    buf.row(3) = buf.row(4);
    convolveRow<T>(buf.row(4), input.row(output.Height() - 3), kernel, loop);
    convolveColumns<T>(output.row(output.Height() - 1), buf, kernel);

    return true;
}

} // namespace aliceVision