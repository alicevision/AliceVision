// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (module="pyalicevision") image

%{
  #define SWIG_FILE_WITH_INIT
%}

%include <aliceVision/numeric/numpy.i>
%include <std_string.i>


%init
%{
  import_array();
%}

%{    
#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/io.hpp>
 
using namespace aliceVision;
using namespace aliceVision::image;

template <typename T> 
int NumPyType() 
{
    return -1;
}

template<> 
int NumPyType<float>() 
{
    return NPY_FLOAT;
}

template<> 
int NumPyType<unsigned char>() 
{
    return NPY_UINT8;
}

template<> 
int NumPyType<image::RGBAfColor>() 
{
    return NPY_FLOAT;
}

template<> 
int NumPyType<image::RGBAColor>() 
{
    return NPY_UINT8;
}

template<> 
int NumPyType<image::RGBfColor>() 
{
    return NPY_FLOAT;
}

template<> 
int NumPyType<image::RGBColor>() 
{
    return NPY_UINT8;
}

%} 

%include <OpenImageIO/oiioversion.h>
%include <aliceVision/image/colorspace.hpp> 
%include <aliceVision/image/Image.hpp>
%include <aliceVision/image/io.hpp> 


%fragment("NumPy_Fragments");
%extend aliceVision::image::Image
{
    PyObject * getNumpyArray()
    {
        aliceVision::image::Image<T> & image = *$self;

        npy_intp dims[3] = {image.height(), image.width(), NbChannels<T>::size};
        PyObject * ret = PyArray_SimpleNew(3, dims, NumPyType<T>());
        if (!ret)
        {
            return nullptr;
        }

        T * data = static_cast<T*>(PyArray_DATA((PyArrayObject*)ret));
        for (int i = 0; i != dims[0]; ++i)
        {
            for (int j = 0; j != dims[1]; ++j)
            {
                data[i*dims[1]+j] = image(i,j);
            }
        }

        return ret;
    }

    bool fromNumpyArray(PyObject * obj)
    {
        // Check object type
        if (!is_array(obj))
        {
            PyErr_SetString(PyExc_ValueError, "The given input is not known as a NumPy array or matrix.");
            return false;
        }
        
        if (array_type(obj) != NumPyType<T>())
        {
            PyErr_SetString(PyExc_ValueError, "Type mismatch between NumPy and Eigen objects.");
            return false;
        }
        
        if (array_numdims(obj) != 3)
        {
            PyErr_SetString(PyExc_ValueError, "Image requires a 3D numpy array");
            return false;
        }

        int rows = array_size(obj, 0);
        int cols = array_size(obj, 1);
        int channels = array_size(obj, 2);

        if (channels != NbChannels<T>::size)
        {
            PyErr_SetString(PyExc_ValueError, "Numpy array 3rd dimension in incorrect.");
            return false;
        }

        aliceVision::image::Image<T> & image = *$self;
        image.resize(cols, rows, false);

        const T * const data = static_cast<const T * const>(PyArray_DATA((PyArrayObject*)obj));
        for (int i = 0; i != rows; ++i)
        {
            for (int j = 0; j != cols; ++j)
            {
                image(i,j) = data[i*cols+j];
            }
        }

        return true;
    }
}

%{
class oiioParams
{
    public:

    oiioParams() {}
    oiioParams(int orientation, float pixelAspectRatio, std::string compression = "")
    {
        _myParamList["Orientation"] = orientation;
        _myParamList["pixelAspectRatio"] = pixelAspectRatio;
        if (compression != "")
        {
            _myParamList["compression"] = compression;
        }
    }

    const oiio::ParamValueList & get()
    {
        return _myParamList;
    }

    private:

    oiio::ParamValueList _myParamList;
};

%}
class oiioParams
{
    public:

    oiioParams();
    oiioParams(int orientation, float pixelAspectRatio, std::string compression = "");

    const oiio::ParamValueList & get();

    private:

    oiio::ParamValueList _myParamList;
};

%template(Image_float) aliceVision::image::Image<float>; 
%template(Image_uchar) aliceVision::image::Image<unsigned char>; 
%template(Image_RGBAfColor) aliceVision::image::Image<RGBAfColor>; 
%template(Image_RGBAColor) aliceVision::image::Image<RGBAColor>;
%template(Image_RGBfColor) aliceVision::image::Image<RGBfColor>; 
%template(Image_RGBColor) aliceVision::image::Image<RGBColor>;