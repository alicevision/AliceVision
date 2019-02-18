#pragma once

#include "CombinedSolverBase.hpp"
#include "SolverIteration.hpp"

#include <aliceVision/image/convertion.hpp>
#include <aliceVision/mvsData/Color.hpp>

#include <cuda_runtime.h>


namespace aliceVision {
namespace intrinsicDecomposition {

class IntrinsicDecompositionSolver : public CombinedSolverBase
{
public:
    IntrinsicDecompositionSolver(unsigned int width, unsigned int height, const SolverParams& params)
    {
        m_solverParams = params;

        std::vector<unsigned int> dims = { width, height };
        _albedo = createEmptyOptImage(dims, OptImage::Type::FLOAT, 3, OptImage::GPU, true);
        _shading = createEmptyOptImage(dims, OptImage::Type::FLOAT, 1, OptImage::GPU, true);
        _normals = createEmptyOptImage(dims, OptImage::Type::FLOAT, 3, OptImage::GPU, true);
        _image = createEmptyOptImage(dims, OptImage::Type::FLOAT, 3, OptImage::GPU, true);

		const std::string sourceCodePath = __FILE__;
        std::size_t sepIndex = sourceCodePath.find_last_of("/\\");
        std::string sourceCodeFolder = sourceCodePath.substr(0, sepIndex);

        addOptSolvers(dims,
                      sourceCodeFolder + "/intrinsicImageDecomposition.opt",
        	          m_solverParams.optDoublePrecision);
	}

    void combinedSolveInit() override
    {
        m_problemParams.set("albedo", _albedo);
        m_problemParams.set("shading", _shading);
        m_problemParams.set("normals", _normals);
        m_problemParams.set("image", _image);

        m_problemParams.set("pNorm", &_pNorm);
        m_problemParams.set("w_regAlbedo", &_w_regAlbedo);
        m_problemParams.set("w_smoothShading", &_w_smoothShading);
        m_problemParams.set("w_dataShading", &_w_dataShading);

        for (int i = 0; i < 9; ++i)
        {
            char buff[5];
            sprintf(buff, "L_%d", i+1);
            m_problemParams.set(buff, (void*)&(_lightingCoefficients[i]));
        }

        m_solverNamedParams.set("nIterations", &m_solverParams.nonLinearIter);
        m_solverNamedParams.set("lIterations", &m_solverParams.linearIter);
    }
    void preSingleSolve() override {}
    void postSingleSolve() override {}
    void preNonlinearSolve(int) override {}
    void postNonlinearSolve(int) override {}
    void combinedSolveFinalize() override
    {
        reportFinalCosts("Intrinsic Image Decomposition", m_solverParams, getCost("Opt(GN)"), getCost("Opt(LM)"));
    }

    void setImage(const std::vector<Color>& image, const std::vector<Color>& normals, unsigned int w, unsigned int h)
	{
		//std::vector<float3> h_imageFloat3Albedo(w*h);
		std::vector<float>  h_imageFloatIllumination(w*h);

		for (unsigned int y = 0; y < h; ++y)
		{
			for (unsigned int x = 0; x < w; ++x)
			{
				int index = y * w + x;
                const Color& c = image.at(index);
				//h_imageFloat3Albedo[index] = make_float3(c.r, c.g, c.b);
                h_imageFloatIllumination.at(index) = 0.5; // aliceVision::image::Rgb2Gray(c.r, c.g, c.b);
			}
		}
        _image->update(image);
        // _albedo->update(h_imageFloat3Albedo);
        _albedo->update(image);
        _normals->update(normals);
        _shading->update(h_imageFloatIllumination);
	}

	void retrieveImages(std::vector<float>& shading, std::vector<Color>& albedo) const
	{
        _shading->copyTo(shading);
        _albedo->copyTo(albedo);
	}

private:
    float _w_regAlbedo = 1.0f;
    float _w_smoothShading = 1.0f;
    float _w_dataShading = 1.0f;
    float _pNorm = 0.8f;
    std::array<float, 9> _lightingCoefficients = {{1.f,0.f,0.f, 0.f,0.f,0.f, 0.f,0.f,0.f}};

    std::shared_ptr<OptImage> _image;
    std::shared_ptr<OptImage> _albedo;
    std::shared_ptr<OptImage> _normals;
    std::shared_ptr<OptImage> _shading;
};

}
}
