#pragma once

#include <openMVG/image/image.hpp> //TODO : class forward declaration
#include <openMVG/voctree/vocabulary_tree.hpp> //TODO : class forward declaration
#include <openMVG/features/features.hpp> //TODO : class forward declaration
#include <openMVG/dataio/FeedProvider.hpp>

#include <OpenImageIO/imageio.h>  
#include <string>
#include <vector>
#include <memory>
#include <limits>

namespace oiio = OpenImageIO;

namespace openMVG {
namespace keyframe {

class KeyframeSelector
{
private:
    // descriptor definition
    const static std::size_t _dimension = 128;
    using DescriptorFloat = openMVG::features::Descriptor<float, _dimension>;
  
public:
    
    /**
     * @brief KeyframeSelector constructor
     * @param mediaFilePath[in] video file path or image sequence directory
     * @param sensorDbPath[in] camera sensor width database path
     * @param voctreeFilePath[in] vocabulary tree path
     * @param outputDirectory[in] output keyframes directory
     */
    KeyframeSelector(const std::string& mediaFilePath,
                        const std::string& sensorDbPath,
                        const std::string& voctreeFilePath,
                        const std::string& outputDirectory);
    
    /**
     * @brief KeyframeSelector copy constructor - NO COPY
     * @param copy[in] keyframeSelector
     */
    KeyframeSelector(const KeyframeSelector& copy) = delete;
    
    
    /**
     * @brief Process media path and extract keyframes
     */
    void process();
    
    
    /**
     * @brief Set camera informations for output keyframes metadatas
     * @param brand[in] camera brand
     * @param model[in] camera model
     * @param focalLength[in] focal length (mm or px)
     * @param focalIsMM[in] is focal length in mm
     */
    void setCameraInfo(const std::string& brand, const std::string& model, float focalLength, bool focalIsMM = true)
    {
        _brand = brand;
        _model = model;
        _focalLength = focalLength;
        _focalIsMM = focalIsMM;
    }
         
    /**
     * @brief Set sharp subset size for process algorithm
     * @param subset[in] sharp part of the image (1 = all, 2 = size/2, ...)
     */
    void setSharpSubset(unsigned int subset)
    {
        _sharpSubset = subset;
    }
    
    /**
     * @brief Set min frame step for process algorithm
     * @param frameStep[in] minimum number of frame between two keyframes
     */
    void setMinFrameStep(unsigned int frameStep)
    {
        _minFrameStep = frameStep;
    }
    
    /**
     * @brief Set max frame step for process algorithm
     * @param frameStep[in] maximum number of frame for evaluation
     */
    void setMaxFrameStep(unsigned int frameStep)
    {
        _maxFrameStep = frameStep;
    }
    
    /**
     * @brief Set max output frame number for process algorithm
     * @param nbFrame[in] maximum number of output frame (if 0, no limit)
     */
    void setMaxOutFrame(unsigned int nbFrame)
    {
        _maxOutFrame = nbFrame;
    }
    
    /**
     * @brief Get sharp subset size for process algorithm
     * @return sharp part of the image (1 = all, 2 = size/2, ...)
     */
    unsigned int getSharpSubset() const 
    {
        return _sharpSubset;
    }
        
    /**
     * @brief Get min frame step for process algorithm
     * @return minimum number of frame between two keyframes
     */
    unsigned int getMinFrameStep() const 
    {
        return _minFrameStep;
    }
            
    /**
     * @brief Get max output frame number for process algorithm
     * @return maximum number of frame for evaluation
     */
    unsigned int getMaxFrameStep() const 
    {
        return _maxFrameStep;
    }
    
    /**
     * @brief Get max output frame number for process algorithm
     * @return maximum number of output frame (if 0, no limit)
     */
    unsigned int getMaxOutFrame() const 
    {
        return _maxOutFrame;
    }
    
private:
    
    // paths

    std::string _mediaFilePath;    // media path
    std::string _sensorDbPath;     // camera sensor width database
    std::string _voctreeFilePath;  // SIFT voctree file path
    std::string _outputDirectory;  // output folder for keyframes
    
    // algorithm variables

    unsigned int _sharpSubset = 4; 
    unsigned int _minFrameStep = 12;
    unsigned int _maxFrameStep = 36;
    unsigned int _maxOutFrame = 0;
    unsigned int _nbTileSide = 20;
    unsigned int _nbKeyFrameDist = 10; // number of distance from previous keyframes to evaluate distance score
    float _sharpnessThreshold = 15.0f; // image with higher sharpness will be selected
    float _distScoreMax = 100.0f; // image with smallest distance from the last keyframe will be selected

    // camera metadata
    
    std::string _brand = "Pinhole";
    std::string _model = "Pinhole";
    float _focalLength = 1.0f;
    bool _focalIsMM = true;
  
    // tools 
    
    std::unique_ptr<features::Image_describer> _imageDescriber; // to extract describer
    std::unique_ptr< openMVG::voctree::VocabularyTree<DescriptorFloat> > _voctree; // to compute sparseHistogram
    dataio::FeedProvider _feed; // to extract image from media path 
    
    // process structure
    
    struct frameData {
        float sharpness = 0;
        float distScore = std::numeric_limits<float>::max();
        bool selected = false;
        bool keyframe = false;
        std::unique_ptr<voctree::SparseHistogram> histogram;
    };
    std::vector<frameData> _selectionData; 
    std::vector<size_t> _keyframeIndexes;
    
    /**
     * @brief Compute sharpness and distance score in order to select best frame
     * @param image[in] an image of the media
     * @param frameIndex[in] the image index in the media sequence
     * @param tileSharpSubset[in] number of sharp tile
     * @param tileHeight[in] tile height in px 
     * @param tileWidth[in] tile width in px
     */
    void computeFrameData(const image::Image<image::RGBColor>& image, 
                            size_t frameIndex, 
                            unsigned int tileSharpSubset,
                            unsigned int tileHeight,
                            unsigned int tileWidth);
    
    /**
     * @brief Write a keyframe and metadata
     * @param image[in] an image of the media
     * @param spec[in] image informations
     * @param frameIndex[in] the image index in the media sequence
     */
    void writeKeyframe(const image::Image<image::RGBColor>& image, 
                        const oiio::ImageSpec& spec,
                        size_t frameIndex);
    
    /**
     * @brief Convert focal length from px to mm using sensor width database
     * @param imageWidth[in] media image width in px
     */
    void convertFocalLengthInMM(int imageWidth);
};

} // namespace keyframe 
} // namespace openMVG