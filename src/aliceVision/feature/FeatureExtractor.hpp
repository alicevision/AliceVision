// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/View.hpp>

namespace aliceVision {
namespace feature {

class FeatureExtractorViewJob
{
public:
    FeatureExtractorViewJob(const sfmData::View& view,
                            const std::string& outputFolder);

    ~FeatureExtractorViewJob();

    bool useGPU() const
    {
        return !_gpuImageDescriberIndexes.empty();
    }

    bool useCPU() const
    {
        return !_cpuImageDescriberIndexes.empty();
    }

    std::string getFeaturesPath(feature::EImageDescriberType imageDescriberType) const
    {
        return _outputBasename + "." + EImageDescriberType_enumToString(imageDescriberType) + ".feat";
    }

    std::string getDescriptorPath(feature::EImageDescriberType imageDescriberType) const
    {
        return _outputBasename + "." + EImageDescriberType_enumToString(imageDescriberType) + ".desc";
    }

    void setImageDescribers(
            const std::vector<std::shared_ptr<feature::ImageDescriber>>& imageDescribers);

    const sfmData::View& view() const
    {
        return _view;
    }

    std::size_t memoryConsuption() const
    {
        return _memoryConsuption;
    }

    const std::vector<std::size_t>& imageDescriberIndexes(bool useGPU) const
    {
        return useGPU ? _gpuImageDescriberIndexes
                      : _cpuImageDescriberIndexes;
    }

private:
    const sfmData::View& _view;
    std::size_t _memoryConsuption = 0;
    std::string _outputBasename;
    std::vector<std::size_t> _cpuImageDescriberIndexes;
    std::vector<std::size_t> _gpuImageDescriberIndexes;
};

class FeatureExtractor
{
public:

    explicit FeatureExtractor(const sfmData::SfMData& sfmData);
    ~FeatureExtractor();

    void setRange(int rangeStart, int rangeSize)
    {
      _rangeStart = rangeStart;
      _rangeSize = rangeSize;
    }

    void setMaxThreads(int maxThreads)
    {
      _maxThreads = maxThreads;
    }

    void setMasksFolder(const std::string& folder)
    {
      _masksFolder = folder;
    }

    void setOutputFolder(const std::string& folder)
    {
      _outputFolder = folder;
    }

    void addImageDescriber(std::shared_ptr<feature::ImageDescriber>& imageDescriber)
    {
      _imageDescribers.push_back(imageDescriber);
    }

    void process();

private:

    void computeViewJob(const FeatureExtractorViewJob& job, bool useGPU);

    const sfmData::SfMData& _sfmData;
    std::vector<std::shared_ptr<feature::ImageDescriber>> _imageDescribers;
    std::string _masksFolder;
    std::string _outputFolder;
    int _rangeStart = -1;
    int _rangeSize = -1;
    int _maxThreads = -1;
};

} // namespace feature
} // namespace aliceVision
