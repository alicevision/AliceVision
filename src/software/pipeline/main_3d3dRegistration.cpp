// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#include <aliceVision/registration/PointcloudRegistration.hpp>

#include <pcl/console/parse.h>

using namespace aliceVision;
using namespace aliceVision::registration;

void printHelp()
{
	pcl::console::print_error("Syntax is:\n"
		"\t-s <source file (moving)>\n"
		"\t-t <target file (fixed)>\n"
		"Optional:\n"
		"\t--sourceMeasurements <source measurements>\n"
		"\t--targetMeasurements <target measurements>\n"
		"\t--scaleRatio <scale ratio (target measur. / source measur.)>\n"
		"\t--voxelSize <voxel size>\n");
}

int main(int argc, char** argv)
{
	std::string sourceFile = "";
	std::string targetFile = "";
	
	if (pcl::console::find_switch(argc, argv, "-s") && pcl::console::find_switch(argc, argv, "-t")) // source file
	{
		pcl::console::parse(argc, argv, "-s", sourceFile);
		pcl::console::parse(argc, argv, "-t", targetFile);
	}
	else
	{
		printHelp();
		return EXIT_FAILURE;
	}

	float sourceMeasurements = 1.f;
	if (pcl::console::find_switch(argc, argv, "--sourceMeasurements"))
		pcl::console::parse(argc, argv, "--sourceMeasurements", sourceMeasurements);
	
	float targetMeasurements = 1.f;
	if (pcl::console::find_switch(argc, argv, "--targetMeasurements"))
		pcl::console::parse(argc, argv, "--targetMeasurements", targetMeasurements);

	float scaleRatio = 1.f;
	if (pcl::console::find_switch(argc, argv, "--scaleRatio"))
		pcl::console::parse(argc, argv, "--scaleRatio", scaleRatio);

	float voxelSize = 0.1f;
	if (pcl::console::find_switch(argc, argv, "--voxelSize"))
		pcl::console::parse(argc, argv, "--voxelSize", voxelSize);

	// ===========================================================
	// -- Run alignement
	// ===========================================================

	PointcloudRegistration pa;

	if (pa.loadSourceCloud(sourceFile) == EXIT_FAILURE)
    return EXIT_FAILURE;
	
	if (pa.loadTargetCloud(targetFile) == EXIT_FAILURE)
    return EXIT_FAILURE;
	
	if (pa.setSourceMeasurements(sourceMeasurements) == EXIT_FAILURE)
		return EXIT_FAILURE;

	if (pa.setTargetMeasurements(targetMeasurements) == EXIT_FAILURE)
		return EXIT_FAILURE;

	pa.setVoxelSize(voxelSize);
	pa.setVerbose(true);
	pa.setShowPipeline(true);

	pcl::PointCloud<pcl::PointXYZ> regGICP_source;

	pa.align(regGICP_source);

	Eigen::Matrix4f T = pa.getFinalTransformation();
	std::cout << "Transformation (such as: T * source = target) =\n" << T << std::endl;
	
	return EXIT_SUCCESS;
}
