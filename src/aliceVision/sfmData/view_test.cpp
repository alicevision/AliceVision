
#include <boost/filesystem.hpp>
#include <aliceVision/sfmData/View.hpp>

#define BOOST_TEST_MODULE view

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
namespace fs = boost::filesystem;

BOOST_AUTO_TEST_CASE(View_Metadata)
{
  {
    sfmData::View view;

    view.addMetadata("key", "value");
    view.addMetadata("exif/2/FocalLength", "50");

    BOOST_CHECK(view.hasMetadata({"key"}));
    BOOST_CHECK_EQUAL(view.getMetadata({"key"}), "value");

    BOOST_CHECK(view.hasMetadata({"FocalLength"}));
    BOOST_CHECK_EQUAL(view.getMetadata({"FocalLength"}), "50");

    BOOST_CHECK(!view.hasMetadata({"DoesNotExist"}));
    BOOST_CHECK_EQUAL(view.getMetadata({"DoesNotExist"}), "");

    BOOST_CHECK_EQUAL(view.getMetadataFocalLength(), 50.0);
  }

  {
    sfmData::View view;

    view.addMetadata("Exif:FocalLength", "50");

    BOOST_CHECK(view.hasMetadata({"FocalLength"}));
    BOOST_CHECK_EQUAL(view.getMetadata({"focalLength"}), "50");

    BOOST_CHECK_EQUAL(view.getMetadataFocalLength(), 50.0);
  }

  {
      sfmData::View view;

      view.addMetadata("test:FOCALLENGTH", "50/10");

      BOOST_CHECK(view.hasMetadata({"FocalLength"}));
      BOOST_CHECK_EQUAL(view.getMetadata({"focalLength"}), "50/10");

      BOOST_CHECK_EQUAL(view.getMetadataFocalLength(), 5.0);
  }
}


BOOST_AUTO_TEST_CASE(View_ExposureValues)
{
    using namespace aliceVision::sfmData;

    // See https://en.wikipedia.org/wiki/Exposure_value#EV_as_an_indicator_of_camera_settings
    BOOST_CHECK_EQUAL(std::round(computeEV(60.0, 1.0, 100.0)), -6);
    BOOST_CHECK_EQUAL(std::round(computeEV(1.0, 8.0, 100.0)), 6);
    BOOST_CHECK_EQUAL(std::round(computeEV(1.0 / 500.0, 64.0, 100.0)), 21);
    BOOST_CHECK_EQUAL(std::round(computeEV(1.0 / 32000.0, 8.0, 100.0)), 21);
    // https://en.wikipedia.org/wiki/Sunny_16_rule
    BOOST_CHECK_EQUAL(std::round(computeEV(1.0 / 100.0, 16.0, 100.0)), 15);
    // https://en.wikipedia.org/wiki/Looney_11_rule
    BOOST_CHECK_EQUAL(std::round(computeEV(1.0 / 100.0, 11.0, 100.0)), 14);
}

