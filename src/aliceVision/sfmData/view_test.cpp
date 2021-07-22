
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

    {
        sfmData::View view;

        BOOST_CHECK_THROW(view.getGpsPositionFromMetadata(), std::out_of_range);

        for(const auto& t : sfmData::GPSExifTags::all())
        {
            BOOST_CHECK(!view.hasGpsMetadata());
            view.addMetadata(t, "100.6");
        }
        BOOST_CHECK(view.hasGpsMetadata());
    }
}

