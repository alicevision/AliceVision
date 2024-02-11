#include <aliceVision/sfmData/View.hpp>

#define BOOST_TEST_MODULE view

#include <boost/test/unit_test.hpp>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(View_Metadata)
{
    {
        sfmData::View view;

        view.getImage().addMetadata("key", "value");
        view.getImage().addMetadata("exif/2/FocalLength", "50");

        BOOST_CHECK(view.getImage().hasMetadata({"key"}));
        BOOST_CHECK_EQUAL(view.getImage().getMetadata({"key"}), "value");

        BOOST_CHECK(view.getImage().hasMetadata({"FocalLength"}));
        BOOST_CHECK_EQUAL(view.getImage().getMetadata({"FocalLength"}), "50");

        BOOST_CHECK(!view.getImage().hasMetadata({"DoesNotExist"}));
        BOOST_CHECK_EQUAL(view.getImage().getMetadata({"DoesNotExist"}), "");

        BOOST_CHECK_EQUAL(view.getImage().getMetadataFocalLength(), 50.0);
    }

    {
        sfmData::View view;

        view.getImage().addMetadata("Exif:FocalLength", "50");

        BOOST_CHECK(view.getImage().hasMetadata({"FocalLength"}));
        BOOST_CHECK_EQUAL(view.getImage().getMetadata({"focalLength"}), "50");

        BOOST_CHECK_EQUAL(view.getImage().getMetadataFocalLength(), 50.0);
    }

    {
        sfmData::View view;

        view.getImage().addMetadata("test:FOCALLENGTH", "50/10");

        BOOST_CHECK(view.getImage().hasMetadata({"FocalLength"}));
        BOOST_CHECK_EQUAL(view.getImage().getMetadata({"focalLength"}), "50/10");

        BOOST_CHECK_EQUAL(view.getImage().getMetadataFocalLength(), 5.0);
    }

    {
        sfmData::View view;

        BOOST_CHECK_THROW(view.getImage().getGpsPositionFromMetadata(), std::out_of_range);

        for (const auto& t : sfmData::GPSExifTags::all())
        {
            BOOST_CHECK(!view.getImage().hasGpsMetadata());
            view.getImage().addMetadata(t, "100.6");
        }
        BOOST_CHECK(view.getImage().hasGpsMetadata());
    }
}
