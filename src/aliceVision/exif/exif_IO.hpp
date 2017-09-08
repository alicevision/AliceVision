// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef EXIF_IO_HPP
#define EXIF_IO_HPP

#include <string>
#include <map>

namespace aliceVision {
namespace exif  {

class Exif_IO
{
  public:
    virtual size_t getWidth() const = 0;

    virtual size_t getHeight() const = 0;

    virtual float getFocal() const = 0;

    virtual std::string getBrand() const = 0;

    virtual std::string getModel() const = 0;

    virtual std::string getLensModel() const = 0;

    virtual std::string getImageUniqueID() const = 0;

    /// Open the file for checking and parsing
    virtual bool open( const std::string & sFileName ) = 0;

    /// Verify if the file has metadata
    virtual bool doesHaveExifInfo() const = 0;

    virtual std::string getSerialNumber() const = 0;

    virtual std::string getLensSerialNumber() const = 0;

    virtual std::string getExifDataString() const = 0;

    virtual std::map<std::string, std::string> getExifData () const = 0;

    /// File change date and time
    virtual std::string getDateTime() const = 0;

    /// Original file date and time (may not exist)
    virtual std::string getDateTimeOriginal() const = 0;

    /// Digitization date and time (may not exist)
    virtual std::string getDateTimeDigitized() const = 0;

    /// Sub-second time that original picture was taken
    virtual std::string getSubSecTimeOriginal() const = 0;
};

std::size_t computeUID(const Exif_IO& exifReader, const std::string& imageFilename);

} // namespace exif
} // namespace aliceVision
#endif //EXIF_IO_HPP

