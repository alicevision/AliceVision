# sensorDB

This module provides a database of camera sensor sizes indexed by camera brand and model name. It is used to initialize camera intrinsics when no calibration data is available.

## Overview

When processing images whose camera intrinsics are unknown, AliceVision can estimate the focal length from the sensor size stored in EXIF metadata combined with the sensor physical size from a database.

The `sensorDB` module reads a flat-text database of sensor specifications and provides a lookup interface.

## Datasheet

The `Datasheet` struct stores the entry for a single camera model:

```cpp
struct Datasheet {
    std::string _brand;    // Camera manufacturer (e.g. "Canon")
    std::string _model;    // Camera model name (e.g. "EOS 5D Mark IV")
    double _sensorWidth;   // Sensor width in millimetres
};
```

## Database

The sensor database is stored in `cameraSensors.db`, a plain-text file with one entry per line. The `parseDatabase` functions read this file and populate a list of `Datasheet` objects.

```cpp
#include <aliceVision/sensorDB/parseDatabase.hpp>

std::vector<aliceVision::sensorDB::Datasheet> db;
aliceVision::sensorDB::parseDatabase("/path/to/cameraSensors.db", db);

aliceVision::sensorDB::Datasheet result;
if (aliceVision::sensorDB::getInfo("Canon", "EOS 5D Mark IV", db, result))
{
    double sensorWidth = result._sensorWidth; // in mm
}
```
