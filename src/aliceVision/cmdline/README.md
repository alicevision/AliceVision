# cmdline

This module provides utilities for building command-line applications in AliceVision.

It includes macros and helper functions to simplify the setup of command-line tools with consistent error handling, logging, and option validation.

## Macros

### `ALICEVISION_COMMANDLINE_START` / `ALICEVISION_COMMANDLINE_END`

These macros wrap the main body of a command-line tool, providing:

- Automatic timing of the command execution
- Consistent error handling for `std::exception` and unknown exceptions
- Standardized success and failure return codes

```cpp
int main(int argc, char** argv)
{
    ALICEVISION_COMMANDLINE_START

    // ... parse options and run the algorithm ...

    ALICEVISION_COMMANDLINE_END
}
```

## Option Validation

The `optInRange<T>(min, max, opt_name)` helper returns a validation function that checks whether a command-line option value falls within a specified range. It integrates with Boost.Program_options:

```cpp
po::options_description params("Parameters");
params.add_options()
    ("threshold", po::value<float>()->notifier(aliceVision::optInRange(0.f, 1.f, "threshold")),
     "Threshold value in [0, 1].");
```

If the value is outside the range, a `boost::program_options::validation_error` is thrown with an appropriate message.
