# A helper function which allows setting a regular CMake option conditionally
#
# Why would I want this? Because cmake_dependent_option *hard* disables the
# option, even if the user has a legitimate reason to manually overwrite it.
# This function sets a reasonable default based on the given conditions, but
# allows the user to overwrite it using the CLI/TUI/GUI.
#
# Call signature:
# av_conditional_option(<OPT_NAME> <DESCRIPTION> IF <CONDITION> THEN <BOOL> ELSE <BOOL>)
function(av_conditional_option)

    # Parse first two unnamed args
    set(_co_OPT_NAME ${ARGV0})
    set(_opt_DESCRIPTION ${ARGV1})

    # Parse remaining args
    set(_co_OPTIONS)
    set(_co_OV_ARGS THEN ELSE)
    set(_co_MV_ARGS IF)
    cmake_parse_arguments(PARSE_ARGV 2 _co ${_co_OPTIONS} ${_co_OV_ARGS} ${_co_MV_ARGS})

    # Do not set if the variable is already defined (i.e., provided by the
    # command line)
    if(DEFINED ${_co_OPT_NAME})
        return()
    endif()

    # A condition given as a single quoted string (e.g. IF "A AND NOT B") arrives
    # as one list element, which if() would treat as the name of a variable.
    # Split it into separate arguments so that it is evaluated as an expression.
    # Multi-element conditions are left untouched, so quoted values inside them
    # (e.g. IF X STREQUAL "a b") keep their spaces.
    list(LENGTH _co_IF _co_IF_LEN)
    if(_co_IF_LEN EQUAL 1)
        separate_arguments(_co_IF)
    endif()
    if(${_co_IF})
        option(${_co_OPT_NAME} ${_opt_DESCRIPTION} ${_co_THEN})
    else()
        option(${_co_OPT_NAME} ${_opt_DESCRIPTION} ${_co_ELSE})
    endif()

endfunction()
