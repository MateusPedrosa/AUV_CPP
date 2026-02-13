find_path(Unwind_INCLUDE_DIR NAMES libunwind.h PATHS /usr/include /usr/include/libunwind)
find_library(Unwind_LIBRARY NAMES unwind)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Unwind DEFAULT_MSG Unwind_LIBRARY Unwind_INCLUDE_DIR)
