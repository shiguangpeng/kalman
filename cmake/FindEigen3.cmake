#  Eigen3_INCLUDE_DIR
#  Eigen3_FOUND

find_path( Eigen3_INCLUDE_DIR Eigen/Core
#    /usr/include/eigen3
#    /usr/local/include/eigen3
    "C:/Program Files (x86)/Eigen3/include/eigen3"
    "C:/Program Files (x86)/Eigen3/include/eigen3/unsupported"
)


include(LibFindMacros)
set(Eigen3_PROCESS_INCLUDES Eigen3_INCLUDE_DIR)
libfind_process(Eigen3)
