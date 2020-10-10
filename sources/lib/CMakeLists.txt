##
## Copyright 2020 kenken. All rights reserved.
##

#### Include ####

# Low-level modules
#add_subdirectory(common)
add_subdirectory(types)
add_subdirectory(utils)
add_subdirectory(camera)

# High-level modules
add_subdirectory(feature_tracking)
add_subdirectory(initial_estimation)
add_subdirectory(subspace_gauss_newton)
add_subdirectory(optimization)

# Final package
add_subdirectory(vitamin_e)

# File I/O
add_subdirectory(video)
add_subdirectory(calibration)

# Visualization
add_subdirectory(visualization)

# Test
if(BUILD_TEST)
	add_subdirectory(test_tool)
endif()