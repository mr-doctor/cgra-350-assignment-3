set(GLM_VERSION "0.9.7")
set(GLM_INCLUDE_DIRS "/cygdrive/c/Users/Daniel/workspace/cgra350/cgra-350-assignment-3/work/vendor/glm")

if (NOT CMAKE_VERSION VERSION_LESS "3.0")
    include("${CMAKE_CURRENT_LIST_DIR}/glmTargets.cmake")
endif()
