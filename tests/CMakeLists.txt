
# Detection tests: run the app in console mode on the sample images (bin/box.png
# in bin/box_in_scene.png) with the features available in this build.
#   detect_*: the combination must detect the object
#   run_*:    the combination must not crash (it may not detect the object)
#   nn_*:     nearest neighbor strategies must detect the object
# Run them with "ctest" in the build directory, or "ctest -L detect" for a subset.

IF(OpenCV_VERSION VERSION_LESS "3.0.0")
   MESSAGE(STATUS "Tests are not available with OpenCV 2")
   RETURN()
ENDIF()

# Same lists as Feature2D/1Detector, Feature2D/2Descriptor and NearestNeighbor/1Strategy in Settings.h
SET(DETECTORS Dense Fast GFTT MSER ORB SIFT Star SURF BRISK AGAST KAZE AKAZE SuperPointTorch)
SET(DESCRIPTORS Brief ORB SIFT SURF BRISK FREAK KAZE AKAZE LUCID LATCH DAISY SuperPointTorch)
SET(NN_STRATEGIES Linear KDTree KMeans Composite Autotuned Lsh BruteForce)

# Features available in this build (see Settings.cpp)
SET(XFEATURES2D 0)
IF(OPENCV_XFEATURES2D_FOUND)
   SET(XFEATURES2D 1)
ENDIF()
SET(SIFT ${NONFREE})
IF(OpenCV_VERSION VERSION_GREATER_EQUAL "4.4.0" OR
   (OpenCV_VERSION VERSION_GREATER_EQUAL "3.4.11" AND OpenCV_VERSION VERSION_LESS "4.0.0"))
   SET(SIFT 1) # SIFT is in features2d since 4.4.0 / 3.4.11
ENDIF()
SET(MOVED_TO_XFEATURES2D 1) # BRISK, KAZE, AKAZE and AGAST moved to xfeatures2d in OpenCV 5
IF(OpenCV_VERSION VERSION_GREATER_EQUAL "5.0.0")
   SET(MOVED_TO_XFEATURES2D ${XFEATURES2D})
ENDIF()

SET(AVAILABLE_FEATURES Fast GFTT MSER ORB) # Dense is not available since OpenCV 3, SuperPointTorch needs a model
IF(SIFT)
   LIST(APPEND AVAILABLE_FEATURES SIFT)
ENDIF()
IF(NONFREE)
   LIST(APPEND AVAILABLE_FEATURES SURF)
ENDIF()
IF(MOVED_TO_XFEATURES2D)
   LIST(APPEND AVAILABLE_FEATURES BRISK AGAST KAZE AKAZE)
ENDIF()
IF(XFEATURES2D)
   LIST(APPEND AVAILABLE_FEATURES Star Brief FREAK LUCID LATCH DAISY)
ENDIF()

# Combinations detecting the object with a good margin of inliers (>= ~20, the minimum is 10).
SET(DETECTING_COMBINATIONS
   ORB:ORB SIFT:SIFT SURF:SURF BRISK:BRISK KAZE:KAZE AKAZE:AKAZE
   KAZE:SIFT BRISK:SIFT AKAZE:SIFT SURF:SIFT
   BRISK:SURF KAZE:SURF BRISK:FREAK BRISK:DAISY)

# Parameter values are "index:list", with the list separated by ';'
FUNCTION(GET_PARAMETER_VALUE VAR LIST_NAME ITEM)
   LIST(FIND ${LIST_NAME} ${ITEM} index)
   STRING(REPLACE ";" "\\;" items "${${LIST_NAME}}")
   SET(${VAR} "${index}:${items}" PARENT_SCOPE)
ENDFUNCTION()

# ADD_DETECTION_TEST(NAME DETECTOR DESCRIPTOR MUST_DETECT [NN_STRATEGY])
FUNCTION(ADD_DETECTION_TEST NAME DETECTOR DESCRIPTOR MUST_DETECT)
   GET_PARAMETER_VALUE(detector DETECTORS ${DETECTOR})
   GET_PARAMETER_VALUE(descriptor DESCRIPTORS ${DESCRIPTOR})
   SET(extraArgs "")
   IF(ARGC GREATER 4)
      GET_PARAMETER_VALUE(nnStrategy NN_STRATEGIES ${ARGV4})
      SET(extraArgs --NearestNeighbor/1Strategy "${nnStrategy}")
   ENDIF()
   ADD_TEST(NAME ${NAME}
      COMMAND find_object_app --console
         --object ${PROJECT_SOURCE_DIR}/bin/box.png
         --scene ${PROJECT_SOURCE_DIR}/bin/box_in_scene.png
         --Feature2D/1Detector ${detector}
         --Feature2D/2Descriptor ${descriptor}
         ${extraArgs})
   IF(MUST_DETECT)
      SET_TESTS_PROPERTIES(${NAME} PROPERTIES PASS_REGULAR_EXPRESSION "detected!")
   ENDIF()
   # An unavailable feature falls back to the default one, which would hide what is tested
   SET_TESTS_PROPERTIES(${NAME} PROPERTIES
      FAIL_REGULAR_EXPRESSION "Using default"
      TIMEOUT 120)
ENDFUNCTION()

FOREACH(detector ${DETECTORS})
   LIST(FIND AVAILABLE_FEATURES ${detector} detectorAvailable)
   IF(NOT detectorAvailable EQUAL -1)
      FOREACH(descriptor ${DESCRIPTORS})
         LIST(FIND AVAILABLE_FEATURES ${descriptor} descriptorAvailable)
         IF(NOT descriptorAvailable EQUAL -1)
            LIST(FIND DETECTING_COMBINATIONS "${detector}:${descriptor}" detecting)
            IF(detecting EQUAL -1)
               ADD_DETECTION_TEST(run_${detector}_${descriptor} ${detector} ${descriptor} FALSE)
               SET_TESTS_PROPERTIES(run_${detector}_${descriptor} PROPERTIES LABELS run)
            ELSE()
               ADD_DETECTION_TEST(detect_${detector}_${descriptor} ${detector} ${descriptor} TRUE)
               SET_TESTS_PROPERTIES(detect_${detector}_${descriptor} PROPERTIES LABELS detect)
            ENDIF()
         ENDIF()
      ENDFOREACH()
   ENDIF()
ENDFOREACH()

# Nearest neighbor strategies with binary (ORB) and float (SIFT) descriptors
SET(NN_FEATURES ORB)
IF(SIFT)
   LIST(APPEND NN_FEATURES SIFT)
ENDIF()
FOREACH(feature ${NN_FEATURES})
   FOREACH(strategy ${NN_STRATEGIES})
      ADD_DETECTION_TEST(nn_${feature}_${strategy} ${feature} ${feature} TRUE ${strategy})
      SET_TESTS_PROPERTIES(nn_${feature}_${strategy} PROPERTIES LABELS nn)
   ENDFOREACH()
ENDFOREACH()
