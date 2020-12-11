####################
# LinkOpenCV.cmake #
####################

TARGET_LINK_LIBRARIES(${targetname} ${OpenCV_LIBS})

IF(MSVC_IDE)
  FILE(GLOB RUNTIMELIBS "${_OpenCV_LIB_PATH}/*.dll")
ENDIF()

FOREACH(RUNTIMELIB ${RUNTIMELIBS})
  ADD_CUSTOM_COMMAND(TARGET ${targetname} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different ${RUNTIMELIB} "$<TARGET_FILE_DIR:${targetname}>")
ENDFOREACH()
