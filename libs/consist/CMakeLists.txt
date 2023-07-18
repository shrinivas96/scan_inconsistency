CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(consist)

SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

FIND_PACKAGE(BLAS REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Cholmod REQUIRED)
FIND_PACKAGE(G2O REQUIRED)
# Called by FindQGLViewer
# FIND_PACKAGE(Qt4)
FIND_PACKAGE(QGLViewer)
FIND_PACKAGE(OpenGL)
FIND_PACKAGE(Doxygen)

IF(QGLVIEWER_LIBRARY AND QT4_FOUND AND OPENGL_FOUND)
	MESSAGE(STATUS "Enabling GUI support")
	SET(WITH_GUI TRUE)
ELSE()
	MESSAGE(STATUS "Disabling GUI support")
	SET(WITH_GUI FALSE)
ENDIF()

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)

IF(DOXYGEN_FOUND)
    MESSAGE(STATUS "Enabling documentation build")
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    ADD_CUSTOM_TARGET(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM)
ELSE(DOXYGEN_FOUND)
    MESSAGE(STATUS "Disabling documentation build")
ENDIF(DOXYGEN_FOUND)

IF(WITH_GUI)
    SET(QT_USE_QTOPENGL TRUE)
    SET(QT_USE_QTXML TRUE)
	INCLUDE(${QT_USE_FILE})
    ADD_DEFINITIONS(${QT_DEFINITIONS})
    INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${QGLVIEWER_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
    SET(GUI_LIBRARIES
        ${QT_LIBRARIES}
        ${QGLVIEWER_LIBRARY}
        ${OPENGL_LIBRARIES})
ENDIF(WITH_GUI)

# include_directories(${PROJECT_SOURCE_DIR}/libs/g2o)


ADD_LIBRARY(consist STATIC
    src/consist/gridbisectionoptimizer.cpp
    src/consist/hypothesistester.cpp
    src/consist/inconsistencyse2.cpp
    src/consist/mapquality.cpp
    src/consist/probability.cpp
    src/consist/quadrilateral.cpp
    src/consist/support.cpp
    src/consist/visibility.cpp)

SET(shared_SOURCES
	src/shared/argumentparser.cpp)
	
SET(optimizer_SOURCES
	${shared_SOURCES}
    src/optimizer/optimizeroptions.cpp
    src/optimizer/optimizer.cpp)
    
SET(tester_SOURCES
	${shared_SOURCES}
    src/tester/tester.cpp
    src/tester/testeroptions.cpp)
	
IF(WITH_GUI)
	ADD_DEFINITIONS(-DGUI_SUPPORT)
	SET(tester_SOURCES
		${tester_SOURCES}
		src/tester/mapviewer.cpp
		src/tester/testerwindow.cpp)
	QT4_WRAP_UI(tester_HEADERS_FORMS
		forms/testerwindow.ui)
	QT4_WRAP_CPP(tester_HEADERS_MOC
		src/tester/mapviewer.h
		src/tester/testerwindow.h)
ENDIF(WITH_GUI)

ADD_EXECUTABLE(optimizer
    ${optimizer_SOURCES})
    
ADD_EXECUTABLE(tester
    ${tester_SOURCES}
    ${tester_HEADERS_FORMS}
    ${tester_HEADERS_MOC})

TARGET_LINK_LIBRARIES(optimizer
	consist
    ${BLAS_LIBRARIES}
    ${CHOLMOD_LIBRARY}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_TYPES_SLAM2D}
    ${G2O_TYPES_DATA}
    ${G2O_SOLVER_CHOLMOD})
    
TARGET_LINK_LIBRARIES(tester
	consist
    ${BLAS_LIBRARIES}
    ${GUI_LIBRARIES}
    ${CHOLMOD_LIBRARY}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_TYPES_SLAM2D}
    ${G2O_TYPES_DATA}
    ${G2O_SOLVER_CHOLMOD})

