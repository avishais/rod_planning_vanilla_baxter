#compiler
OMPL_DIR = /usr/local
INC_CLASSES = ../proj_classes/
INC_PLANNERS = ../planners/
INC_VALIDITY = ../validity_checkers/

GL_INCPATH = -I/usr/include/
GL_LIBPATH = -L/usr/lib/ -L/usr/X11R6/lib/
GL_LIBS = -lGLU -lGL -lXext -lXmu -lXi -lX11 -lglut

PQP_DIR= /home/avishai/Documents/PQP_v1.3/

EIGEN_DIR = /home/avishai/Documents/eigen
KDL_DIR = /usr/local

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${INC_CLASSES} -I${INC_PLANNERS} -I${PQP_DIR}/include $(GL_INCPATH) -I${KDL_DIR}/include  -I$(EIGEN_DIR) 
LDFLAGS=  -L${OMPL_DIR}/lib -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app_base -lompl_app -lboost_filesystem -lboost_system -lboost_serialization -lboost_program_options -Wl,-rpath ${OMPL_DIR}/lib/x86_64-linux-gnu -L${PQP_DIR}/lib -L${KDL_DIR}/lib -lPQP -lm $(GL_LIBS) -larmadillo -lorocos-kdl 
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

CPPPQP = ${INC_VALIDITY}collisionDetection.cpp ${INC_VALIDITY}model.cpp

CPPROD = ${INC_CLASSES}Rod_ODE_class.cpp
CPPROB = ${INC_CLASSES}kdl_class.cpp ${INC_VALIDITY}StateValidityChecker.cpp


all:
	$(CXX) test_rbs_gd.cpp ${CPPROD} ${CPPROB} ${CPPPQP} -o trbs $(CXXFLAGS) $(LDFLAGS) -std=c++11





