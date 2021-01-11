###########################################################################
## Makefile generated for MATLAB file/project 'planTrajectory'. 
## 
## Makefile     : planTrajectory_rtw.mk
## Generated on : Fri Jan 08 13:31:44 2021
## MATLAB Coder version: 5.0 (R2020a)
## 
## Build Info:
## 
## Final product: ./planTrajectory.so
## Product type : dynamic-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# DEF_FILE                Definition file

PRODUCT_NAME              = planTrajectory
MAKEFILE                  = planTrajectory_rtw.mk
MATLAB_ROOT               = /usr/local/MATLAB/R2020a
MATLAB_BIN                = /usr/local/MATLAB/R2020a/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
MASTER_ANCHOR_DIR         = 
START_DIR                 = /home/vp/Downloads/cpm/software/tools/go_to_formation/matlab/Coder/codegen/dll/planTrajectory
TGT_FCN_LIB               = ISO_C++
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
DEF_FILE                  = $(PRODUCT_NAME).def
C_STANDARD_OPTS           = -fwrapv -ansi -pedantic -Wno-long-long
CPP_STANDARD_OPTS         = -fwrapv -std=c++03 -pedantic -Wno-long-long

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU gcc/g++ | gmake (64-bit Linux)
# Supported Version(s):    
# ToolchainInfo Version:   2020a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS

#-----------
# MACROS
#-----------

WARN_FLAGS         = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align
WARN_FLAGS_MAX     = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS     = -Wall -W -Wwrite-strings -Winline -Wpointer-arith -Wcast-align
CPP_WARN_FLAGS_MAX = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: GNU C Compiler
CC = gcc

# Linker: GNU Linker
LD = g++

# C++ Compiler: GNU C++ Compiler
CPP = g++

# C++ Linker: GNU C++ Linker
CPP_LD = g++

# Archiver: GNU Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%/bin/glnxa64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @rm -f
ECHO                = @echo
MV                  = @mv
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c $(C_STANDARD_OPTS) -fPIC \
                       -O3 -fno-loop-optimize -fno-aggressive-loop-optimizations
CPPFLAGS             = -c $(CPP_STANDARD_OPTS) -fPIC \
                       -O3 -fno-loop-optimize -fno-aggressive-loop-optimizations
CPP_LDFLAGS          = -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)"
CPP_SHAREDLIB_LDFLAGS  = -shared -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -Wl,--no-undefined
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)"
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -Wl,--no-undefined



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./planTrajectory.so
PRODUCT_TYPE = "dynamic-library"
BUILD_TYPE = "Dynamic Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I/home/vp/Downloads/cpm/software/tools/go_to_formation/matlab/Coder -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/extern/include/shared_autonomous

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -DBUILDING_PLANTRAJECTORY -DMODEL=planTrajectory
DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=planTrajectory

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/rt_nonfinite.cpp $(START_DIR)/rtGetNaN.cpp $(START_DIR)/rtGetInf.cpp $(START_DIR)/planTrajectory_rtwutil.cpp $(START_DIR)/planTrajectory_data.cpp $(START_DIR)/planTrajectory_initialize.cpp $(START_DIR)/planTrajectory_terminate.cpp $(START_DIR)/planTrajectory.cpp $(START_DIR)/setCostmap.cpp $(START_DIR)/minOrMax.cpp $(START_DIR)/VehicleCostmapCodegen.cpp $(START_DIR)/VehicleCostmapImpl.cpp $(START_DIR)/imdilate.cpp $(START_DIR)/cosd.cpp $(START_DIR)/sind.cpp $(START_DIR)/PlanRRTPath.cpp $(START_DIR)/pathPlannerRRT.cpp $(START_DIR)/RRTPlanner.cpp $(START_DIR)/UniformPoseSampler.cpp $(START_DIR)/rand.cpp $(START_DIR)/eml_rand_mt19937ar_stateful.cpp $(START_DIR)/all.cpp $(START_DIR)/DubinsConnectionMechanism.cpp $(START_DIR)/SqrtApproxNeighborSearcher.cpp $(START_DIR)/NeighborSearcher.cpp $(START_DIR)/RRTTree.cpp $(START_DIR)/angleUtilities.cpp $(START_DIR)/DubinsPathSegmentCodegen.cpp $(START_DIR)/OneDimArrayBehavior.cpp $(START_DIR)/DubinsConnection.cpp $(START_DIR)/DubinsConnection1.cpp $(START_DIR)/NameValueParser.cpp $(START_DIR)/DubinsPathSegmentImpl.cpp $(START_DIR)/DubinsPathSegment.cpp $(START_DIR)/Path.cpp $(START_DIR)/Path1.cpp $(START_DIR)/DubinsBuiltins.cpp $(START_DIR)/extremeKElements.cpp $(START_DIR)/sortIdx.cpp $(START_DIR)/mergesort.cpp $(START_DIR)/sortedInsertion.cpp $(START_DIR)/stack1.cpp $(START_DIR)/eml_setop.cpp $(START_DIR)/sort.cpp $(START_DIR)/checkPathValidity.cpp $(START_DIR)/unique.cpp $(START_DIR)/pathToTrajectory.cpp $(START_DIR)/ismember.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = rt_nonfinite.o rtGetNaN.o rtGetInf.o planTrajectory_rtwutil.o planTrajectory_data.o planTrajectory_initialize.o planTrajectory_terminate.o planTrajectory.o setCostmap.o minOrMax.o VehicleCostmapCodegen.o VehicleCostmapImpl.o imdilate.o cosd.o sind.o PlanRRTPath.o pathPlannerRRT.o RRTPlanner.o UniformPoseSampler.o rand.o eml_rand_mt19937ar_stateful.o all.o DubinsConnectionMechanism.o SqrtApproxNeighborSearcher.o NeighborSearcher.o RRTTree.o angleUtilities.o DubinsPathSegmentCodegen.o OneDimArrayBehavior.o DubinsConnection.o DubinsConnection1.o NameValueParser.o DubinsPathSegmentImpl.o DubinsPathSegment.o Path.o Path1.o DubinsBuiltins.o extremeKElements.o sortIdx.o mergesort.o sortedInsertion.o stack1.o eml_setop.o sort.o checkPathValidity.o unique.o pathToTrajectory.o ismember.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = $(MATLAB_ROOT)/bin/glnxa64/libmwautonomouscodegen.so

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS =  -L"$(MATLAB_ROOT)/bin/glnxa64" -L"$(MATLAB_ROOT)/sys/os/glnxa64" -lmwmorphop_binary_tbb -lmwnhood -lm -lstdc++ -liomp5

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_ = -fvisibility=hidden
CFLAGS_OPTS = -fopenmp
CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_) $(CFLAGS_OPTS) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_ = -fvisibility=hidden
CPPFLAGS_OPTS = -fopenmp
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_) $(CPPFLAGS_OPTS) $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#----------------------------------------
# Create a dynamic library
#----------------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS) $(LIBS)
	@echo "### Creating dynamic library "$(PRODUCT)" ..."
	$(CPP_LD) $(CPP_SHAREDLIB_LDFLAGS) -o $(PRODUCT) $(OBJS) -Wl,--start-group $(LIBS) -Wl,--end-group $(SYSTEM_LIBS) $(TOOLCHAIN_LIBS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/vp/Downloads/cpm/software/tools/go_to_formation/matlab/Coder/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : /home/vp/Downloads/cpm/software/tools/go_to_formation/matlab/Coder/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rt_nonfinite.o : $(START_DIR)/rt_nonfinite.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetNaN.o : $(START_DIR)/rtGetNaN.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetInf.o : $(START_DIR)/rtGetInf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


planTrajectory_rtwutil.o : $(START_DIR)/planTrajectory_rtwutil.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


planTrajectory_data.o : $(START_DIR)/planTrajectory_data.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


planTrajectory_initialize.o : $(START_DIR)/planTrajectory_initialize.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


planTrajectory_terminate.o : $(START_DIR)/planTrajectory_terminate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


planTrajectory.o : $(START_DIR)/planTrajectory.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


setCostmap.o : $(START_DIR)/setCostmap.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


minOrMax.o : $(START_DIR)/minOrMax.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


VehicleCostmapCodegen.o : $(START_DIR)/VehicleCostmapCodegen.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


VehicleCostmapImpl.o : $(START_DIR)/VehicleCostmapImpl.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


imdilate.o : $(START_DIR)/imdilate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


cosd.o : $(START_DIR)/cosd.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sind.o : $(START_DIR)/sind.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


PlanRRTPath.o : $(START_DIR)/PlanRRTPath.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


pathPlannerRRT.o : $(START_DIR)/pathPlannerRRT.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


RRTPlanner.o : $(START_DIR)/RRTPlanner.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


UniformPoseSampler.o : $(START_DIR)/UniformPoseSampler.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rand.o : $(START_DIR)/rand.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


eml_rand_mt19937ar_stateful.o : $(START_DIR)/eml_rand_mt19937ar_stateful.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


all.o : $(START_DIR)/all.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DubinsConnectionMechanism.o : $(START_DIR)/DubinsConnectionMechanism.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


SqrtApproxNeighborSearcher.o : $(START_DIR)/SqrtApproxNeighborSearcher.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


NeighborSearcher.o : $(START_DIR)/NeighborSearcher.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


RRTTree.o : $(START_DIR)/RRTTree.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


angleUtilities.o : $(START_DIR)/angleUtilities.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DubinsPathSegmentCodegen.o : $(START_DIR)/DubinsPathSegmentCodegen.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


OneDimArrayBehavior.o : $(START_DIR)/OneDimArrayBehavior.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DubinsConnection.o : $(START_DIR)/DubinsConnection.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DubinsConnection1.o : $(START_DIR)/DubinsConnection1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


NameValueParser.o : $(START_DIR)/NameValueParser.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DubinsPathSegmentImpl.o : $(START_DIR)/DubinsPathSegmentImpl.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DubinsPathSegment.o : $(START_DIR)/DubinsPathSegment.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Path.o : $(START_DIR)/Path.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Path1.o : $(START_DIR)/Path1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


DubinsBuiltins.o : $(START_DIR)/DubinsBuiltins.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


extremeKElements.o : $(START_DIR)/extremeKElements.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sortIdx.o : $(START_DIR)/sortIdx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mergesort.o : $(START_DIR)/mergesort.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sortedInsertion.o : $(START_DIR)/sortedInsertion.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


stack1.o : $(START_DIR)/stack1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


eml_setop.o : $(START_DIR)/eml_setop.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sort.o : $(START_DIR)/sort.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


checkPathValidity.o : $(START_DIR)/checkPathValidity.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


unique.o : $(START_DIR)/unique.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


pathToTrajectory.o : $(START_DIR)/pathToTrajectory.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ismember.o : $(START_DIR)/ismember.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


