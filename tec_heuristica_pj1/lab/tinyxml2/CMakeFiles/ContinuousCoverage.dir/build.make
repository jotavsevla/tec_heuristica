# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1

# Utility rule file for ContinuousCoverage.

# Include any custom commands dependencies for this target.
include lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/compiler_depend.make

# Include the progress variables for this target.
include lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/progress.make

lab/tinyxml2/CMakeFiles/ContinuousCoverage:
	cd /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1/lab/tinyxml2 && /usr/local/bin/ctest -D ContinuousCoverage

lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/codegen:
.PHONY : lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/codegen

ContinuousCoverage: lab/tinyxml2/CMakeFiles/ContinuousCoverage
ContinuousCoverage: lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/build.make
.PHONY : ContinuousCoverage

# Rule to build all files generated by this target.
lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/build: ContinuousCoverage
.PHONY : lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/build

lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/clean:
	cd /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1/lab/tinyxml2 && $(CMAKE_COMMAND) -P CMakeFiles/ContinuousCoverage.dir/cmake_clean.cmake
.PHONY : lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/clean

lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/depend:
	cd /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1 /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1/lab/tinyxml2 /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1 /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1/lab/tinyxml2 /Users/jotavsevla/CLionProjects/tec_heuristica/tec_heuristica_pj1/lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : lab/tinyxml2/CMakeFiles/ContinuousCoverage.dir/depend

