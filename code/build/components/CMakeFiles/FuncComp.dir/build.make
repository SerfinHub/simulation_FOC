# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.24

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\build"

# Include any dependencies generated for this target.
include components/CMakeFiles/FuncComp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include components/CMakeFiles/FuncComp.dir/compiler_depend.make

# Include the progress variables for this target.
include components/CMakeFiles/FuncComp.dir/progress.make

# Include the compile flags for this target's objects.
include components/CMakeFiles/FuncComp.dir/flags.make

components/CMakeFiles/FuncComp.dir/Controllers.cpp.obj: components/CMakeFiles/FuncComp.dir/flags.make
components/CMakeFiles/FuncComp.dir/Controllers.cpp.obj: C:/Users/bachm/Documents/Plexim/FOC/1\ Close-loop/code/components/Controllers.cpp
components/CMakeFiles/FuncComp.dir/Controllers.cpp.obj: components/CMakeFiles/FuncComp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\build\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object components/CMakeFiles/FuncComp.dir/Controllers.cpp.obj"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\X86_64~2.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT components/CMakeFiles/FuncComp.dir/Controllers.cpp.obj -MF CMakeFiles\FuncComp.dir\Controllers.cpp.obj.d -o CMakeFiles\FuncComp.dir\Controllers.cpp.obj -c "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components\Controllers.cpp"

components/CMakeFiles/FuncComp.dir/Controllers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FuncComp.dir/Controllers.cpp.i"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\X86_64~2.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components\Controllers.cpp" > CMakeFiles\FuncComp.dir\Controllers.cpp.i

components/CMakeFiles/FuncComp.dir/Controllers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FuncComp.dir/Controllers.cpp.s"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\X86_64~2.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components\Controllers.cpp" -o CMakeFiles\FuncComp.dir\Controllers.cpp.s

components/CMakeFiles/FuncComp.dir/SVM.cpp.obj: components/CMakeFiles/FuncComp.dir/flags.make
components/CMakeFiles/FuncComp.dir/SVM.cpp.obj: C:/Users/bachm/Documents/Plexim/FOC/1\ Close-loop/code/components/SVM.cpp
components/CMakeFiles/FuncComp.dir/SVM.cpp.obj: components/CMakeFiles/FuncComp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\build\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object components/CMakeFiles/FuncComp.dir/SVM.cpp.obj"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\X86_64~2.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT components/CMakeFiles/FuncComp.dir/SVM.cpp.obj -MF CMakeFiles\FuncComp.dir\SVM.cpp.obj.d -o CMakeFiles\FuncComp.dir\SVM.cpp.obj -c "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components\SVM.cpp"

components/CMakeFiles/FuncComp.dir/SVM.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FuncComp.dir/SVM.cpp.i"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\X86_64~2.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components\SVM.cpp" > CMakeFiles\FuncComp.dir\SVM.cpp.i

components/CMakeFiles/FuncComp.dir/SVM.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FuncComp.dir/SVM.cpp.s"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\X86_64~2.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components\SVM.cpp" -o CMakeFiles\FuncComp.dir\SVM.cpp.s

components/CMakeFiles/FuncComp.dir/FOC.cpp.obj: components/CMakeFiles/FuncComp.dir/flags.make
components/CMakeFiles/FuncComp.dir/FOC.cpp.obj: C:/Users/bachm/Documents/Plexim/FOC/1\ Close-loop/code/components/FOC.cpp
components/CMakeFiles/FuncComp.dir/FOC.cpp.obj: components/CMakeFiles/FuncComp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\build\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object components/CMakeFiles/FuncComp.dir/FOC.cpp.obj"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\X86_64~2.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT components/CMakeFiles/FuncComp.dir/FOC.cpp.obj -MF CMakeFiles\FuncComp.dir\FOC.cpp.obj.d -o CMakeFiles\FuncComp.dir\FOC.cpp.obj -c "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components\FOC.cpp"

components/CMakeFiles/FuncComp.dir/FOC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FuncComp.dir/FOC.cpp.i"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\X86_64~2.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components\FOC.cpp" > CMakeFiles\FuncComp.dir\FOC.cpp.i

components/CMakeFiles/FuncComp.dir/FOC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FuncComp.dir/FOC.cpp.s"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && C:\PROGRA~1\MINGW-~1\X86_64~1.0-P\mingw64\bin\X86_64~2.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components\FOC.cpp" -o CMakeFiles\FuncComp.dir\FOC.cpp.s

# Object files for target FuncComp
FuncComp_OBJECTS = \
"CMakeFiles/FuncComp.dir/Controllers.cpp.obj" \
"CMakeFiles/FuncComp.dir/SVM.cpp.obj" \
"CMakeFiles/FuncComp.dir/FOC.cpp.obj"

# External object files for target FuncComp
FuncComp_EXTERNAL_OBJECTS =

components/libFuncComp.a: components/CMakeFiles/FuncComp.dir/Controllers.cpp.obj
components/libFuncComp.a: components/CMakeFiles/FuncComp.dir/SVM.cpp.obj
components/libFuncComp.a: components/CMakeFiles/FuncComp.dir/FOC.cpp.obj
components/libFuncComp.a: components/CMakeFiles/FuncComp.dir/build.make
components/libFuncComp.a: components/CMakeFiles/FuncComp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\build\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libFuncComp.a"
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && $(CMAKE_COMMAND) -P CMakeFiles\FuncComp.dir\cmake_clean_target.cmake
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\FuncComp.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
components/CMakeFiles/FuncComp.dir/build: components/libFuncComp.a
.PHONY : components/CMakeFiles/FuncComp.dir/build

components/CMakeFiles/FuncComp.dir/clean:
	cd /d C:\Users\bachm\DOCUME~1\Plexim\FOC\1CLOSE~1\code\build\COMPON~1 && $(CMAKE_COMMAND) -P CMakeFiles\FuncComp.dir\cmake_clean.cmake
.PHONY : components/CMakeFiles/FuncComp.dir/clean

components/CMakeFiles/FuncComp.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code" "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\components" "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\build" "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\build\components" "C:\Users\bachm\Documents\Plexim\FOC\1 Close-loop\code\build\components\CMakeFiles\FuncComp.dir\DependInfo.cmake" --color=$(COLOR)
.PHONY : components/CMakeFiles/FuncComp.dir/depend
