set(HEADER_FILES
  Controllers.h
  SVM.h
  FOC.h
  AngleUnwrapper.h
  TrajectoryPlanner.h
)

add_library(FuncComp
  Controllers.cpp
  SVM.cpp
  FOC.cpp
  TrajectoryPlanner.cpp
)
