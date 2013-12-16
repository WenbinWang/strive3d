INCLUDE_DIRECTORIES( 	../ ../Action ../AdvancedAgent ../BehaviorControl 
			../Geometry ../HeadControl ../Module ../Net
			../Parser ../RobotModel ../salt ../Trainer_Client 
			../Utilities ../WorldModel ../Xabsl ../DataAnalyse
 )

SET(	BehaviorControl_LIB_HDRS
BehaviorControl.h          NaoBehaviorControl.h       XabslBehaviorControl.h
BehaviorControlSelector.h  Strive3DBehaviorControl.h
CommonBasicBehaviors.h     WalkPathPlanningControl.h
)

SET(	BehaviorControl_LIB_SRCS
CommonBasicBehaviors.cpp  Strive3DBehaviorControl.cpp  XabslBehaviorControl.cpp
NaoBehaviorControl.cpp    WalkPathPlanningControl.cpp
)

ADD_LIBRARY(BehaviorControl ${BehaviorControl_LIB_SRCS} ${BehaviorControl_LIB_HDRS})
