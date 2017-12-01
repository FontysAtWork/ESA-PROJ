#Comment only lines are allowed
TMP="Example variable" #Example comment

############## MULTI FILE VARS ##############

MAP_NORMAL="map_normal"

############## /MULTI FILE VARS ##############

# nav_only.template
ROBOT_DESC="\$(find xacro)/xacro.py \$(find youbot_description)/robots/youbot.urdf.xacro"
MAP_FILE="\$(find faw)/map/test2.yaml"

#planning_context.template
ROBOT_SEMANTIC="\$(find youbot_description)/moveit_config/youbot.srdf"
JOINT_LIMITS="\$(find youbot_description)/moveit_config/joint_limits.yaml"
ROBOT_KINEMATICS="\$(find youbot_description)/moveit_config/kinematics.yaml"

#lasers.template
FRONT_LASER_NAME="hokuyo_H1100477"
BACK_LASER_NAME="hokuyo_H0904583"
