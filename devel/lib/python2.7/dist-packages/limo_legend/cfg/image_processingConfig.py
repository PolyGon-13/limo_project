## *********************************************************
##
## File autogenerated for the limo_application package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [{'upper': 'YELLOW_LANE_DETECTION', 'lower': 'yellow_lane_detection', 'srcline': 124, 'name': 'Yellow_Lane_Detection', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::YELLOW_LANE_DETECTION', 'field': 'DEFAULT::yellow_lane_detection', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 25, 'description': 'H Channel Low threshold', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/agilex/limo_project/src/limo_legend/cfg/lane_detection/image_processing.cfg', 'name': 'yellow_h_low', 'edit_method': '', 'default': 50, 'level': 4, 'min': 0, 'type': 'int'}, {'srcline': 26, 'description': 'H Channel High threshold', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/agilex/limo_project/src/limo_legend/cfg/lane_detection/image_processing.cfg', 'name': 'yellow_h_high', 'edit_method': '', 'default': 50, 'level': 5, 'min': 0, 'type': 'int'}, {'srcline': 27, 'description': 'L ChannelLow threshold', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/agilex/limo_project/src/limo_legend/cfg/lane_detection/image_processing.cfg', 'name': 'yellow_l_low', 'edit_method': '', 'default': 50, 'level': 6, 'min': 0, 'type': 'int'}, {'srcline': 28, 'description': 'L Channel High threshold', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/agilex/limo_project/src/limo_legend/cfg/lane_detection/image_processing.cfg', 'name': 'yellow_l_high', 'edit_method': '', 'default': 50, 'level': 7, 'min': 0, 'type': 'int'}, {'srcline': 29, 'description': 'S Channel Low threshold', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/agilex/limo_project/src/limo_legend/cfg/lane_detection/image_processing.cfg', 'name': 'yellow_s_low', 'edit_method': '', 'default': 50, 'level': 8, 'min': 0, 'type': 'int'}, {'srcline': 30, 'description': 'S Channel High threshold', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/agilex/limo_project/src/limo_legend/cfg/lane_detection/image_processing.cfg', 'name': 'yellow_s_high', 'edit_method': '', 'default': 50, 'level': 9, 'min': 0, 'type': 'int'}], 'type': '', 'id': 1}], 'parameters': [], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

