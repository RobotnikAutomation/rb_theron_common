#!/usr/bin/env python

# This handler structure is temporary. Not standard.

from robot_local_control_msgs.msg import Pose2DStamped
from poi_manager_msgs.srv import GetPOI_params, GetPOI_paramsRequest
from robot_simple_command_manager.handlers.command_procedure_interface import *


class GoToTagProcedureInterface(CommandProcedureInterface):
    def __init__(self, name, parameters):
        CommandProcedureInterface.__init__(self, name, parameters)

        self.args_description = ['tag_name','environement']
        self.args_types = [str,str]
        self.args_void_allowed = [False]

    def set_parameters(self, parameters):
        '''
            Set all the required parameters of the interface
        '''
        CommandProcedureInterface.set_parameters(self, parameters)
        self.get_poi_ns = self.get_parameter('get_poi_params', 'poi_manager/get_poi_params')


    def build_msg(self, args):
        '''
            Return the desired goal or None
        '''
        client = rospy.ServiceProxy(self.get_poi_ns, GetPOI_params)
        request = GetPOI_paramsRequest()
        request.name = args[0]
        request.environment = args[1]
        response = client.call(request)
        if not response.success:
            self.raise_exception("Could not get pose from POI: "+args[0])

        print (str(response))

        if type(args) == list:

            self.request.procedure.goals = [Pose2DStamped()]
            self.request.procedure.goals[0].header.frame_id = response.frame_id
            self.request.procedure.goals[0].pose.x = response.x
            self.request.procedure.goals[0].pose.y= response.y
            self.request.procedure.goals[0].pose.theta = response.yaw
