#!/usr/bin/env python

import rospy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import *

from controller_manager_msgs.utils\
    import ControllerLister, ControllerManagerLister,\
    get_rosparam_controller_names

 
def _resolve_controllers_ns(cm_ns):
    """
    Resolve the namespace containing controller configurations from that of
    the controller manager.
    Controllers are assumed to live one level above the controller
    manager, e.g.
        >>> _resolve_controller_ns('/path/to/controller_manager')
        '/path/to'
    In the particular case in which the controller manager is not
    namespaced, the controller is assumed to live in the root namespace
        >>> _resolve_controller_ns('/')
        '/'
        >>> _resolve_controller_ns('')
        '/'
    @param cm_ns Controller manager namespace (can be an empty string)
    @type cm_ns str
    @return Controllers namespace
    @rtype str
    """
    ns = cm_ns.rsplit('/', 1)[0]
    if not ns:
        ns += '/'
    return ns


def _append_ns(in_ns, suffix):
    """
    Append a sub-namespace (suffix) to the input namespace
    @param in_ns Input namespace
    @type in_ns str
    @return Suffix namespace
    @rtype str
    """
    ns = in_ns
    if ns[-1] != '/':
        ns += '/'
    ns += suffix
    return ns


def _rosparam_controller_type(ctrls_ns, ctrl_name):
    """
    Get a controller's type from its ROS parameter server configuration
    @param ctrls_ns Namespace where controllers should be located
    @type ctrls_ns str
    @param ctrl_name Controller name
    @type ctrl_name str
    @return Controller type
    @rtype str
    """
    type_param = _append_ns(ctrls_ns, ctrl_name) + '/type'
    return rospy.get_param(type_param)   

class ControllerManagerInterface(object):

    def __init__(self, ns="left_arm/"):

        self._ns = ns
        self._prefix = "controller_manager"
        self._cm_ns = self._ns + self._prefix
        self._service_names = ["list_controllers",
                         "unload_controller",
                         "load_controller",
                         "switch_controller"]


        load_srv_name = self._cm_ns + "/load_controller"
        self._load_srv = rospy.ServiceProxy(load_srv_name,
                                                LoadController,
                                                persistent=True)
        unload_srv_name = self._cm_ns + "/unload_controller"
        self._unload_srv = rospy.ServiceProxy(unload_srv_name,
                                                  UnloadController,
                                                  persistent=True)
        switch_srv_name = self._cm_ns + "/switch_controller"
        self._switch_srv = rospy.ServiceProxy(switch_srv_name,
                                                  SwitchController,
                                                  persistent=True)

        list_srv_name = self._cm_ns + "/list_controllers"
        self._list_srv = rospy.ServiceProxy(list_srv_name,
                                                  ListControllers,
                                                  persistent=True)

        list_types_srv_name = self._cm_ns + "/list_controller_types"
        self._list_types_srv = rospy.ServiceProxy(list_types_srv_name,
                                                  ListControllerTypes,
                                                  persistent=True)



        self._controller_lister = ControllerLister(self._cm_ns)

    def load_controller(self, name):
        self._load_srv.call(LoadControllerRequest(name=name))

    def unload_controller(self, name):
        self._unload_srv.call(UnloadControllerRequest(name=name))

    def start_controller(self, name):
        strict = SwitchControllerRequest.STRICT
        req = SwitchControllerRequest(start_controllers=[name],
                                      stop_controllers=[],
                                      strictness=strict)
        self._switch_srv.call(req)

    def stop_controller(self, name):
        strict = SwitchControllerRequest.STRICT
        req = SwitchControllerRequest(start_controllers=[],
                                      stop_controllers=[name],
                                      strictness=strict)
        self._switch_srv.call(req)


    def list_loaded_controllers(self):

        req = ListControllersRequest()

        return self._list_srv.call(req)
    
    def list_controller_types(self):

        req = ListControllerTypesRequest()

        return self._list_types_srv.call(req)

    def list_controllers(self):
        """
        @return List of controllers associated to a controller manager
        namespace. Contains both stopped/running controllers, as returned by
        the C{list_controllers} service, plus uninitialized controllers with
        configurations loaded in the parameter server.
        @rtype [str]
        """
        if not self._cm_ns:
            return []

        # Add loaded controllers first
        controllers = self._controller_lister()

        # Append potential controller configs found in the parameter server
        all_ctrls_ns = _resolve_controllers_ns(self._cm_ns)
        for name in get_rosparam_controller_names(all_ctrls_ns):

            add_ctrl = not any(name == ctrl.name for ctrl in controllers)
            if add_ctrl:
                type_str = _rosparam_controller_type(all_ctrls_ns, name)
                uninit_ctrl = ControllerState(name=name,
                                              type=type_str,
                                              state='uninitialized')
                controllers.append(uninit_ctrl)
        return controllers

    def controller_dict(self):

        controllers = self.list_controllers()

        controller_dict = {}
        for c in controllers:
            controller_dict[c.name] = c


        return controller_dict

    def is_running(self, controller_name):

        controllers = self.controller_dict()

        ctrl_state = controllers.get(controller_name,None)

        return ctrl_state is not None and ctrl_state.state=="running"

    def is_loaded(self, controller_name):

        controllers = self.controller_dict()

        ctrl_state = controllers.get(controller_name,None)

        return ctrl_state is not None and ctrl_state.state!="uninitialized"




if __name__ == '__main__':

    cmi = ControllerManagerInterface()




