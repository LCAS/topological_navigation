
class ActionsType:
    def __init__(self):

        self.NAVIGATE_TO_POSE = "NavigateToPose"
        self.NAVIGATE_THROUGH_POSES = "NavigateThroughPoses"
        self.DRIVE_ON_HEADING = "DriveOnHeading"

        self.ROW_TRAVERSAL = "row_traversal"
        self.ROW_CHANGE = "row_change"
        self.GOAL_ALIGN = "goal_align"
        self.GOAL_ALIGN_INDEX = ["ca"]
        self.GOAL_ALIGN_GOAL = ["cb"]
        self.ABORT_NOT_CONTINUE = [self.GOAL_ALIGN, self.ROW_CHANGE, self.ROW_TRAVERSAL, self.NAVIGATE_TO_POSE, self.NAVIGATE_THROUGH_POSES]

        self.BT_DEFAULT = "bt_tree_default"
        self.BT_IN_ROW = "bt_tree_in_row"
        self.BT_GOAL_ALIGN = "bt_tree_goal_align"
        
        self.navigation_actions = [
            self.NAVIGATE_TO_POSE,
            self.NAVIGATE_THROUGH_POSES,
            self.DRIVE_ON_HEADING,
            self.ROW_CHANGE,
            self.ROW_TRAVERSAL
        ]

        self.bt_tree_types = [
            self.BT_IN_ROW,
            self.BT_DEFAULT,
            self.BT_GOAL_ALIGN
        ]

        self.bt_tree_with_actions = {}
        self.bt_tree_with_actions[self.ROW_TRAVERSAL] = self.BT_IN_ROW
        self.bt_tree_with_actions[self.NAVIGATE_THROUGH_POSES] = self.BT_DEFAULT
        self.bt_tree_with_actions[self.NAVIGATE_TO_POSE] = self.BT_DEFAULT
        self.bt_tree_with_actions[self.GOAL_ALIGN] = self.BT_GOAL_ALIGN

        self.status_mapping = {}
        self.status_mapping[0] = "STATUS_UNKNOWN"
        self.status_mapping[1] = "STATUS_ACCEPTED"
        self.status_mapping[2] = "STATUS_EXECUTING"
        self.status_mapping[3] = "STATUS_CANCELING"
        self.status_mapping[4] = "STATUS_SUCCEEDED"
        self.status_mapping[5] = "STATUS_CANCELED"
        self.status_mapping[6] = "STATUS_ABORTED"

        self.goal_cancle_error_codes = {} 
        self.goal_cancle_error_codes[0] = "ERROR_NONE"
        self.goal_cancle_error_codes[1] = "ERROR_REJECTED"
        self.goal_cancle_error_codes[2] = "ERROR_UNKNOWN_GOAL_ID"
        self.goal_cancle_error_codes[3] = "ERROR_GOAL_TERMINATED"

        self.planner_with_goal_checker_config = {
            "dwb_core::DWBLocalPlanner": {
                "goal_checker.xy_goal_tolerance": 0.78,
                "goal_checker.yaw_goal_tolerance": 0.25,
            },
            "teb_local_planner::TebLocalPlannerROS": {
                "goal_checker.xy_goal_tolerance": 0.1,
                "goal_checker.yaw_goal_tolerance": 0.05,
            },
        }

        self.bt_tree_with_control_server_config = {}
        self.bt_tree_with_control_server_config[self.ROW_TRAVERSAL] = "dwb_core::DWBLocalPlanner"
        self.bt_tree_with_control_server_config[self.NAVIGATE_THROUGH_POSES] = "dwb_core::DWBLocalPlanner"
        self.bt_tree_with_control_server_config[self.NAVIGATE_TO_POSE] = "dwb_core::DWBLocalPlanner"
        self.bt_tree_with_control_server_config[self.GOAL_ALIGN] = "dwb_core::DWBLocalPlanner"

    def setPlanner(self, planner_name, action_type):
        self.bt_tree_with_control_server_config[action_type] = planner_name

    def setPlannerParams(self, planner_name, xy_goal_tolerance, yaw_goal_tolerance):
        if not planner_name in self.planner_with_goal_checker_config:
            print("The planner {} is not one of configured types".format(planner_name))
            return 
        self.planner_with_goal_checker_config[planner_name]["goal_checker.xy_goal_tolerance"] = xy_goal_tolerance
        self.planner_with_goal_checker_config[planner_name]["goal_checker.yaw_goal_tolerance"] = yaw_goal_tolerance
        print("Setting planner {} params xy_goal_tolerance: {}, yaw_goal_tolerance: {}".format(planner_name, xy_goal_tolerance, yaw_goal_tolerance))