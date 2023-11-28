
class ActionsType:
    def __init__(self):

        self.NAVIGATE_TO_POSE = "NavigateToPose"
        self.NAVIGATE_THROUGH_POSES = "NavigateThroughPoses"
        self.DRIVE_ON_HEADING = "DriveOnHeading"

        self.ROW_TRAVERSAL = "row_traversal"
        self.ROW_CHANGE = "row_change"
        self.GOAL_ALIGN = "goal_align"
        self.GOAL_ALIGN_INDEX = ["cb"]

        self.BT_DEFAULT = "bt_tree_default"
        self.BT_IN_ROW = "bt_tree_in_row"
        self.BT_GOAL_ALIGN = "bt_tree_goal_align"
        
        self.navigation_actions = [
            self.NAVIGATE_TO_POSE,
            self.NAVIGATE_THROUGH_POSES,
            self.DRIVE_ON_HEADING,
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