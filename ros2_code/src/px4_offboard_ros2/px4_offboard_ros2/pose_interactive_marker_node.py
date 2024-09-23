#!/usr/bin/env python3

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

# adapted from https://github.com/ros-visualization/visualization_tutorials
# Tareq Alqutami (Tareqaziz2010@gmail.com)
# Dec 2023

import sys

import rclpy
from interactive_markers import InteractiveMarkerServer
from interactive_markers import MenuHandler
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarkerFeedback


class MarkerServerPublisher():
    """
    A class that creates an interactive marker server with its menu handler 
    and publish the marker Pose to a topic on click of menu items
    Developed and tested on ROS2 Foxy
    """
    def __init__(self,topic_name:str='/target_pose'):
        # ros2 node and logger
        self.node = rclpy.create_node('pose_interactive_marker')
        self.logger = self.node.get_logger()
        
        # default position
        self.x0, self.y0, self.z0 = 0.0, 0.0, 1.0
        # track the marker pose        
        self.marker_pose = Pose(position=Point(x=self.x0,y=self.y0,z=self.z0))

        # marker pose publisher
        self.pose_pub = self.node.create_publisher(PoseStamped,topic_name,10)
        
        # interactive marker server and menu handler
        self.server = InteractiveMarkerServer(self.node, 'pose_interactive_marker')
        self.menu_handler = MenuHandler()

    def pub_pose(self, pose:Pose):
        """Publish a pose to pose topic

        Args:
            pose (Pose): pose to publish
        """
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = pose.position.x
        pose_msg.pose.position.y = pose.position.y
        pose_msg.pose.position.z = pose.position.z
        pose_msg.pose.orientation.w = pose.orientation.w
        pose_msg.pose.orientation.x = pose.orientation.x
        pose_msg.pose.orientation.y = pose.orientation.y
        pose_msg.pose.orientation.z = pose.orientation.z
        self.pose_pub.publish(pose_msg)

    def processFeedback(self, feedback:InteractiveMarkerFeedback):
        """Callback to process marker feedback

        Args:
            feedback (InteractiveMarkerFeedback): marker feedback
        """
        log_prefix = (
            f"Feedback from marker '{feedback.marker_name}' / control '{feedback.control_name}'"
        )

        log_mouse = ''
        if feedback.mouse_point_valid:
            log_mouse = (
                f'{feedback.mouse_point.x}, {feedback.mouse_point.y}, '
                f'{feedback.mouse_point.z} in frame {feedback.header.frame_id}'
            )

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.logger.debug(f'{log_prefix}: button click at {log_mouse}')
            
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self.logger.debug(f'menu_entry_id : {feedback.menu_entry_id}')
            if feedback.menu_entry_id == 1:
                # publish pose to topic
                self.logger.info(f'Publishing Position = ({self.marker_pose.position.x}, {self.marker_pose.position.y}, '
                                 f'{self.marker_pose.position.z}), orientation = '
                                 f'({self.marker_pose.orientation.w}, {self.marker_pose.orientation.x}, '
                                 f'{self.marker_pose.orientation.y}, {self.marker_pose.orientation.z}) \n')
                self.pub_pose(self.marker_pose)
            elif feedback.menu_entry_id == 3:
                # reset position
                feedback.pose.position.x = self.x0
                feedback.pose.position.y = self.y0
                feedback.pose.position.z = self.z0
                self.marker_pose = feedback.pose
                self.reset_marker(feedback)
            elif feedback.menu_entry_id == 4:
                # reset orientation
                feedback.pose.orientation.w = 1.0
                feedback.pose.orientation.x = 0.0
                feedback.pose.orientation.y = 0.0
                feedback.pose.orientation.z = 0.0
                self.marker_pose = feedback.pose
                self.reset_marker(feedback)
                
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # update marker pose
            self.marker_pose = feedback.pose
            
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.logger.debug(f'{log_prefix}: mouse down at {log_mouse}')
            
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.logger.debug(f'{log_prefix}: mouse up at {log_mouse}')

    def reset_marker(self,feedback:InteractiveMarkerFeedback):
        """Reset marker pose to the values given in the feedback pose
        This is used to programmatically change the marker pose for example when clicking position reset.

        Args:
            feedback (InteractiveMarkerFeedback): marker feedback
        """
        
        pose = feedback.pose

        self.logger.info(
            f'{feedback.marker_name}: Resetting to Position = ({pose.position.x}, {pose.position.y}, {pose.position.z})'
            f' , orientation = ({pose.orientation.w}, {pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}) \n')

        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()

    def make_box(self,msg):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.3
        marker.scale.y = msg.scale * 0.3
        marker.scale.z = msg.scale * 0.3
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def make_box_control(self,msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_box(msg))
        msg.controls.append(control)
        return control


    def normalize_quaternion(self,quaternion_msg):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def make_6dof_marker(self,position, fixed, move_3d, rotate_3d, frame_id='map'):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.pose.position = position
        int_marker.scale = 0.2

        int_marker.name = '6d_marker'
        int_marker.description = '6-DOF Control'

        # insert a box and control
        self.make_box_control(int_marker)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MENU

        if move_3d:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            self.normalize_quaternion(control.orientation)
            control.name = 'move_x'
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)
            

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 1.0
            control.orientation.z = 0.0
            self.normalize_quaternion(control.orientation)
            control.name = 'move_z'
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 1.0
            self.normalize_quaternion(control.orientation)
            control.name = 'move_y'
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)
            
        if rotate_3d:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 1.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            self.normalize_quaternion(control.orientation)
            control.name = 'rotate_x'
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 1.0
            control.orientation.z = 0.0
            self.normalize_quaternion(control.orientation)
            control.name = 'rotate_z'
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 1.0
            self.normalize_quaternion(control.orientation)
            control.name = 'rotate_y'
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)


        self.server.insert(int_marker, feedback_callback=self.processFeedback)
        
        self.menu_handler.insert('Command Pose', callback=self.processFeedback)
        sub_menu_handle = self.menu_handler.insert('Reset')
        self.menu_handler.insert('Reset Position', parent=sub_menu_handle, callback=self.processFeedback)
        self.menu_handler.insert('Reset Orientation', parent=sub_menu_handle, callback=self.processFeedback)

        self.menu_handler.apply(self.server, int_marker.name)
        


def main(args=None):
    rclpy.init(args=args)

    marker_server = MarkerServerPublisher()
    
    # crate interactive marker
    position = Point(x=0.0, y=0.0, z=2.0)
    marker_server.make_6dof_marker(position=position,
                                   fixed=True,
                                   move_3d=True,
                                   rotate_3d=True,
                                   frame_id='map')
    marker_server.server.applyChanges()

    rclpy.spin(marker_server.node)
    marker_server.server.shutdown()
    

if __name__ == '__main__':
    main(args=sys.argv)
