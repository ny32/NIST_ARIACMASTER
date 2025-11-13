from typing import cast

import math 

import tf2_ros

from rclpy.duration import Duration

from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Pose, Point, Vector3, Quaternion

from ariac_interfaces.msg import VacuumTools
from ariac_interfaces.srv import AttachTool, Trigger

from example_team.robot_interface import Robot
from example_team.gripper_interface import Gripper
from example_team.utils import AsyncUtils, quaternion_from_list

class InspectionRobot2(Robot):
    def __init__(self):
        super().__init__('inspection_robot_2')

        self.tester_locations: dict[int, Vector3] = {}

        self.orientations: dict[str, Quaternion] = {
            'tester': quaternion_from_list(quaternion_from_euler(math.pi, 0.0, 0.0)),
            'recycling': quaternion_from_list(quaternion_from_euler(math.pi, 0.0, math.pi/2))
        }

        self.joint_states = {
            'ready': [-0.52, -1.7, 2.22, -2.08, -1.57, 1.05],
            'recycling': [-5.15, -1.76, 1.822, -1.57, -1.57, 0.0],
        }

        # Initialize TF
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        self.gripper = Gripper(self, 'inspection_robot_2')

    async def ready(self): 
        await super().ready()
        for i in (1, 2):
            t = await AsyncUtils.get_tf_transform(self.tf_buffer, 'world', f'voltage_tester_{i}_frame')
            self.tester_locations[i] = t.transform.translation

        plan = await self.plan_to_joint_state(self.joint_states['ready'])
        await self.execute(plan)

    async def pick_cell_from_tester(self, tester: int):
        self.get_logger().info(f'Picking cell from tester {tester}')

        await self.gripper.open()
        
        p = self.tester_locations[tester]
        above = Point(x=p.x, y=p.y, z=p.z+0.05)
        grasp = Point(x=p.x, y=p.y, z=p.z)

        for i, position in enumerate([above, grasp, above]):
            plan = await self.plan_to_pose(Pose(position=position, orientation=self.orientations['tester']))
            await self.execute(plan)

            if i == 1:
                await self.gripper.close()
                if not self.gripper.is_holding:
                    raise RuntimeError(f"Failed to grasp cell from tester {tester}")
    
    async def recycle_cell(self):
        self.get_logger().info(f'Recycling cell')

        plan = await self.plan_to_joint_state(self.joint_states['recycling'])
        await self.execute(plan)

        position = self.current_pose.position
        position.z -= 0.2

        plan = await self.plan_to_pose(Pose(position=position, orientation=self.orientations['recycling']))
        await self.execute(plan)

        await self.gripper.open()

        plan = await self.plan_to_joint_state(self.joint_states['ready'], vsf=0.5)
        await self.execute(plan)


class AssemblyRobot2(Robot):
    def __init__(self):
        super().__init__('assembly_robot_2')

        self.tool_locations: dict[int, Vector3] = {}

        self.orientations: dict[str, Quaternion] = {
            'tool_changer': quaternion_from_list(quaternion_from_euler(math.pi, 0.0, math.pi)),
        }

        # Initialize TF
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        self.attached_tool = VacuumTools.NONE

        self.attach_tool_client = self.create_client(AttachTool, '/assembly_robot_2/tool_changer/attach_tool')
        self.detach_tool_client = self.create_client(Trigger, '/assembly_robot_2/tool_changer/detach_tool')

    async def ready(self): 
        await super().ready()

        # Fill tf locations
        tool_frames = {
            VacuumTools.VG_2: "tool_holder_vg_2",
            VacuumTools.VG_4: "tool_holder_vg_4",
        }

        for tool, frame in tool_frames.items():
            t = await AsyncUtils.get_tf_transform(self.tf_buffer, 'world', frame)
            self.tool_locations[tool] = t.transform.translation

    async def pick_tool_from_stand(self, tool_type: int):
        if tool_type not in (VacuumTools.VG_2, VacuumTools.VG_4):
            self.get_logger().warn(f"Unable to attach {tool_type}. Invalid tool type")
            return

        if self.attached_tool != VacuumTools.NONE:
            self.get_logger().warn(f"Unable to attach {tool_type}. A tool is already attached")
            return

        p = self.tool_locations[tool_type]
        above = Point(x=p.x, y=p.y, z=p.z+0.025)
        attach = Point(x=p.x, y=p.y, z=p.z)
        back = Point(x=p.x, y=p.y+0.05, z=p.z+0.025)

        for i, position in enumerate([above, attach, above, back]):
            plan = await self.plan_to_pose(Pose(position=position, orientation=self.orientations['tool_changer']), vsf=0.5, asf=0.2)
            await self.execute(plan)

            if i == 1:
                await AsyncUtils.await_for_duration(self.get_clock(), Duration(seconds=0.5))
                await self._attach_tool(tool_type)

        self.attached_tool = tool_type

    async def return_tool_to_stand(self):
        if self.attached_tool == VacuumTools.NONE:
            self.get_logger().warn(f"Unable to release. No tool currently attached")
            return
        
        p = self.tool_locations[self.attached_tool]
        above = Point(x=p.x, y=p.y, z=p.z+0.025)
        detatch = Point(x=p.x, y=p.y, z=p.z)
        back = Point(x=p.x, y=p.y+0.05, z=p.z+0.025)

        for i, position in enumerate([back, above, detatch, above]):
            plan = await self.plan_to_pose(Pose(position=position, orientation=self.orientations['tool_changer']), vsf=0.5, asf=0.2)
            await self.execute(plan)

            if i == 2:
                await AsyncUtils.await_for_duration(self.get_clock(), Duration(seconds=0.5))
                await self._detach_tool()

        self.attached_tool = VacuumTools.NONE
        
    async def _attach_tool(self, tool: int):
        await AsyncUtils.await_service_ready(self.attach_tool_client)

        req = AttachTool.Request()
        req.tool = tool

        result = await AsyncUtils.await_service_response(self.attach_tool_client, req)

        response = cast(AttachTool.Response, result)

        if not response.success:
            raise RuntimeError(response.message)
        
    async def _detach_tool(self):
        await AsyncUtils.await_service_ready(self.detach_tool_client)

        result = await AsyncUtils.await_service_response(self.detach_tool_client, Trigger.Request())

        response = cast(Trigger.Response, result)

        if not response.success:
            raise RuntimeError(response.message)
    