import asyncio

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ariac_interfaces.msg import VacuumTools

from ariacmaster.environment_interface import Environment
from ariacmaster.robots import AssemblyRobot2
from ariacmaster.utils import AsyncUtils

async def run():
    rclpy.init()

    # Create and spin executor
    executor = MultiThreadedExecutor()
    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(AsyncUtils.spin_executor(executor, shutdown_event))

    environment = Environment()
    executor.add_node(environment)

    # Create robot nodes
    ar2 = AssemblyRobot2()
    executor.add_node(ar2)

    try:
        # Wait for competition to be ready
        environment.get_logger().info("Waiting for environment to be ready")
        await environment.competition.ready.wait()

        await ar2.ready()

        for i in range(10):
            if i%2 == 0:
                await ar2.pick_tool_from_stand(VacuumTools.VG_2)
            else:
                await ar2.pick_tool_from_stand(VacuumTools.VG_4)
            
            await ar2.return_tool_to_stand()

    except Exception as e:
        print(5*'\n', e, 5*'\n')
    finally:
        shutdown_event.set()
        await spin_task  

def main():
    asyncio.run(run())