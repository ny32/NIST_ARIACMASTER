import asyncio

import rclpy
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor

from ariac_interfaces.msg import CellTypes, InspectionReport

from ariacmaster.environment_interface import Environment
from ariacmaster.sensors_interface import Sensors
from ariacmaster.utils import AsyncUtils

from ariacmaster.robot_interface import Robot

async def run():
    rclpy.init()

    # Initialization snippet
    executor = MultiThreadedExecutor()
    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(AsyncUtils.spin_executor(executor, shutdown_event))

    environment = Environment()
    sensors = Sensors()
    executor.add_node(environment)
    executor.add_node(sensors)

    IR1 = Robot("inspection_robot_1")
    executor.add_node(IR1)

    #Shorthand for logging
    elog = environment.get_logger().info
    try:
        elog("Waiting for environment to be ready")
        await environment.competition.ready.wait()

        elog("Waiting on Inspection Robot 1...")
        await IR1.ready()
    except Exception as e:
        print(f"\n\n\n---------\n{e}\n-------------\n\n\n")
    finally:
        shutdown_event.set()
        await spin_task
def main():
    asyncio.run(run())
