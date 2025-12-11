import asyncio

import rclpy
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor

from ariac_interfaces.msg import CellTypes, InspectionReport

from ariacmaster.environment_interface import Environment
from ariacmaster.sensors_interface import Sensors
from ariacmaster.utils import AsyncUtils


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

    #Shorthand for logging
    elog = environment.get_logger().info

    try:
        elog("Waiting for environment to be ready")
        await environment.competition.ready.wait()

        elog("Starting competition")
        await environment.competition.start()

        elog("Starting cell feed with Lithium Ion Cells")
        await environment.inspection_conveyor.start_cell_feed(CellTypes.LI_ION)
        max_cells = 8
        for i in range(max_cells):
        
            current_cell = await sensors.inspection_bb.cell_queue.get()
            elog("Cell Detected")

            # Simulate inspection time
            elog("Wait... Inpecting Cell")
            await AsyncUtils.await_for_duration(environment.get_clock(), Duration(seconds=1))

            # Implement logic to look for defects in cell
            """
            environment.get_logger().info("Submitting inspection report")
            report = InspectionReport(passed=True)
            await environment.inspection_conveyor.submit_inspection_report(report)
            """


            if i == max_cells / 2:
                environment.get_logger().info("Changing cell feed type to Nickel Metal Hydride Cells")
                await environment.inspection_conveyor.start_cell_feed(CellTypes.NIMH)
            
        # End loop
                # Wait for last cell to pass through inspection door
        await AsyncUtils.await_for_duration(environment.get_clock(), Duration(seconds=0.7 / environment.inspection_conveyor.speed)) 

        environment.get_logger().info("Maximum capacity is 8: Loop exited, stopping cell feed")
        await environment.inspection_conveyor.stop_cell_feed()

        await environment.competition.end(shutdown=False)
    except Exception as e:
        print(f"\n\n\n---------\n{e}\n-------------\n\n\n")
    finally:
        shutdown_event.set()
        await spin_task

def main():
    asyncio.run(run())