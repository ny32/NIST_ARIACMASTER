import py_trees
import core.constants as constants
import action_nodes.competition_behaviors as competition
import action_nodes.inspection_behaviors as inspection_node
import action_nodes.agv_behaviors as agv
import action_nodes.ir_behaviors as ir_node
import condition_nodes as condition
def create_behavior_tree():
        # Create the behavior tree:
        # Nodes without children...
        root = py_trees.composites.Sequence(
        name="Root",
        memory=False  # Re-evaluate from first child each tick
        )
        parallel_tasks = py_trees.composites.Parallel(
            name="Parallel Tasks",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
        )
    
        inspection = py_trees.composites.Sequence(
            name="Inspection",
            memory=False
        )

        cell_check = py_trees.composites.Selector(
            name="Cell Check",
            memory=False
        )

        conveyor_pickup = py_trees.composites.Sequence(
            name="Conveyor Pickup",
            memory=False
        )

        wait_for_queue = py_trees.composites.Selector(
            name="Wait for Queue",
            memory=False
        )

        wait_sequence = py_trees.composites.Sequence(
            name="Wait Sequence",
            memory=False
        )

        ir1_tester_selection = py_trees.composites.Selector(
            name="IR1 Tester Selection",
            memory=False
        )

        ir1_tester1_sequence = py_trees.composites.Sequence(
            name="IR1 Tester 1 Sequence",
            memory=False
        )
        
        ir1_tester2_sequence = py_trees.composites.Sequence(
            name="IR1 Tester 2 Sequence",
            memory=False
        )

        voltage_inspection = py_trees.composites.Sequence(
            name="Voltage Inspection",
            memory=False
        )

        check_testers_and_wait = py_trees.composites.Selector(
            name="Check Testers and Wait",
            memory=False
        )

        ir2_tester_selection = py_trees.composites.Selector(
            name="IR2 Tester Selection",
            memory=False
        )

        ir2_check_tester1 = py_trees.composites.Sequence(
            name="IR2 Check Tester 1",
            memory=False
        )

        ir2_check_tester2 = py_trees.composites.Sequence(
            name="IR2 Check Tester 2",
            memory=False
        )

        recycle_or_keep_cell_selector = py_trees.composites.Selector(
            name="Recycle or Keep Cell",
            memory=False
        )

        take_good_cell_to_agv = py_trees.composites.Sequence(
            name="Take Good Cell to AGV",
            memory=False
        )

        dispose_bad_cell = py_trees.composites.Sequence(
            name="Dispose Bad Cell",
            memory=False
        )

        agv_movement = py_trees.composites.Parallel(
            name="AGV Movement",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
        )

        get_agv_to_assembly = py_trees.composites.Sequence(
            name="Get AGV to Assembly",
            memory=False
        )

        yield_to_assembly_agv = py_trees.composites.Sequence(
            name="Yield to Assembly AGV",
            memory=False
        )
        return_agv_to_inspection = py_trees.composites.Sequence(
            name="Return AGV to Inspection",
            memory=False
        )


        # Decorators/Inverters added here w/Children
        wait_for_cell_arrival = py_trees.decorators.Retry(
            name="Wait for Cell Arrival",
            child=wait_for_queue,
            num_failures=constants.NORMAL_RETRIES
        )
        not_queued_cell = py_trees.decorators.Inverter(
            name="Not Queued Cell",
            child=condition.QueuedCell()
        )
        ir1_wait_for_tester = py_trees.decorators.Retry(
            name="IR1 Wait for Tester",
            child=ir1_tester_selection,
            num_failures=constants.NORMAL_RETRIES
        )
        wait_for_filled_tester = py_trees.decorators.Retry(
            name="Wait for Filled Tester",
            child=check_testers_and_wait,
            num_failures=constants.NORMAL_RETRIES
        )

        voltage_tester1_filled = py_trees.decorators.Inverter(
            name="Voltage Tester 1 Filled",
            child=condition.VoltageTester1Free()
        )
        voltage_tester2_filled = py_trees.decorators.Inverter(
            name="Voltage Tester 2 Filled",
            child=condition.VoltageTester2Free()
        )

        keep_yielding = py_trees.decorators.Retry(
            name="Keep Yielding",
            child=yield_to_assembly_agv,
            num_failures=constants.NORMAL_RETRIES
        )

        no_agv_at_assembly = py_trees.decorators.Inverter(
            name="No AGV at Assembly",
            child=condition.AGVAtAssembly()
        )

        # Build the tree by adding children to parents here... (do later)

        # Construct full tree now
        root.add_children([
            competition.StartCompetition(),
            parallel_tasks,
            competition.EndCompetition()
        ])

        parallel_tasks.add_children([
            inspection,
            agv_movement,
            voltage_inspection,
            agv_movement
        ])
        inspection.add_children([
            inspection_node.ConstructLIDARModel(),
            inspection_node.PopulateReport(),
            inspection_node.SubmitReport(),
            cell_check
        ])
        cell_check.add_children([
            condition.DefectFound(),
            inspection_node.OpenInspectionDoor()
        ])
        conveyor_pickup.add_children([
            ir_node.IR1MoveToPickup(),
            wait_for_cell_arrival,
            ir_node.IR1GraspCell(),
            ir1_wait_for_tester,
            ir_node.IR1PlaceInTester()
        ])
        wait_for_queue.add_children([
            condition.QueuedCell(),
            wait_sequence
        ])
        wait_sequence.add_children([
            not_queued_cell,
            ir_node.Wait1Sec()
        ])
        ir1_tester_selection.add_children([
            ir1_tester1_sequence,
            ir1_tester2_sequence
        ])
        ir1_tester1_sequence.add_children([
            condition.VoltageTester1Free(),
            ir_node.IR1MoveToTester1()
        ])
        ir1_tester2_sequence.add_children([
            condition.VoltageTester2Free(),
            ir_node.IR1MoveToTester2()
        ])
        voltage_inspection.add_children([
            wait_for_filled_tester,
            ir_node.TakeCurrentCellVoltage(),
            ir_node.IR2MoveToCurrentCell(),
            ir_node.IR2GraspCell(),
            recycle_or_keep_cell_selector
        ])
        check_testers_and_wait.add_children([
            ir_node.Wait1Sec(),
            ir2_tester_selection
        ])
        ir2_tester_selection.add_children([
            ir2_check_tester1,
            ir2_check_tester2
        ])
        ir2_check_tester1.add_children([
            voltage_tester1_filled,
            ir_node.setIR2TargetCellTo1()
        ])
        ir2_check_tester2.add_children([
            voltage_tester2_filled,
            ir_node.setIR2TargetCellTo2()
        ])
        recycle_or_keep_cell_selector.add_children([
            take_good_cell_to_agv,
            dispose_bad_cell
        ])
        take_good_cell_to_agv.add_children([
            condition.CurrentCellTolerable(),
            agv.FindNearestFreeAGVSlot(),
            ir_node.IR2MoveToAGVSlot(),
            ir_node.IR2PlaceInSlot()
        ])
        dispose_bad_cell.add_children([
            ir_node.IR2MoveToRecycling(),
            ir_node.IR2ReleaseCell()
        ])
        agv_movement.add_children([
            get_agv_to_assembly,
            return_agv_to_inspection
        ])
        get_agv_to_assembly.add_children([
            agv.LocateFilledInspectionAGVs(),
            agv.MoveAGVsToIntersection(),
            keep_yielding,
            agv.MoveAGVToAssembly()
        ])
        yield_to_assembly_agv.add_children([
            agv.Wait1Sec(),
            no_agv_at_assembly
        ])
        return_agv_to_inspection.add_children([
            condition.AssemblyAGVSlotsEmpty(),
            agv.MoveAssemblyAGVToInspection()
        ])
