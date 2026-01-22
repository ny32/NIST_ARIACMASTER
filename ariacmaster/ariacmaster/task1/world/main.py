from state import WorldState
from create_tree import create_behavior_tree
import py_trees
import time

WORLD = WorldState()
def tick_tree(tree, num_ticks=1, delay=0.5):
    """Tick the tree and display the state."""
    for i in range(num_ticks):
        WORLD.tick_count += 1
        print(f"\n{'─'*60}")
        print(f"TICK {WORLD.tick_count}")
        print(f"{'─'*60}")
        # Show WORLD state BEFORE tick
        print(f"WORLD: {WORLD}")

        # Tick the tree
        tree.tick_once()

        # Show tree state AFTER tick
        print(f"\nTree Status: {tree.status}")
        print(py_trees.display.unicode_tree(root=tree, show_status=True))

        if delay > 0:
            time.sleep(delay)

if __name__ == "__main__":
    tree = create_behavior_tree()

    # Keep initialized conditions

    # Run until lane change completes
    tick_tree(tree, num_ticks=3, delay=1)
