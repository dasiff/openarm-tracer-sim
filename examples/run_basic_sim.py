"""Basic simulation test."""

import sys
from pathlib import Path

# Auto-detect project root and add to path
current_file = Path(__file__).resolve()
examples_dir = current_file.parent
project_root = examples_dir.parent

# Try both paths (works from any directory)
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))
if str(examples_dir) not in sys.path:
    sys.path.insert(0, str(examples_dir))

from src.simulator import MaterialsLabSimulator

def main():
    print("=" * 50)
    print("Materials Lab Simulation - Basic Test")
    print("=" * 50)
    
    # Initialize (will find model automatically)
    sim = MaterialsLabSimulator()
    
    # Run for 100 steps
    print("\nRunning 100 simulation steps...")
    for i in range(100):
        sim.step()
        if i % 20 == 0:
            state = sim.get_state()
            print(f"  Step {i}: time={state['time']:.3f}s")
    
    print("\n✓ Test completed successfully!")

if __name__ == "__main__":
    main()