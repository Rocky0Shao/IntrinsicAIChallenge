#!/bin/bash
# Original command (raw wrist_wrench from /observations):
# pixi run ros2 topic echo /observations | grep -A 14 wrist_wrench

# Tared force magnitude is now printed by the CheatCode teleop process.
# Look for "[CheatCode] Tared Force Mag: ..." in the teleop terminal.
# This script greps the teleop's stdout for those lines.
# Run this in the SAME terminal as your teleop, or pipe teleop output here.

echo "Listening for [CheatCode] tared force prints..."
echo "(Make sure teleop is running with the updated aic_teleop.py)"
echo ""
pixi run ros2 topic echo /observations | python3 -c "
import sys, re
for line in sys.stdin:
    if 'wrist_wrench' in line:
        # Collect the next 14 lines for context
        block = [line]
        for _ in range(14):
            block.append(next(sys.stdin, ''))
        print(''.join(block))
"
