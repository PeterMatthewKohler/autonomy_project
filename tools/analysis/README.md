# Week 5 analysis scaffold

Run from the repository root with the ROS 2 Python environment sourced:

```bash
source /opt/ros/humble/setup.bash
python3 tools/analysis/analyze_mission.py bags/week5_mission
```

The script opens the bag, filters `/chassis/odom`, deserializes Odometry
messages, and prints the first message's frame identifiers and total sample
count. It uses the storage plugin from the bag metadata; override it with
`--storage-id sqlite3` or `--storage-id mcap` if needed. The matching plugin
must be installed in your ROS environment.

Complete the `TODO (you)` comments in the script for your measurement data and
frame choices, geometry, ideal distance, path stretch, and interpretation.
The sample count checks the reader; it does not confirm full mission coverage.
No performance metric is calculated by this scaffold.

The assignment and AI-use boundary are in
[the week 5 plan](../../docs/plans/2026-10-19-week-05.md).
