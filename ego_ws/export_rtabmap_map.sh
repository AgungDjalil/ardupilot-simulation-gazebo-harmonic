#!/usr/bin/env bash
# Export an RTAB-Map map of the onboard stack to a .ply point cloud (open in CloudCompare / MeshLab).
# usage: ./export_rtabmap_map.sh [database.db] [output_dir]
#   default: ~/.ros/realsense_rtabmap.db -> ~/ego_maps/
# The map is only saved when the onboard stack runs with save:=true (./run_onboard_local.sh save:=true
# here; on the Pi ~/drone/run_onboard.sh --sim save:=true, then copy the .db from the Pi's ~/.ros).
# The database stays after the stack stopped; also open it with: rtabmap-databaseViewer <db>
DB=${1:-$HOME/.ros/realsense_rtabmap.db}
OUT=${2:-$HOME/ego_maps}
source /opt/ros/jazzy/setup.bash
if [ ! -s "$DB" ]; then
  echo "Database not found: $DB (run the onboard stack with save:=true and stop it first)" >&2
  exit 1
fi
mkdir -p "$OUT"
NAME=$(basename "$DB" .db)_$(date +%Y%m%d_%H%M%S)
# Depth up to 5 m (the range of EGO's map), 5 cm voxels.
rtabmap-export --cloud --poses --max_range 5 --voxel 0.05 --output "$NAME" --output_dir "$OUT" "$DB"
ls -l "$OUT"/"$NAME"* 2>/dev/null
