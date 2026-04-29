#!/usr/bin/env bash
# Render Mermaid diagrams to PNG + SVG.
# Usage: bash render.sh [-a|-r|-h]
#   -a  App diagrams only (GUI, vision, poker)
#   -r  ROS 2 diagrams only
#   -h  Show usage
#   (no flag) All diagrams

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUTPUT_DIR="$SCRIPT_DIR/output"

APP=("system-architecture" "vision-pipeline" "game-state-fsm" "gui-signals" "class-relationships")
ROS2=("ros2-node-graph" "launch-modes")

# Parse flags
DIAGRAMS=()
case "${1:-}" in
  -a) DIAGRAMS=("${APP[@]}") ;;
  -r) DIAGRAMS=("${ROS2[@]}") ;;
  -h) echo "Usage: bash render.sh [-a|-r|-h]"; echo "  -a  App only"; echo "  -r  ROS 2 only"; exit 0 ;;
  "") DIAGRAMS=("${APP[@]}" "${ROS2[@]}") ;;
  *)  echo "Unknown flag: $1. Use -a, -r, or -h."; exit 1 ;;
esac

for name in "${DIAGRAMS[@]}"; do
  file="$SCRIPT_DIR/$name/$name.mmd"
  [ -f "$file" ] || { echo "Skipping $name (no .mmd file)"; continue; }
  mkdir -p "$OUTPUT_DIR/$name"
  echo "Rendering $name..."
  npx -y @mermaid-js/mermaid-cli -i "$file" -o "$OUTPUT_DIR/$name/$name.png" -b white
  npx -y @mermaid-js/mermaid-cli -i "$file" -o "$OUTPUT_DIR/$name/$name.svg" -b white
done

echo "Done. Output in $OUTPUT_DIR/"