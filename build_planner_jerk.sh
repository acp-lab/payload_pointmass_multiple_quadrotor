#!/bin/bash
set -e

echo ""
echo "Generating multiple-quad payload NMPC acados code"
echo "  - point-mass payload / multiple quadrotors"
echo ""

if [ -z "$COLCON_PAYLOAD_WS_DIR" ]; then
  echo "COLCON_PAYLOAD_WS_DIR is not set."
  echo "Example:"
  echo "  export COLCON_PAYLOAD_WS_DIR=/home/fer/payload_transportation_ws"
  exit 1
fi

CONFIG_PATH="$COLCON_PAYLOAD_WS_DIR/src/acp-autonomy-stack/config/eagle/default/dq_multiple.yaml"

python3 -m payload_pointmass_multiple_quadrotor.build_payload_planner_pointmass_multiple \
  "$CONFIG_PATH"

mkdir -p "$COLCON_PAYLOAD_WS_DIR/install/payload_pointmass_multiple_quadrotor/lib/"

if [ -f "$COLCON_PAYLOAD_WS_DIR/src/payload_pointmass_multiple_quadrotor/c_generated_code/libacados_ocp_solver_planner_payload_pointmass_multiple.so" ]; then
  cp "$COLCON_PAYLOAD_WS_DIR/src/payload_pointmass_multiple_quadrotor/c_generated_code/libacados_ocp_solver_planner_payload_pointmass_multiple.so" \
    "$COLCON_PAYLOAD_WS_DIR/install/payload_pointmass_multiple_quadrotor/lib/"
fi

echo "Generated solver artifacts under:"
echo "  $COLCON_PAYLOAD_WS_DIR/src/payload_pointmass_multiple_quadrotor/c_generated_code"
echo "  $COLCON_PAYLOAD_WS_DIR/src/payload_pointmass_multiple_quadrotor/acados_ocp_planner_payload_pointmass_multiple.json"

cd "$COLCON_PAYLOAD_WS_DIR"
source "$COLCON_PAYLOAD_WS_DIR/install/setup.bash"
colcon build --symlink-install --packages-select payload_pointmass_multiple_quadrotor
source "$COLCON_PAYLOAD_WS_DIR/install/setup.bash"
