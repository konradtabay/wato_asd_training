#!/bin/bash
# Run navigation stress scenarios against the running stack (./watod up first).
#
#   ./tests/run_nav_scenarios.sh                 # all scenarios
#   ./tests/run_nav_scenarios.sh corner_escape   # one or more by name
#   ./tests/run_nav_scenarios.sh --list          # describe scenarios
set -e
cd "$(dirname "$0")/.."

if [ ! -f modules/.env ]; then
    echo "modules/.env missing; start the stack with ./watod up first" >&2
    exit 2
fi
source modules/.env
GAZEBO="${COMPOSE_PROJECT_NAME}-gazeboserver-1"

docker cp tests/nav_scenarios.py "$GAZEBO:/tmp/nav_scenarios.py" >/dev/null
docker exec "$GAZEBO" bash -c \
    'source /opt/ros/humble/setup.bash && python3 -u /tmp/nav_scenarios.py "$@"' _ "$@"
