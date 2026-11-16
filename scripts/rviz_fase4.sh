#!/bin/bash
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$D/env_external.sh"
exec rviz2 -d "$D/../rviz/fase4.rviz"
