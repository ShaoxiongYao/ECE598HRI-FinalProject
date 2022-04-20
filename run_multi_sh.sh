#!/bin/bash
set -Eeuo pipefail
set -x

(trap 'kill 0' SIGINT;\
python test_sh.py &\

python test_sh.py &\

python test_sh.py)