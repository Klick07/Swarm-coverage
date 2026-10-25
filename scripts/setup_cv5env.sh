#!/usr/bin/env bash
# OpenCV 5 environment on EC2, next to ROS 2 Jazzy (run after scripts/ec2_setup.sh).
# rclpy and cv_bridge are built against Ubuntu's numpy 1.26 and OpenCV 4.6. The OpenCV 5 wheel
# declares numpy>=2, but wheels built against numpy 2 also run on numpy 1.x, so it is installed
# with --no-deps into a venv that sees the system packages. Our nodes never import cv_bridge.
set -euo pipefail
python3 -m venv --system-site-packages ~/cv5env
~/cv5env/bin/pip install -q --no-cache-dir --no-deps opencv-contrib-python-headless==5.0.0.93
# Ubuntu's python3-boto3 is too old for the Bedrock Converse API.
~/cv5env/bin/pip install -q --no-cache-dir --ignore-installed boto3 botocore s3transfer
~/cv5env/bin/pip install -q --no-cache-dir pyyaml pytest fastapi==0.121.1 uvicorn==0.38.0 python-multipart==0.0.20 httpx==0.28.1
set +u; source /opt/ros/jazzy/setup.bash; set -u
~/cv5env/bin/python -c "import cv2, numpy, rclpy; assert cv2.__version__.startswith('5'); print('cv2', cv2.__version__, 'numpy', numpy.__version__, 'rclpy ok')"
