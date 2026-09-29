ARG BASE_IMAGE=ros:humble-ros-base-jammy@sha256:1813d3c85d7f96ff7d3012d865204583255740182db5d0065f8f8cd029a83138
FROM ${BASE_IMAGE}
ENV PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
RUN sed -i 's|http://|https://|g' /etc/apt/sources.list && apt-get update && apt-get install -y --no-install-recommends python3-pip python3-numpy python3-scipy && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY requirements-container.txt /app/requirements.txt
RUN python3 -m pip install --no-cache-dir -r requirements.txt
COPY metro_detector /app/metro_detector
COPY config /app/config
COPY web /app/web
COPY launch /app/launch
COPY tools/ros_smoke.py /app/tools/ros_smoke.py
COPY examples /app/examples
LABEL org.opencontainers.image.title="Metro Obstacle Detector" org.opencontainers.image.version="0.4.2"
EXPOSE 8190
ENTRYPOINT ["/ros_entrypoint.sh", "python3", "-m", "metro_detector"]
CMD ["serve", "--host", "0.0.0.0", "--state", "/output/state"]
