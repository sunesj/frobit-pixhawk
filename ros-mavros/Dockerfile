FROM ros:melodic
RUN apt-get update && \
    apt-get install ros-melodic-mavros -y

# Mavros dependency
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm ./install_geographiclib_datasets.sh