FROM ros:melodic
WORKDIR /root/

RUN apt-get update && apt-get install -y wget unzip vim libeigen3-dev libboost-all-dev

# install Ceres solver (latest stable version by 06/20/2020)
RUN apt-get update && apt-get install -y cmake libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev
RUN wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz && tar zxf ceres-solver-1.14.0.tar.gz && rm ceres-solver-1.14.0.tar.gz && \
    cd ceres-solver-1.14.0 && mkdir build && cd build && cmake .. && make -j5 && make test && make install && cd ../.. && rm -rf ceres-solver-1.14.0


# install gtsam 4.0.3 (latest release version by 06/20/2020)
RUN git clone https://github.com/borglab/gtsam.git  && cd gtsam && mkdir build && cd build && cmake -DGTSAM_WITH_TBB=Off .. && make -j5 && make install && \
    cd ../../ && rm -rf gtsam

# setup base ros env
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && mkdir -p /root/slam_ws/src && cd /root/slam_ws/src && catkin_init_workspace && cd .. && catkin_make"
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc && echo "source /root/slam_ws/devel/setup.bash" >> /root/.bashrc
RUN echo "export LD_LIBRARY_PATH=/root/slam_ws/devel/lib:/opt/ros/melodic/lib:/lib:/usr/lib:/usr/local/lib" >> /root/.bashrc

