pcl_online_viewer
=================

This (ROS_) package contains a minimal point cloud viewer (and grabber) based on the PCLVisualizer_.
It is based on fairly old example code, enriched by some simplistic grabbing functionality and fully supports ROS_ (via catkin_).

In order to visualize a point cloud (online), simpy run::

  rosrun pcl_online_viewer pcl_online_viewer input:=/CLOUD

where ``/CLOUD`` is the topic where the cloud to be visualized is published.

.. _ROS: http://ros.org
.. _PCLVisualizer: http://pointclouds.org
.. _catkin: http://wiki.ros.org/catkin

