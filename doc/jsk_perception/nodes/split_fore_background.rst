split_fore_background.py
========================

What is this?
-------------

.. image:: ./images/split_fore_background.png

Publish foreground and background image splitted by local maximum of depth image.


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Raw image.

* ``~input/depth`` (``sensor_msgs/Image``, encoding: ``16UC1`` or ``32FC1``)

  Depth image.


Publishing Topic
----------------

* ``~output/fg`` (``sensor_msgs/Image``)

  Foreground image.

* ``~output/bg`` (``sensor_msgs/Image``)

  Background image.

* ``~output/fg_mask`` (``sensor_msgs/Image``, encoding: ``8UC1``)

  Mask image to extract foreground.

* ``~output/bg_mask`` (``sensor_msgs/Image``, encoding: ``8UC1``)

  Mask image to extract background
