^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.36 (2014-05-01)
-------------------
* Added CI with travis-ci for Groovy
* Fix memory leak in BillboardLine destructor (material not being destroyed correctly) #746
* Changed TF listener to use a dedicated thread #707
* Contributors: Jordan Brindza, Timm Linder, William Woodall

1.9.35 (2014-02-24)
-------------------
* point_cloud: back ported changes to pc iteration
  fixes `#715 <https://github.com/ros-visualization/rviz/issues/715>`_ for me
* Contributors: William Woodall
