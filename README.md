# Panasonic Servo interface over Ethercat implemented using SOEM Library

This software allows selection of direction, torque, velocity of the servo Minas A6. 
Base code is SOEM. Driver implementation and application layer added for implementation.
For any similar implementation requirements contact: aditya.ayachit@ianet.in
For video of this in action check out: https://ianet.in/mechatronictestequipment/







# Simple Open EtherCAT Master Library
[![Build Status](https://github.com/OpenEtherCATsociety/SOEM/workflows/build/badge.svg?branch=master)](https://github.com/OpenEtherCATsociety/SOEM/actions?workflow=build)

BUILDING
========


Prerequisites for all platforms
-------------------------------

 * CMake 3.9 or later


Windows (Visual Studio)
-----------------------

 * Start a Visual Studio command prompt then:
   * `mkdir build`
   * `cd build`
   * `cmake .. -G "NMake Makefiles"`
   * `nmake`

Linux & macOS
--------------

   * `mkdir build`
   * `cd build`
   * `cmake ..`
   * `make`

ERIKA Enterprise RTOS
---------------------

 * Refer to http://www.erika-enterprise.com/wiki/index.php?title=EtherCAT_Master

Documentation
-------------

See https://openethercatsociety.github.io/doc/soem/


Want to contribute to SOEM or SOES?
-----------------------------------

If you want to contribute to SOEM or SOES you will need to sign a Contributor
License Agreement and send it to us either by e-mail or by physical mail. More
information is available in the [PDF](http://openethercatsociety.github.io/cla/cla_soem_soes.pdf).
