^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plotjuggler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.1 (2017-02-14)
-------------------
* adding more command line functionality
* BUG-FIX: bad resizing when a matrix row or column is deleted
* simplifying how random colors are managed
* more streaming buffer
* remember selected topics
* improvements and bug fixes
* Contributors: Davide Faconti

0.10.0 (2017-02-12)
-------------------
* auto loading of streamer based on saved layout
* refactoring of the ROS plugins 
* REFACTORING to allow future improvements of drag&drop
* trying to fix a compilation problem
* Update README.md
* FIX: menu bar will stay where it is supposed to.
* Contributors: Davide Faconti

0.9.1 (2017-02-09)
------------------
* FIX: avoid the use of catkin when using plain cmake
* IMPROVEMENT: exit option in the file menu
* IMPROVEMENT: reduce the number of steps to launch a streamer
* SPEEDUP: use a cache to avoid repeated creation of std::string
* better way to stop streaming and reload the plugins
* fixed a compilation problem on windows
* fixed a problem with resizing
* help menu with About added
* qDebug commented
* default to RelWithDebInfo
* Contributors: Davide Faconti

0.9.0 (2017-02-07)
------------------
* bug fixes
* QWT submodule removed
* removed boost dependency
* Contributors: Davide Faconti

* remove submodule
* Contributors: Davide Faconti

0.8.1 (2017-01-24)
------------------
* removing the old name "SuperPlotter"
* bug fix that affected data streaming
* this explicit dependency might be needed by bloom

0.8.0 (2017-01-23)
------------------
* First official beta of PJ
* Contributors: Arturo Martín-de-Nicolás, Davide Faconti, Kartik Mohta, Mikael Arguedas
