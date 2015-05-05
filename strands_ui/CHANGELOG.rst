^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_ui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2014-11-11)
------------------

0.0.4 (2014-11-11)
------------------
* 0.0.3
* updated changelogs
* Contributors: Jenkins

0.0.24 (2015-05-05)
-------------------

0.0.23 (2015-04-27)
-------------------

0.0.22 (2015-04-22)
-------------------

0.0.21 (2015-04-15)
-------------------

0.0.20 (2015-04-10)
-------------------
* Adding support to start mary on different machine
* Adding sound player server to meta package
* Contributors: Christian Dondrup

0.0.19 (2015-03-31)
-------------------

0.0.18 (2015-03-28)
-------------------
* Add media server to metapackage. Closes `#71 <https://github.com/strands-project/strands_ui/issues/71>`_.
* Contributors: Chris Burbridge

0.0.17 (2015-03-24)
-------------------

0.0.15 (2015-03-09)
-------------------

0.0.11 (2014-11-22)
-------------------

0.0.10 (2014-11-20)
-------------------

0.0.9 (2014-11-14)
------------------
* adding machine tags to launch file
* Contributors: Jaime Pulido Fentanes

0.0.8 (2014-11-13)
------------------

0.0.7 (2014-11-13)
------------------

0.0.6 (2014-11-12)
------------------

0.0.5 (2014-11-11)
------------------
* 0.0.4
* updated changelogs
* 0.0.3
* updated changelogs
* Contributors: Jenkins

0.0.2 (2014-10-31)
------------------
* Removed nav_help_s* packages from metapackage run_dependencies. They are now in the recovery behaviour package.
* Creating a "metapackage" containing a strands_ui.launch file
  * The strands_ui package is not a metapackage in the pure sense of the word but has the same functionality + installs a launch file for mary and the webserver. Since metapackages are not allowed to install things it's a regualar package.
  * The strands_ui.launch file launches mary and the webserver together
  * The strands_ui package does not include the marathon gui because I didn't know if that will be released in the first place.
  * The nav_help_* launch files now have a parameter that allows to start the webserver and mary together with the help nodes. This has just been added for backwards compatibility but the default value for the parameters is false to it to peoples attention that the structure has changed.
* Contributors: Christian Dondrup
