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

Forthcoming
-----------

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
