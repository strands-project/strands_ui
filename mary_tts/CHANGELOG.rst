^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mary_tts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2014-11-11)
------------------

0.0.4 (2014-11-11)
------------------
* 0.0.3
* updated changelogs
* Contributors: Jenkins

0.0.23 (2015-04-27)
-------------------

0.0.22 (2015-04-22)
-------------------

0.0.21 (2015-04-15)
-------------------

0.0.20 (2015-04-10)
-------------------
* Update marybridge.py
  Setting default for mary host to `localhost`
* Adding support to start mary on different machine
* Contributors: Christian Dondrup

0.0.19 (2015-03-31)
-------------------

0.0.18 (2015-03-28)
-------------------

0.0.17 (2015-03-24)
-------------------

0.0.15 (2015-03-09)
-------------------

0.0.11 (2014-11-22)
-------------------

0.0.10 (2014-11-20)
-------------------
* [mary_tts] Added troubleshooting info
* Contributors: Christian Dondrup

0.0.9 (2014-11-14)
------------------

0.0.8 (2014-11-13)
------------------
* Fixing a bug in the naming of the local en_US.
* Contributors: Christian Dondrup

0.0.7 (2014-11-13)
------------------
* Merge branch 'hydro-devel' of http://github.com/strands-project/strands_ui into hydro-devel
* Fixing `#52 <https://github.com/strands-project/strands_ui/issues/52>`_:
  * Enabling selection of `en_GB` locale
  * Setting `en_GB` as default locale because the default language is part of that
  Fixing `#50 <https://github.com/strands-project/strands_ui/issues/50>`_:
  * removed autostart from action server
  Additional bugfix:
  * If an empty string is pushied a warning will be diplayed and the ervice or action server report failure. This prevents mary from crashing when trying to say nothing.
* Adding correct permissions to marytts-5.0 install target
  Fixing `#53 <https://github.com/strands-project/strands_ui/issues/53>`_
* Contributors: Christian Dondrup

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
* Updating dependencies.
  Preparing for release.
* moving human_help_manager service definition to human_help_manager pa…
  …ckage
* - made the Mary start script ROS compatible and installable
  - disabled file logging, closing https://github.com/strands-project/strands_hri/issues/6
* - made speak_webserver installable
  - made template and static dirs to be found
  - tidied up
* changed all ros_mary_tts to mary_tts
* changed package name
* Add 'mary_tts/' from commit 'ee851b0aa5851bc39fe6f13a6c3522d9f4783b74'
  git-subtree-dir: mary_tts
  git-subtree-mainline: 252851fb65fc7aa6c5439377d75a6899794efb36
  git-subtree-split: ee851b0aa5851bc39fe6f13a6c3522d9f4783b74
* Contributors: Bruno Lacerda, Christian Dondrup, Marc Hanheide
