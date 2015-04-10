^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_webserver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2014-11-11)
------------------
* Added correct information to package.xml.
* Contributors: Nick Hawes

0.0.4 (2014-11-11)
------------------
* 0.0.3
* updated changelogs
* Added correct information to package.xml.
* Contributors: Jenkins, Nick Hawes

0.0.20 (2015-04-10)
-------------------

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
* Added a script to create a simple index.html page.
* Contributors: Nick Hawes

0.0.10 (2014-11-20)
-------------------

0.0.9 (2014-11-14)
------------------

0.0.8 (2014-11-13)
------------------

0.0.7 (2014-11-13)
------------------

0.0.6 (2014-11-12)
------------------
* added install targets to fix `#48 <https://github.com/strands-project/strands_ui/issues/48>`_
* Contributors: Marc Hanheide

0.0.5 (2014-11-11)
------------------
* 0.0.4
* updated changelogs
* 0.0.3
* updated changelogs
* Added correct information to package.xml.
* Contributors: Jenkins, Nick Hawes

0.0.2 (2014-10-31)
------------------
* changed twitter_bootstrap package name
* added support for page reload
* Removed stray character.
* Added support for multiple webservers.
  The webserver now uses its name to define the service prefix. This means you can run multiple webservers in parallel to show different things, or hosted on different machines, but within the same ros system.
  Default behaviour remains unchanged, i.e. launched with:
  ```
  rosrun strands_webserver strands_webserver
  ```
  To run an extra server in parallel on port 8091 with service prefix /bob
  ```
  rosrun strands_webserver strands_webserver __name:=bob _port:=8091
  ```
* Added an action server that provides additional functionalities to display webpages like reload, going back and showing temporary pages.
* Added ability to kill webserver on exit.
* Made buttons bigger.
* Allowing local queries for javascript now.
* Passing on requests if not in strands_webserver
* Removing unnecessary README
* Removed broken include
* Added host_ip to launch file
* Added call to make client utils use retrieved server address. Fixes issue `#5 <https://github.com/strands-project/strands_ui/issues/5>`_.
* Added service call to get hostname of webserver
* Updating packages, adding rosinstall info.
* First draft readme finished. Examples updaed for clarity.
* Adding docs
* Adding example and some docs.
* Added example with services being called.
* Service calls now made with auto-generated buttons.
  It's ugly, but it works.
* Added templating for auto-generation of button-based pages with linked services.
  Current problem is that javascript lazily loaded into webserver main page is not evaluated. WIll try to fix this next with an extended message type.
* More proper package structure
* Adding more proper code structure.
* Working setup needs documentation and a proper use case.
* Added option to publish to all registered displays simulataneously by sending a 0 for display number.
* Working test script for rendering a page
* adding an almost working structure
* Contributors: Christian Dondrup, Marc Hanheide, Nick Hawes
