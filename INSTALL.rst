INSTALL
=======

1. Build
--------

::

  ./configure
  make
  make ros

2. Install
----------

::

  sudo make install # not necessary

3. Make API doc
---------------

::

  make doxygen

4. Uninstall
------------

::

  sudo make uninstall

maybe also:

::

  sudo rm -rf /usr/local/include/laustracker
  sudo rm -rf /usr/local/lib/liblaus*
  sudo rm -rf /usr/local/lib/libueye*

6. Clean everything
-------------------

::

  make distclean
