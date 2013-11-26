Xares
=====

*eXploration module for Autonomous Robotic Embedded Systems*

Perform exploration planning, while using data provided by the gladys_[1] API.

[1] https://github.com/pierriko/gladys

INSTALL
-------

    git clone http://trac.laas.fr/git/xares && cd xares
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/dev/ ..
    make -j8 && make test && make install

CONTRIBUTE
----------

Code is available on GitHub at https://github.com/cyrobin/xares/

Feel free to fork, pull request or submit issues to improve the project!

* https://github.com/cyrobin/xares/fork
* https://github.com/cyrobin/xares/issues
* https://github.com/cyrobin/xares/pulls
* https://help.github.com/articles/fork-a-repo
* https://help.github.com/articles/using-pull-requests

### STYLE

Please configure your editor to insert 4 spaces instead of TABs, maximum line
length to 79, `lower_case_with_underscores` instead of `CamelCase`. Most of the
rules are taken from [Python PEP8](http://www.python.org/dev/peps/pep-0008/)

Other ideas can be found in Google Guides:
[Python](http://google-styleguide.googlecode.com/svn/trunk/pyguide.html),
[C++](http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml).


LICENSE
-------

[BSD 2-Clause](http://opensource.org/licenses/BSD-2-Clause)
