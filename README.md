# motorNPoint
EPICS motor drivers for the following [nPoint](https://www.npoint.com/) controllers: C300 and LC400 piezo controllers

motorNPoint is a submodule of [motor](https://github.com/epics-modules/motor).  When motorNPoint is built in the ``motor/modules`` directory, no manual configuration is needed.

motorNPoint can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorNPoint contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
