#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/kauvoy/gps_ws/src/GPS_Package/geonav_transform"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/kauvoy/gps_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/kauvoy/gps_ws/install/lib/python3/dist-packages:/home/kauvoy/gps_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/kauvoy/gps_ws/build" \
    "/home/kauvoy/.conda/envs/main/bin/python3" \
    "/home/kauvoy/gps_ws/src/GPS_Package/geonav_transform/setup.py" \
     \
    build --build-base "/home/kauvoy/gps_ws/build/GPS_Package/geonav_transform" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/kauvoy/gps_ws/install" --install-scripts="/home/kauvoy/gps_ws/install/bin"
