SUMMARY = "Python MAVLink bindings"
HOMEPAGE = "https://github.com/ArduPilot/pymavlink"
LICENSE = "LGPL-3.0-only"
LIC_FILES_CHKSUM = "file://COPYING;md5=6ea13ec5f0f3dd35ac5b53afdc3ed9ff"

PYPI_PACKAGE = "pymavlink"
SRC_URI[sha256sum] = "8d65a8b05b2971babeef4b6ca74a247a72276f62f7559466c64e96ece7f10062"
inherit pypi setuptools3

DEPENDS += "python3-lxml-native python3-future-native"
RDEPENDS:${PN} = "python3-modules python3-lxml"
