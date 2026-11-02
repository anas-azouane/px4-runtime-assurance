SUMMARY = "Route MAVLink between UART and UDP endpoints"
HOMEPAGE = "https://github.com/mavlink-router/mavlink-router"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://LICENSE;md5=93888867ace35ffec2c845ea90b2e16b"

SRC_URI = "gitsm://github.com/mavlink-router/mavlink-router.git;protocol=https;branch=master \
           file://main.conf \
           "
SRCREV = "42529d55b665e0a9a29e424e186f514c56c2e5b5"
S = "${WORKDIR}/git"

inherit meson pkgconfig systemd

# An explicit path keeps meson from asking for systemd.pc, so this also builds
# on sysvinit images like the DK1's poky.
EXTRA_OEMESON = "-Dsystemdsystemunitdir=${systemd_system_unitdir}"
FILES:${PN} += "${systemd_system_unitdir}"

SYSTEMD_SERVICE:${PN} = "mavlink-router.service"
# Enabled by hand for manual flights. The bench proxy owns the tty in CI.
SYSTEMD_AUTO_ENABLE = "disable"

do_install:append() {
    install -d ${D}${sysconfdir}/mavlink-router
    install -m 0644 ${WORKDIR}/main.conf ${D}${sysconfdir}/mavlink-router/main.conf
}
