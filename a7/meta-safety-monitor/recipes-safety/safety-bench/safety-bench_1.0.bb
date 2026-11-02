SUMMARY = "Fault-injection proxy and M4 deploy script for the safety monitor bench"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://bench_proxy.py \
           file://deploy_m4.sh \
           "
S = "${WORKDIR}"

RDEPENDS:${PN} = "python3-pymavlink"

do_install() {
    install -d ${D}/opt/safety
    install -m 0755 ${WORKDIR}/bench_proxy.py ${D}/opt/safety/
    install -m 0755 ${WORKDIR}/deploy_m4.sh ${D}/opt/safety/
}
FILES:${PN} = "/opt/safety"
