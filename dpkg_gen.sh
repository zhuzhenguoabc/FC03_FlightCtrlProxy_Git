cp ./flightctrl_proxy  /mnt/workspace/Dpkg_Make/FC03_flightctrl_proxy_snav58/usr/bin/
cp ./nfz.info  /mnt/workspace/Dpkg_Make/FC03_flightctrl_proxy_snav58/etc/fctrl_proxy/
cd /mnt/workspace/Dpkg_Make/
pwd
dpkg -b /mnt/workspace/Dpkg_Make/FC03_flightctrl_proxy_snav58/  flightctrl_proxy_1.1.16_for_snav58_8x74.deb
