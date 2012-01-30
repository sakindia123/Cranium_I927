make -j8
cp arch/arm/boot/zImage /home/sarthak/Downloads/kitchen_4g/output
cp drivers/net/wireless/bcm4330/dhd.ko /home/sarthak/Downloads/kitchen_4g/output
cp drivers/bluetooth/bthid/bthid.ko /home/sarthak/Downloads/kitchen_4g/output
cp drivers/scsi/scsi_wait_scan.ko /home/sarthak/Downloads/kitchen_4g/output
cd /home/sarthak/Downloads/kitchen_4g
./strip
./menu
