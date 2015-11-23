make -j8
sudo make modules_install
sudo cp arch/x86_64/boot/bzImage /boot/vmlinuz-3.18.0-rc7-vgt-2015q3+ -iv
sudo mkinitramfs -o /boot/initrd.img-3.18.0-rc7-vgt-2015q3+ -v 3.18.0-rc7-vgt-2015q3+
