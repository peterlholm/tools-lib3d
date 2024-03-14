# makefile for compare_pcl

VERSION=1.0.0-1
PKG_NAME=danbots-tools-lib3d-$(VERSION)
PKG_FOLDER=tmp/package

help:
	@echo "make install to install in /usr/local/bin"
	@echo "make deb-pkg to make an deb installation packet"

install:
	#chmod a+x prg/*.py
	mkdir -p /usr/local/lib/lib3d
	cp lib3d/* /usr/local/lib/lib3d

uninstall:
	rm -f /usr/local/lib/lib3d/noise_removal.py
	rm -f /usr/local/lib/lib3d/pcl_utils.py
	rm -f /usr/local/lib/lib3d/registration.py
	rm -f /usr/local/lib/lib3d/stitchl.py

copy-pkg:
	#mkdir -p $(PKG_FOLDER)
	mkdir -p $(PKG_FOLDER)/usr/local/lib/lib3d
	@echo Copy files
	cp -r lib3d/* $(PKG_FOLDER)/usr/local/lib/lib3d

deb-pkg:	copy-pkg
	mkdir -p $(PKG_FOLDER)/DEBIAN
	cp deb_pkg/DEBIAN/* $(PKG_FOLDER)/DEBIAN
	dpkg-deb --build --root-owner-group $(PKG_FOLDER) tmp/$(PKG_NAME).deb

deb-push:
	rcp tmp/$(PKG_NAME).deb  danbots:/var/www/apt/simple/pool/tools/
	rsh danbots /var/www/apt/simple/scan

clean:
	#rm -rf $(PKG_FOLDER)
	#rm -f $(PKG_FOLDER).deb
	rm -r tmp
	#rm -f out.ply
