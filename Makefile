# makefile for compare_pcl

VERSION=1.0.0-1
DEST_DIR=/usr/local/lib/lib3d
PKG_NAME=danbots-tools-lib3d-$(VERSION)
PKG_FOLDER=tmp/package

help:
	@echo "make install to install in /usr/local/bin"
	@echo "make deb-pkg to make an deb installation packet"

install:
	#chmod a+x prg/*.py
	mkdir -p $(DEST_DIR)
	cp lib3d/__init__.py lib3d/registration.py $(DEST_DIR)

uninstall:
	rm -f $(DEST_DIR)/noise_removal.py
	rm -f $(DEST_DIR)/pcl_utils.py
	rm -f $(DEST_DIR)/registration.py
	rm -f $(DEST_DIR)/stitch.py
	-rmdir $(DEST_DIR)

copy-pkg:
	#mkdir -p $(PKG_FOLDER)
	mkdir -p $(PKG_FOLDER)$(DEST_DIR)
	@echo Copy files
	cp -r lib3d/* $(PKG_FOLDER)$(DEST_DIR)

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
