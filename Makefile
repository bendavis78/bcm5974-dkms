NAME	= bcm5974
VERSION	= 0.99
SRC	= usr/src
ETC	= etc/modprobe.d

TARBALL	= $(NAME)-$(VERSION).dkms.tar.gz
PATCH	= ./scripts/patch-name-version.sh $(NAME) $(VERSION)

all:	$(SRC)/$(TARBALL)

$(SRC)/$(TARBALL): $(SRC)/dkms_source_tree/$(NAME).c
	(cd $(SRC); tar cvfz $(TARBALL) dkms*)

clean:
	rm -f $(SRC)/$(TARBALL)

distclean: clean
	rm -rf *-stamp debian/$(NAME)-dkms debian/*.log debian/files

install: $(SRC)/$(TARBALL)
	install -d "$(DESTDIR)/$(SRC)"
	install -d "$(DESTDIR)/$(ETC)"
	install -m 644 $(SRC)/$(NAME)-$(VERSION).dkms.tar.gz "$(DESTDIR)/$(SRC)"
	install -m 644 $(ETC)/$(NAME) "$(DESTDIR)/$(ETC)"

bump:
	$(PATCH) debian/postinst
	$(PATCH) debian/postrm
	$(PATCH) debian/prerm
	$(PATCH) debian/rules
	$(PATCH) debian/postinst
	$(PATCH) usr/src/dkms_source_tree/dkms.conf
	chmod 0755 debian/rules
