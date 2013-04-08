all: installed

SVN_DIR = build/libtwitcurl
SVN_URL = https://twitcurl.googlecode.com/svn/trunk/libtwitcurl
SVN_REVISION = -r103

include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR)
	mkdir include include/curl lib
	cd $(SVN_DIR) && make
	cp $(SVN_DIR)/*.h include/
	cp $(SVN_DIR)/curl/*.h include/curl/
	cp $(SVN_DIR)/libtwitcurl* lib/
	ln -sv libtwitcurl.so.1.0 lib/libtwitcurl.so
	ln -sv libtwitcurl.so.1.0 lib/libtwitcurl.so.1
	touch installed

clean:
	cd $(SVN_DIR) && make clean
	rm -rf lib include installed

wipe: clean
	rm -rf build patched rospack*
