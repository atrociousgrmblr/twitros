all: installed

GIT_DIR = build/twython
GIT_URL = git://github.com/ryanmcgrath/twython.git

include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	mkdir -p src
	cd $(GIT_DIR) && python setup.py build
	cp -r $(GIT_DIR)/build/*/twython src
	touch installed

clean:
	cd $(GIT_DIR) && rm -rf build
	rm -rf src installed

wipe: clean
	rm -rf build patched rospack*

