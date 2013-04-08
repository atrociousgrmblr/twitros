all: installed

GIT_DIR = build/python-twitter
GIT_URL = git://github.com/bear/python-twitter.git

include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	mkdir -p src
	cd $(GIT_DIR) && sudo pip install -r requirements.txt
	cd $(GIT_DIR) && python setup.py build
	cp $(GIT_DIR)/build/*/*.py src
	touch installed

clean:
	cd $(GIT_DIR) && rm -rf build
	rm -rf src installed

wipe: clean
	rm -rf build patched rospack*

