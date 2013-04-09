##############################################################
#               CMake Project Wrapper Makefile               #
############################################################## 

SHELL := /bin/bash
RM    := rm -rfv
CS    := ./tools/bin/cs

.PHONY: ros

all: ./build/Makefile
	@ $(MAKE) -C build

ros: 
	@ (cd ros >/dev/null 2>&1 && make ros)

./build/Makefile:
	@ (cd build >/dev/null 2>&1 && cmake ..)

style:
	@ (cd src >/dev/null 2>&1 && ../tools/bin/cs style)
	@ (cd include >/dev/null 2>&1 && ../tools/bin/cs style)
	@ (cd ros/src >/dev/null 2>&1 && ../../tools/bin/cs style)

distclean:
	@- (cd build >/dev/null 2>&1 && cmake .. >/dev/null 2>&1)
	@- $(MAKE) --silent -C build clean || true
	@- $(RM) ./build/Makefile
	@- $(RM) ./build/src
	@- $(RM) ./build/test
	@- $(RM) ./build/CMake*
	@- $(RM) ./build/cmake.*
	@- $(RM) ./build/*.cmake
	@- $(RM) ./build/*.txt
	@- $(RM) ./build/examples
	@- $(RM) ./build/liblaustracker
	@- $(RM) ./build/thirdparty
	@- $(RM) ./doxygen/html/*.html
	@- $(RM) ./doxygen/html/*.css
	@- $(RM) ./doxygen/html/*.png
	@- $(RM) ./doxygen/html/*.jpg
	@- $(RM) ./doxygen/html/*.gif
	@- $(RM) ./doxygen/html/*.tiff
	@- $(RM) ./doxygen/html/*.php
	@- $(RM) ./doxygen/html/*.js
	@- $(RM) ./doxygen/html/search
	@- $(RM) ./doxygen/html/installdox
	@- find . \( -name "*.orig" -or -name "*~" \) -exec rm -v "{}" \;
	@- (cd ros && make distclean)

ifeq ($(findstring $(MAKECMDGOALS), ros distclean style),)
    $(MAKECMDGOALS): ./build/Makefile
	@ $(MAKE) -C build $(MAKECMDGOALS)
endif

