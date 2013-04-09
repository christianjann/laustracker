# providing shortcuts to cmake for building outside the source tree
# Michael Kaess, 2010

make = make -j 2 -C build --no-print-directory

all: build build/Makefile
	@$(make)

# the actual build directory
build:
	@mkdir -p build lib include

# populate the build directory
build/Makefile:
	cd build && cmake ..

install:
	cd build && sudo make install

style:
	find -regex ".*[.][CHch]p*" -exec astyle  --style=attach "{}" \;

.PHONY: doc
doc:
	@doxygen Doxyfile

.PHONY: eclipse
eclipse:
	@rm -rf ../build_libueyecam
	@mkdir ../build_libueyecam
	cd ../build_libueyecam && cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../trunk
	@echo "===================================================================================="
	@echo "Eclipse .project and .cproject files are located within ../build_libueyecam"
	@echo "1. Import project using Menu File->Import"
	@echo "2. Select General-> Existing projects into workspace:"
	@echo "3. Browse where your build tree is and select the root build tree directory. Keep Copy projects into workspace unchecked."
	@echo "4. You get a fully functional eclipse project"
	@echo "For example, ctrl+b within a source file will compile the project"

.PHONY: distclean
distclean:
	@rm -rf build doc/html lib bin release
	@find . \( -name "*.orig" -or -name "*~" \) -exec rm -v "{}" \;
#	@find . -name CMakeFiles |xargs rm -rf # clean up in case "cmake ." was called
#	@find . -name cmake_install.cmake -delete
#	@find . -name CMakeCache.txt -delete


.PHONY: examples
examples:
	@$(make) examples

# default target: any target such as "clean", "example"...
# is simply passed on to the cmake-generated Makefile 
%::
	@$(make) $@
