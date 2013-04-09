
RM    := rm -rfv

all: ros

ros:
	catkin_make

distclean:
	@- $(RM) build
	@- $(RM) devel
	@- $(RM) install
	find . \( -name "*.orig" -or -name "*~" \) -exec rm -v "{}" \;
