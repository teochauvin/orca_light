# Build every algorithm (each subfolder with a CMakeLists.txt) and copy the
# resulting Python modules (.so) into ../simulations.
#
#   make              build all algorithms
#   make rvo          build a single algorithm
#   make clean        remove build folders and copied modules
#   make PYTHON=...   build against a specific interpreter (default: python3 in PATH)

PYTHON     ?= $(shell which python3)
BUILD_TYPE ?= RelWithDebInfo
OUT_DIR    := ../run/_ext

ALGOS := $(patsubst %/CMakeLists.txt,%,$(wildcard */CMakeLists.txt))

.PHONY: all clean $(ALGOS)

all: $(ALGOS)

$(ALGOS):
	cmake -S $@ -B $@/build \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DPYTHON_EXECUTABLE=$(PYTHON) \
		-DPython_EXECUTABLE=$(PYTHON)
	cmake --build $@/build -j
	mkdir -p $(OUT_DIR)
	cp $@/build/*.so $(OUT_DIR)/

clean:
	rm -rf $(addsuffix /build,$(ALGOS))
	rm -f $(OUT_DIR)/*.so
