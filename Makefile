# General targets

.PHONY: all
all: format

# Formatting

.PHONY: format
format:
	find . -iname '*.h' -o -iname '*.c' -o -iname '*.cpp' | xargs clang-format -i

# Tests

.PHONY: build
build:
	cmake -S test -B build
	cmake --build build

.PHONY: test
test: build
	ctest --test-dir build --output-on-failure
