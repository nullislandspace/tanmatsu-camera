idf_component_register(
    SRCS
        "src/bitstream.c"
        "src/huffman.c"
        "src/l3bitstream.c"
        "src/l3loop.c"
        "src/l3mdct.c"
        "src/l3subband.c"
        "src/layer3.c"
        "src/reservoir.c"
        "src/tables.c"
    INCLUDE_DIRS
        "src"
)

# Shine upstream is -Wall -Werror clean on gcc -O2 but trips a few of
# ESP-IDF's stricter warnings (unused parameters, implicit conversions
# between float/double constants, etc.). Relax the ones that fire
# inside the vendored code so we don't have to patch upstream.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-parameter
    -Wno-unused-but-set-variable
    -Wno-unused-variable
    -Wno-implicit-fallthrough
    -Wno-sign-compare
    -Wno-maybe-uninitialized
    -Wno-unused-function
    -Wno-missing-field-initializers
    -Wno-error
)
