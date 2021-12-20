set(srcs
    "foo.c")




idf_component_register(SRCS "foo.c"
                    INCLUDE_DIRS "."
                    REQUIRES vfs)
