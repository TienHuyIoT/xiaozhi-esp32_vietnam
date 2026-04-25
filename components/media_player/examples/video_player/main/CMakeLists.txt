set(srcs "video_player.c")

idf_component_register(SRCS ${srcs}
                    REQUIRES ${COMPONENT_PRIV_REQUIRES}
                    INCLUDE_DIRS ".")
