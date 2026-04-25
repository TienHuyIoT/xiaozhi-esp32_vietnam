set(srcs "main.c" "player_cli.c" "folder_reader.c" "sys_state.c")

idf_component_register(SRCS ${srcs}
                    REQUIRES ${COMPONENT_PRIV_REQUIRES}
                    INCLUDE_DIRS ".")
