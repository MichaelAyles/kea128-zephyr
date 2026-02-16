board_runner_args(jlink "--device=SKEAZ128xxx4" "--reset-after-load")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
