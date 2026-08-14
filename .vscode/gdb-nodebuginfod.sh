#!/bin/bash
# Wrapper script to launch GDB with debuginfod completely disabled
unset DEBUGINFOD_URLS
unset DEBUGINFOD_TIMEOUT
unset DEBUGINFOD_MAXTIME
export DEBUGINFOD_URLS=""
exec /usr/bin/gdb -iex "set debuginfod enabled off" "$@"
