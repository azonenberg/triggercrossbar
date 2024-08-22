#!/bin/sh
export OBJDUMP=arm-none-eabi-objdump
orbtop --server localhost:6010 --elf-file debug-build/bin/main --routines 20
