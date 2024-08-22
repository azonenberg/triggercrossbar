#!/bin/sh
orbcat --channel "0,%c" --server localhost:6000 %c | socat - TCP4-LISTEN:6001,reuseaddr,fork
