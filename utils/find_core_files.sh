#!/bin/bash

# This script will find all files named core
# Not all files named core should be deleted!
# (Some folks use the name for "core" vocabularies)
sudo find / -name core -type f -prune -print
