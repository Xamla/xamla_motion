#!/bin/bash

sphinx-apidoc -f -o source/ ../src/xamla_motion/
make html
