#!/bin/bash

# Compile and link the code
g++ -std=c++17 -stdlib=libc++ -g "$1" -o "${1%.*}" -I /opt/homebrew/Cellar/yaml-cpp/0.8.0/include -L /opt/homebrew/Cellar/yaml-cpp/0.8.0/lib -lyaml-cpp
