# python

This module provides Python utilities and bindings infrastructure for AliceVision.

## Overview

The `python` module contains Python helper scripts used within the AliceVision pipeline, along with the infrastructure for parallel processing.

## parallelization.py

This script provides Python-level parallelization helpers. It is used by pipeline scripts to manage distributed or multi-process execution of AliceVision nodes.

## Python Bindings

The Python bindings for the main AliceVision C++ modules are generated using SWIG (`.i` interface files) and are located alongside the corresponding C++ modules. The `python` module provides shared infrastructure for these bindings.
