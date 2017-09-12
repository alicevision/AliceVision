Project: CXSparse
URL: https://github.com/TheFrenchLeaf/CXSparse
License: CXSparse, Copyright (c) 2006-2012, Timothy A. Davis. Distributed under the GNU LGPL license.
Upstream version: 3.1.1

Local modifications:

  * Add CMake based build

#[CXSparse](http://www.cise.ufl.edu/research/sparse/CXSparse/) Multi-platform compilation

CXSparse version 3.1.1.

##Compilation
- Unzip CXSparse
- mkdir CXSparseBuild
- cd CXSparseBuild
- Run cmake . ../CXSparse
- make

##Test
- cd Demo
- ./cs_demo3.exe < ../../CXSparse/Matrix/bcsstk16

 > residual must be very small (1.e-023)

#Modification:
- Disabled the complex support

#Authors:
- TheFrenchLeaf (cmake based build)
