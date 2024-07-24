# Credits
This Project is inspired by the [point-in-polygon](https://github.com/xuzebin/point-in-polygon.git) repository. However, the polygon.h file was completely rewritten. Only the example.cpp and example2.cpp are only made compatible.

So, the given License for this repository only applies to the file in include/polygon.hpp!

# What is this for?
Simple and fast implementation of the [Ray casting algorithm](https://en.wikipedia.org/wiki/Point_in_polygon) using a mix of C-Style array and C++-Style implementations.

Can be used for example for geo fences.

## Runtime
O(N), where N = #sides of polygon(s).

## Requirements

Needs a working cpp compiler and cmake to build the examples.

## Build

Use

```
mkdir build
cd build
cmake ..
```

to configure the Project to be build.

Then use

```
make
```

to build the two example programs.

## Run

Run each with
```
./Example
```

for a single polygon

or

```
./Example2
```

for multiple polygons.

Use "d" to finish the polygon ("done"). "x" removes all polygons. Click will add a new point. Have fun!
