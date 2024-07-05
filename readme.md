# DWGSI

## How to get the submodules

```bash
git submodule update --init --recursive
```

## Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

Currently tested on Windows using toolchain of VisualStudio 2019 and 2022. Should work on UNIX with gcc-likes. With win+VS, could emit several encoding warnings (4819).

Building produces a dwgSim.exe executable.

## Basic Use (current)

```bash
path/to/exe/dwgSim.exe InputDwg.dwg -o OutputJson.json
```

Then you can check the output json visually with

```bash
demo/drawDwgSimJson.py OutputJson.json
```

## Project Structure (current)

A single class in `dwgsimReader` now handles the extraction of data from the libreDWG dwg object.

The CollectModelSpaceEntities() method is the primary translation process.
