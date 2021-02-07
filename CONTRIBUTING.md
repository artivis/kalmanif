# Contributing

- [Contributing](#contributing)
  - [General guidelines](#general-guidelines)
  - [Development environment](#development-environment)
    - [Dependencies](#dependencies)
      - [Eigen3](#eigen3)
      - [manif](#manif)
      - [gnuplot (optional)](#gnuplot-optional)
    - [Fetching kalmanif source code](#fetching-kalmanif-source-code)
    - [Building kalmanif](#building-kalmanif)
    - [Running examples/tests](#running-examplestests)
    - [Generate the documentation](#generate-the-documentation)
  - [Use kalmanif in your project](#use-kalmanif-in-your-project)

## General guidelines

`kalmanif` is developed according to Vincent Driessen's [Gitflow Workflow][git-workflow].
This means,

- the `master` branch is for releases only.
- development is done on feature branches.
- finished features are integrated via PullRequests into the branch `devel`.

For a PullRequest to get merged into `devel`, it must pass

- Review by one of the maintainers.
  - Are the changes introduced in scope of `kalmanif`?
  - Is the documentation updated?
  - Are enough reasonable tests added?
  - Will these changes break the API?
  - Do the new changes follow the current style of naming?
- Compile / Test / Run on all target environments.

Note: The test suite detailed below is run in CI for many targets environments including,

- Ubuntu 18.04/20.04 g++/clang++
- MacOS 10.15
- Visual Studio 15

## Development environment

We will detail here how to set up a development environment.
It is recommended to use containers if you do not want to install the dependencies on your host.
You may refer to [this blog post](lxd-post) to set up a LXD container.

### Dependencies

First, let's install all the dependencies,

#### Eigen3

Linux (Ubuntu and similar),

```bash
apt install libeigen3-dev
```

OS X,

```bash
brew install eigen
```

Windows,

```bash
vcpkg install eigen3:x64-windows
```

#### manif

```bash
git clone https://github.com/artivis/manif.git
cd manif && mkdir build && cd build
cmake ..
cmake --build . --config Release --target install
```

#### gnuplot (optional)

Linux (Ubuntu and similar),

```bash
apt install gnuplot
```

OS X,

```bash
brew install gnuplot
```

Windows (Chocolatey),

```bash
choco install Gnuplot
```

### Fetching kalmanif source code

Now let us clone the `kalmanif` repo,

```bash
git clone https://github.com/artivis/kalmanif.git
cd kalmanif
```

### Building kalmanif

We can now build `kalmanif` examples and tests,

```terminal
mkdir build && cd build
cmake -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON ..
cmake --build . --config Release
```

Optionally, to build the examples with plots,
assuming you have installed `gnuplot`,
add the following flag,

```bash
cmake -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON -DPLOT_EXAMPLES=ON ..
cmake --build . --config Release
```

### Running examples/tests

Examples are located in `build/examples`.
To try one out, e.g.

```bash
cd examples
./demo_se2
```

See the run results [on the dedicated page](docs/demo.md).

> :heavy_exclamation_mark: Pro tip, if you only want to run the example,
> or to deploy your application,
> don't forget to compile in `Release` together with the appropriate optimization flags,
> e.g. `-DBUILD_TESTING=OFF -DBUILD_EXAMPLES=ON -DPLOT_EXAMPLES=ON -DCMAKE_BUILD_TYPE=Release -DNDEBUG=1 -DCMAKE_CXX_FLAGS="-march=native -O3 -mtune=native -mavx2 -mfma"` to get the most out of it.
> :heavy_exclamation_mark:

To run the C++ tests, execute the following in the `build` directory,

```terminal
ctest --output-on-failure
```

### Generate the documentation

To generate the Doxygen documentation,

```bash
cd kalmanif/docs
doxygen Doxyfile
```

## Use kalmanif in your project

In your project `CMakeLists.txt` :

```cmake
project(foo)

# Find the kalmanif library
find_package(kalmanif REQUIRED)

add_executable(${PROJECT_NAME} src/foo.cpp)
# 'Link' kalmanif interface library to the target
target_link_libraries(${PROJECT_NAME} INTERFACE kalmanif::kalmanif)
# Set c++17 flag
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
```

[//]: # (URLs)

[git-workflow]: http://nvie.com/posts/a-successful-git-branching-model
[lxd-post]: https://artivis.github.io/post/2020/lxc
