# kalmanif

## A small collection of Kalman Filters on Lie groups

[![GHA][badge-ci-img]][badge-ci]
<!-- [![codecov][badge-cov-img]][badge-cov] -->
![GitHub][badge-license]
<!-- [![Documentation][badge-doc-img]][kalmanif-doc] -->

## Package Summary

`kalmanif` is a Kalman filter(s) on Lie groups library for state-estimation
targeted at robotics applications.
It is developed as a header-only C++17 library based on [manif][manif-repo].

At the moment, it implements:

- Extended Kalman Filter (EKF)
- Square Root Extended Kalman Filter (SEKF)
- Invariant Extended Kalman Filter (IEKF)
- Unscented Kalman Filter on manifolds (UKFM)
- Rauch-Tung-Striebel Smoother*

(*the RTS Smoother is compatible with all filters - ERTS / SERTS / IERTS/ URTS-M)

Together with a few system and measurement models mostly for demo purpose.
Other filters/models can and will be added, contributions are welcome.

`kalmanif` started has a rework of the excellent [kalman][kalman-repo] library by [Markus Herb][mherb] to turn [the filtering-based examples in manif][manif-examples] into reusable code.
The main difference from the [kalman][kalman-repo] library is its integration with the [manif][manif-repo] library to handle the Lie theory aspects.
There are also numerous implementation details that differ and which can't be summarized here.

`kalmanif` is distributed under the same permissive license as it's inspirational model.

> :heavy_exclamation_mark: kalmanif is very much a work in progress. As such, do not expect it to work out of the box, nor support your application. Do expect its API to change. Headache possible. :heavy_exclamation_mark:

### Details

- Maintainer status: maintained
- Maintainer: Jeremie Deray
- Authors: Jeremie Deray
- License: [MIT](LICENSE)
- Bug / feature tracker: [github.com/artivis/kalmanif/issues][kalmanif-issue]
- Source: [github.com/artivis/kalmanif.git][kalmanif-repo] (branch: devel)

## Quick Start

Checkout the installation guide [over here](CONTRIBUTING.md#development-environment).

### Note

Both the `IEKF` and `UKFM` filters are implemented in their **'right invariant'** flavor.
However they are able to handle both 'right' *and* 'left' measurements.

<!-- ## Documentation -->

## Tutorials and application demos

We provide some self-contained and self-explained executables implementing some real problems.
Their source code is located in `kalmanif/examples/`.
These demos are:

- [`demo_se2.cpp`](examples/demo_se2.cpp): 2D robot localization based on fixed landmarks using SE2 as robot poses.
This implements the example V.A in [SOLA-18-Lie][jsola18].
- [`demo_se3.cpp`](examples/demo_se3.cpp): 3D robot localization based on fixed landmarks using SE3 as robot poses.
This re-implements the example above but in 3D.
- [`demo_se_2_3.cpp`](examples/demo_se_2_3.cpp): 3D robot localization and linear velocity estimation based on strap-down IMU model and fixed beacons.

Check out the documentation to see how to [build them][demo-build] and what are [their options][demo-run].

## References

`kalmanif` is based on several publications, some of which are referenced [here](docs/publications.md).

## Contributing

Want to contribute? Great! Check out our [contribution guidelines](CONTRIBUTING.md).

[//]: # (URLs)

[kalman-repo]: https://github.com/mherb/kalman
[mherb]: https://github.com/mherb

[jsola18]: http://arxiv.org/abs/1812.01537
[barrau15]: https://arxiv.org/pdf/1410.1465.pdf
[deray20]: https://joss.theoj.org/papers/10.21105/joss.01371
[brossard20]: https://arxiv.org/pdf/2002.00878.pdf
[Barrau17]: https://arxiv.org/pdf/1410.1465.pdf

[eigen]: http://eigen.tuxfamily.org
[ceres]: http://ceres-solver.org/
[ceres-jet]: http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets
[crtp]: https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern

[kalmanif-repo]: https://github.com/artivis/kalmanif.git
[kalmanif-issue]: https://github.com/artivis/kalmanif/issues
[kalmanif-doc]: https://codedocs.xyz/artivis/kalmanif
[demo-build]: CONTRIBUTING.md#development-environment
[demo-run]: docs/demo.md

[manif-repo]: https://github.com/artivis/manif
[manif-examples]: https://github.com/artivis/manif/tree/devel/examples

[pybind11]: https://pybind11.readthedocs.io/en/stable/index.html

[git-workflow]: http://nvie.com/posts/a-successful-git-branching-model/

[badge-ci]: https://github.com/artivis/kalmanif/workflows/build-and-test/badge.svg?branch=devel
[badge-ci-img]: https://github.com/artivis/kalmanif/workflows/build-and-test/badge.svg?branch=devel
[badge-ci-win]: https://ci.appveyor.com/project/artivis/kalmanif
[badge-ci-win-img]: https://ci.appveyor.com/api/projects/status/l0q7b0shhonvejrd?svg=true
[badge-doc-img]: https://codedocs.xyz/artivis/kalmanif.svg
[badge-cov]: https://codecov.io/gh/artivis/kalmanif
[badge-cov-img]: https://codecov.io/gh/artivis/kalmanif/branch/devel/graph/badge.svg
[badge-license]: https://img.shields.io/github/license/mashape/apistatus.svg
