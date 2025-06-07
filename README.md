# FreeCAD - Assembly2MuJoCo

[![GitHub Release][release-badge]][releases]
[![Changelog][cc-badge]][cc]
[![GitHub Issues][issues-badge]][issues]
[![Github CI Workflow][ci-workflow-badge]][ci-workflow]
[![License][license-badge]][license-file]


![Assembly2MuJoCo Icon](resources/icons/assembly2mujoco-icon.svg)

This is a [FreeCAD](https://www.freecad.org/) workbench to export an Assembly made with the builtin [Assembly Workbench](https://wiki.freecad.org/Assembly_Workbench) for simulation in [MuJoCo](https://mujoco.org/).

![Export of universal joint assembly to MuJoCo](examples/universal_joint/output.gif)

*Export of universal joint assembly to MuJoCo*

If you find any bugs while using the macro, please feel free to open an [issue](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/issues).

## Changelog

A log of all notables changes made to the macro can be found [here.](CHANGELOG.md)

## Prerequisites

- Python >= 3.10.0
- FreeCAD >= 1.0.0

## Usage

1. Switch to Assembly2MuJoCo workbench
2. Select the assembly you want to export.
3. Execute the export command which will open a task panel
   for configuring parts of the export.
4. Click Ok and if no error was raised, a dialog box will pop-up informing you of the location of the export.

## Simulating the export with MuJoCo

- [Install MuJoCo](https://mujoco.readthedocs.io/en/latest/programming/#getting-started)

- Navigate to the directory of the exported assembly and simulate in MuJoCo using:

  - The `simulate` executable:

  ```shell
  simulate <Path to MJCF (XML) file>
  ```

  - The MuJoCO Python package:

  ```
  python -m mujoco.viewer --mjcf=<Path to MJCF (XML) file>
  ```

  In both cases, this will open a new window with a MuJoCo
  visualization session containing the exported model.

## Examples

Please refer to the [examples](examples/) directory for some examples showcasing the use of this macro.

## Troubleshooting

### Collision Issues

- MuJoCo by default excludes collisions between parent and child bodies
  However, this exclusion is not applied if the parent is a static body
  i.e. the world body, or a descendant of the world body without any degrees of freedom

- A common issue in MuJoCo typically happens when using STL files because MuJoCo handles
  collision geometry differently from visual geometry by default.

## Installation

### Addon Manager

- Open the Addon manager by going to Tools -> Addon manager
- Type `Assembly2MuJoCo` in the search bar.
- Select the workbench and click on install.

### Manual Installation

- Find FreeCAD's Mod directory which is located inside the FreeCAD's data directory. The latter can be found by executing the following in the python console:

  ```python
  App.getUserAppDataDir()
  ```

- Either:

  - Clone the repository into the Mod directory.
  - Download the repository as a zip file from Github and extract its contents into the Mod directory.

## Roadmap

Here are some of the planned developments for this Macro:

- [ ] Support, if possible, all assembly joint types:
  - [X] Grounded
  - [X] Fixed
  - [X] Revolute
  - [ ] Cylindrical
  - [ ] Slider
  - [ ] Ball
  - [ ] Distance
  - [ ] Parallel
  - [ ] Perpendicular
  - [ ] Angle
  - [ ] Rack and Pinion
  - [ ] Screw
  - [ ] Gear
  - [ ] Bolt
- [ ] Add, if possible, all examples from the Assembly Workbench [wiki page](https://wiki.freecad.org/Assembly_Workbench):
  - [ ] Crank and Slider
  - [X] Universal Joint
  - [ ] Vise
  - [ ] Shock Absorber
- [ ] Extract properties from part material (e.g. density).
- [ ] Handle nested assemblies.

## Development

### Setup

- Create and activate a virtual environment:

  ```shell
  python -m venv .venv
  source .venv/bin/activate
  ```

- Install development dependencies:

  ```shelll
  pip install .[dev]
  ```

- Install pre-commit hooks:

  ```shell
  pre-commit install
  ```

## Icon

The project's icon combines the FreeCAD Assembly Workbench icon with the MuJoCo logo.

**FreeCAD Component**: Assembly Workbench icon from FreeCAD Artwork, licensed under LGPL-2.1+

**MuJoCo Component**: MuJoCo logo owned by DeepMind, used to indicate compatibility with MuJoCo.

This project is independent and not affiliated with FreeCAD or MuJoCo/DeepMind. Logo elements are used solely to indicate integration between these platforms.

The project icon, including the arrangement and connecting elements, is licensed under [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/). Any derivative works must maintain proper attribution to both FreeCAD and MuJoCo as outlined above.


## License

This code is licensed under the [LGPL-2.1][license-file] license.

[releases]: https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/releases
[release-badge]: https://img.shields.io/github/v/release/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo?label=version

[license-file]: ./LICENSE
[license-badge]: https://img.shields.io/github/license/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo

[issues]: https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/issues
[issues-badge]: https://img.shields.io/github/issues/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo

[ci-workflow-badge]: https://img.shields.io/github/actions/workflow/status/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/main.yml?label=CI
[ci-workflow]: https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/actions/workflows/main.yml

[cc]: ./CHANGELOG.md
[cc-badge]: https://common-changelog.org/badge.svg
