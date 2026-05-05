# Changelog

## [0.4.0](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/releases/tag/v0.4.0) - 2026-05-05

### Added

- Handle conversion of cylindrical and ball joints ([#14](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/14))
- Add invisible [site](https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-site) element to each exported body ([#17](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/17))
- Allow users to disable adding sites to exported bodies ([#18](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/18))

### Changed

- Set `align="true"` property of MuJoCo [freejoint](https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-freejoint) ([#15](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/15))


### Fixed

- Fix depth first traversal of graph ([#15](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/15))
- Fix error with FreeCAD version 1.1 due to a breaking change in the Assembly workbench's `getMovingPart` function ([#16](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/16))


## [0.3.0](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/releases/tag/v0.3.0) - 2025-11-18

### Added

- Allow configuring the weights assigned to the different joints in order to handle kinematic loops. ([#9](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/9))
- Allow exporting assemblies with disconnected parts.
- Handle conversion of revolute joints in kinematic loops.

### Changed

- Change handling of kinematic loops by adding dummy bodies and weld constraints. ([#9](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/9))
- Use collapsible sections for the different configuration options in the task panel. ([#9](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/9))
- Use part and joint labels instead of names in exported MJCF and mesh files.

### Fixed

- Fix workbench icon path in package metadata ([#11](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/11))
- Fix exporting part appearance.

## [0.2.0](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/releases/tag/v0.2.0) - 2025-06-08

### Added

- Option to select export format for assembly parts and allow exporting as OBJ files and set it as default ([#8](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/8))

  - In MuJoCo:
    - STL is great for basic convex shapes used purely in simulation.
    - OBJ is ideal when you need:
      - Visual detail (materials, normals),
      - Multiple meshes per asset, or
      - To decompose concave meshes (e.g., using CoACD) for valid MuJoCo collisions.

### Changed

- **Breaking:** convert the macro to a workbench ([#7](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/pull/7))

  This makes the code more manageable (split across multiple files instead of a single large file).

## [0.1.0](https://github.com/AnesBenmerzoug/FreeCAD-Assembly2MuJoCo/releases/tag/v0.1.0) - 2025-05-22

This is the very _first release_ of the AssemblyExportToMuJoCo FreeCAD Macro.

### Features

- Support for Grounded, Fixed and Revolute joints.
- Dockable panel in FreeCAD's task panel for configuration.

[0.1.0]: https://github.com/AnesBenmerzoug/FreeCAD-Macro-AssemblyExportToMuJoCo/releases/tag/v0.1.0
