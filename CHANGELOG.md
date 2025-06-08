# Changelog

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
