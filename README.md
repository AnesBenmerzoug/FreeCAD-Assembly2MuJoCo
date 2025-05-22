# FreeCAD Macro - Assembly Export to MuJoCo

<div align="center">
<img src="AssemblyExportToMuJoCo.svg" width="200px"/>
</div>

This repository contains a [FreeCAD](https://www.freecad.org/) macro to export an Assembly made with the builtin [Assembly Workbench](https://wiki.freecad.org/Assembly_Workbench) to [MuJoCo](https://mujoco.org/).

## Prerequisites

- Python >= 3.10
- FreeCAD >= 1.0

## Getting Started

- [Install the Macro](https://wiki.freecad.org/How_to_install_macros) manually in FreeCAD by copying
  the [AssemblyExportToMuJoCo.FCMacro](./AssemblyExportToMuJoCo.FCMacro) and [AssemblyExportToMuJoCo.svg](./AssemblyExportToMuJoCo.svg) files into FreeCAD's Macro directory.

- (Optional) Add the Macro to the toolbar for easier execution.

- Open the assembly you want to export into MuJoCo.

- Select it and execute the Macro.

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

Please refer to the [examples](examples/) directory for some examples showcasing this macro.


## Troubleshooting

### Collision Issues

- MuJoCo by default excludes collisions between parent and child bodies
  However, this exclusion is not applied if the parent is a static body
  i.e. the world body, or a descendant of the world body without any degrees of freedom

- A common issue in MuJoCo typically happens when using STL files because MuJoCo handles
  collision geometry differently from visual geometry by default.

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

To make life easier, we would like to keep the Macro here in the repository and at the same time use it in FreeCAD without
having to manually copy it there or change FreeCAD's Macro directory.

For that, an easy solution is to create a symbolic link to the Macro (and related files) in FreeCAD's Macro directory:

```shell
ln -s $(pwd)/AssemblyExportToMuJoCo.FCMacro <Path to FreeCAD Macro Directory>
ln -s $(pwd)/AssemblyExportToMuJoCo.svg <Path to FreeCAD Macro Directory>
```

## Logo

The project's logo combines the FreeCAD Assembly Workbench icon with the MuJoCo logo.

**FreeCAD Component**: Assembly Workbench icon from FreeCAD Artwork, licensed under LGPL-2.1+

**MuJoCo Component**: MuJoCo logo owned by DeepMind, used to indicate compatibility with MuJoCo.

This project is independent and not affiliated with FreeCAD or MuJoCo/DeepMind. Logo elements are used solely to indicate integration between these platforms.

The project logo, including the arrangement and connecting elements, is licensed under [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/). Any derivative works must maintain proper attribution to both FreeCAD and MuJoCo as outlined above.


## License

This code is licensed under the [LGPL-2.1](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html) license.
