# Contributing Guide

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you shall be license under the LGPL-2.1 license.

### Setup

- Create a virtual environment:

  ```shell
  python -m venv .venv
  activate .venv/bin/activate
  ```

  This virtual environment will only be used to install pre-commit

- Install the dev dependencies:

  ```shell
  pip install .[dev]
  ```

- Install pre-commit hooks:

  ```shell
  pre-commit install
  ```

- After that, to set up FreeCAD for development and testing run the main script inside the scripts directory:

  ```shell
  bash scripts/main.sh
  ```

  This will do the following:

  - Download a FreeCAD AppImage (by default, version 1.1.1) from Github into the `freecad_versions/` directory.
  - Extract the content of the AppImage into a directory inside the `freecad_versions/`.
  - Create a symlink for that directory with the name `FreeCAD_default`. This is useful for vscode suggestions and completions.
  - Install test dependencies for the python interpreter bundled with the downloaded FreeCAD version.
  - Run tests using that interpreter.

  There is caching, so running the script again will be faster.

  If you want to run tests for another FreeCAD version, for example 1.0.2 use:

  ```shell
  bash scripts/main.sh 1.0.2
  ```
