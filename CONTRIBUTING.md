# Contributing Guide

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you shall be license under the LGPL-2.1 license.

### Setup

- To start, create a conda environment with the necessary dependencies and activate it:

  ```shell
  conda create --prefix ./.conda_env -f environment.yml -y
  conda activate ./.conda_env
  ```

  We use conda because it allows use to install FreeCAD, Python and dependencies.

- Ideally this would be enough, but if your editor complains that it can't find the workbench code in the environment
  (happened to me with vscode and pylance), then manually create a symlink of the code inside the site-packages directory of the conda environment:

  ```shell
  ln -s $(pwd)/freecad/assembly2mujoco "$(python -c 'import site; print(site.getsitepackages()[0])')/freecad/assembly2mujoco"
  ```

- Install pre-commit hooks:

  ```shell
  pre-commit install
  ```
