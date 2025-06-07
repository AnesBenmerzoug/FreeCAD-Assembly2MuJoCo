from typing import TypedDict

from src.constants import MUJOCO_JOINT_TYPE

__all__ = ["JointAttributes", "MaterialProperties", "AppearanceDict"]


class JointAttributes(TypedDict):
    type: MUJOCO_JOINT_TYPE


class MaterialProperties(TypedDict):
    CardName: str
    AuthorAndLicense: str
    Author: str
    License: str
    Name: str
    Description: str
    ReferenceSource: str
    SourceURL: str
    AmbientColor: str  # e.g. '(0.8984, 0.7305, 0.3906, 1.0)'
    DiffuseColor: str  # e.g. '(0.8984, 0.7305, 0.3906, 1.0)'
    EmissiveColor: str  # e.g. '(0.8984, 0.7305, 0.3906, 1.0)'
    Shininess: str  # e.g. '0.95'
    SpecularColor: str  # e.g. '(0.8984, 0.7305, 0.3906, 1.0)'
    Transparency: str  # e.g. '0'


class AppearanceDict(TypedDict):
    name: str
    rgba: str
    shininess: str
