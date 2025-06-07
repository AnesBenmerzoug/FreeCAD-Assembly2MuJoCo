from abc import ABC, abstractmethod

__all__ = ["BaseCommand"]

commands: dict[str, "BaseCommand"] = {}


def get_all_commands() -> dict[str, "BaseCommand"]:
    return commands


class BaseCommand(ABC):
    def __init_subclass__(cls, **kwargs):
        global commands
        super().__init_subclass__(**kwargs)
        commands[cls.__name__] = cls

    @abstractmethod
    def GetResources(self) -> dict[str, str]:
        pass

    @abstractmethod
    def Activated(self) -> None:
        pass

    @abstractmethod
    def IsActive(self) -> bool:
        pass
