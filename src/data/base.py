from abc import ABC, abstractmethod


class BaseInfo(ABC):
    @abstractmethod
    def to_json(self):
        pass

    @abstractmethod
    def from_json(self, js):
        pass
