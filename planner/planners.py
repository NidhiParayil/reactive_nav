from abc import ABC, abstractmethod


class Planner(ABC):
    @abstractmethod
    def run_planner(self):
        pass