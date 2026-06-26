from solutions.vortex_1 import MyDroneVortex


class MyDroneEval(MyDroneVortex):
    def __init__(self, **kwargs):
        super().__init__(signature="Module manager", **kwargs)