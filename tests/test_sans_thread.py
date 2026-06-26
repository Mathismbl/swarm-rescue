class A():
    def __init__(self):
        self.b = B()
        self.c = C()
        self.b.link = self.c

    def start(self):
        for obj in [self.b,self.c]:
            obj.add_1(1)

class B():
    def __init__(self):
        self.stack = []
        self.link = None

    def add_1(self, item):
        self.stack.append(item)
        new_item = item*2
        self.link.add_2(new_item)
    
    def get_stack(self):
        return self.stack

class C():
    def __init__(self):
        self.stack = []

    def add_1(self,item):
        self.stack.append(item)
    
    def add_2(self,item):
        if self.stack:
            new_item = self.stack[0] + item
            self.stack.append(new_item)
        else:
            print("⚠️ Impossible de traiter add_2 : aucune donnée de A")
    
    def get_stack(self):
        return self.stack

if __name__ == "__main__":
    a = A()
    a.start()
    print(a.b.get_stack())
    print(a.c.get_stack())