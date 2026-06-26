import threading
import queue

# class MonThread(threading.Thread):
#     def __init__(self, nom):
#         super().__init__()
#         self.nom = nom
#         self.compteur = 0

#     def run(self):
#         print(self.nom, self.compteur)

#     def mod(self, thread):
#         self.compteur += 1
#         thread.mod()
        
# if __name__ == "__main__":
#     # Création des objets
#     a = MonThread("A")
#     b = MonThread("B")

#     # Lancement des threads via une boucle
#     a.start()
#     b.start()

#     for t in [a, b]:
#         t.compteur = 1
#         print(t.nom, t.compteur)

#     # Attend que tous les threads terminent
#     a.join()
#     b.join()


class A(threading.Thread):
    def __init__(self):
        super().__init__() 
        self.b = B()
        self.b.start()
        self.c = C()
        self.c.start()
        self.b.link = self.c

    def run(self):
        for obj in [self.b,self.c]:
            obj.add_1(1)

class B(threading.Thread):
    def __init__(self):
        super().__init__()
        self.stack = []
        self.link = None

    def run(self):
        self.add_1
        
    def add_1(self, item):
        self.stack.append(item)
        new_item = item*2
        self.link.add_2(new_item)
    
    def get_stack(self):
        return self.stack

class C(threading.Thread):
    def __init__(self):
        super().__init__()
        self.stack = []

    def add_1(self,item):
        self.stack.append(item)
    
    def add_2(self,item):
        new_item = self.stack[0] + item
        self.stack.append(new_item)

    
    def get_stack(self):
        return self.stack

if __name__ == "__main__":
    a = A()
    a.start()
    a.join()
    print(a.b.get_stack())
    print(a.c.get_stack())


