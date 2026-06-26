import threading
import queue
import time
import random

q_1 = queue.Queue()
q_2 = queue.Queue()
go1 = threading.Event()
go2 = threading.Event()

def producteur():
    for i in range(10):
        item = f"message {i}"
        print(f"🟢 Produit: {item}")
        q_1.put(item)
        q_2.put(item)  # mettre deux fois pour deux consommateurs
        go1.set()  # signaler au consommateur 1
        go2.set()  # signaler au consommateur 2
        # go1.clear()  # réinitialiser le signal
        # go2.clear()  # réinitialiser le signal

    # mettre un signal de fin pour CHAQUE consommateur
    q_1.put(None)
    q_2.put(None)

def consommateur_1(nom):
    while True:
        go1.wait()  # attendre le signal
        item = q_1.get()
        go1.clear()  # réinitialiser le signal
        if item is None:  # signal de fin
            break
        print(f"🔵 {nom} consomme: {item}")
        
    print(f"❌ {nom} terminé")

def consommateur_2(nom):
    while True:
        go2.wait()  # attendre le signal
        item = q_2.get()
        go2.clear()  # réinitialiser le signal
        if item is None:  # signal de fin
            break
        print(f"🟠 {nom} consomme: {item}")
    print(f"❌ {nom} terminé")

t_prod = threading.Thread(target=producteur)
t_cons1 = threading.Thread(target=consommateur_1, args=("Cons1",))
t_cons2 = threading.Thread(target=consommateur_2, args=("Cons2",))

t_prod.start()
t_cons1.start()
t_cons2.start()

t_prod.join()
t_cons1.join()
t_cons2.join()

print("✅ Fin du programme")