import sys
import pkgutil
import importlib.metadata

# 1. Affiche le site-packages actuellement utilisé
print("📂 Dossier site-packages utilisé :")
for path in sys.path:
    if "site-packages" in path:
        print("  ", path)

# 2. Lister les dossiers présents
print("\n📦 Paquets trouvés dans site-packages (dossiers réels) :")
packages = sorted({pkg.name for pkg in pkgutil.iter_modules()})
print(packages)

# 3. Lister les paquets détectés par pip (via metadata)
print("\n📋 Paquets détectés par pip (via metadata) :")
try:
    pip_packages = sorted([dist.metadata["Name"] for dist in importlib.metadata.distributions()])
    print(pip_packages)
except Exception as e:
    print("Impossible de récupérer la liste via metadata :", e)

# 4. Comparer les deux listes
extra_in_fs = set(packages) - set([p.lower().replace("-", "_") for p in pip_packages])
if extra_in_fs:
    print("\n Paquets présents sur disque mais pas connus de pip :")
    print(extra_in_fs)
else:
    print('\n Pas de paquets "orphelins" dans site-packages.')
    