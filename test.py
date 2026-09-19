import technicmovehub, pkgutil, inspect

print("Package location:", technicmovehub.__path__)

for finder, name, ispkg in pkgutil.iter_modules(technicmovehub.__path__):
    print("Submodule found:", name)
    mod = __import__(f"technicmovehub.{name}", fromlist=[name])
    for member_name, member_obj in inspect.getmembers(mod, inspect.isclass):
        if not member_name.startswith("_"):
            print(f"  Class: {member_name}")
            