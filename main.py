# Lab 4 - runs A* and the heuristic checks
import platform

from src import lab4_astar_audit


def main():
    print("AI Lab 4: Informed Search, Heuristics & A* | Mohammed Emad ALaini")
    print("Python %s, standard library only" % platform.python_version())
    print()
    lab4_astar_audit.main()


if __name__ == "__main__":
    main()
