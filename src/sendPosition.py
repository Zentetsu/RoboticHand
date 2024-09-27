from IRONbark import Module
import ast

if __name__ == "__main__":

    Target_Module = Module(file="./data/Target_Module.json")

    t = None

    while t != "q":
        print("Example: \n[\"wrist\", [-100, 100, 250, 0, 0, 45]]\n[\"wrist\", [0, 250, 205, 0, 0, 0]]")
        # ["wrist", [0, 250, 175, 0, 0, 0]]
        # ["wrist", [0, 250, 150, 0, 0, 0]]
        # ["wrist", [0, 200, 150, 0, 0, 0]]
        # ["wrist", [0, 200, 100, 0, 0, 0]]
        # ["index", [-50.0, 170.0, 15.0, 180, 90, 0]]
        # ["wrist", [0, 200, 75, 0, 0, 0]]
        # ["wrist", [0, 200, 60, 0, 0, 0]]
        # ["index", [-70.0, 170.0, 15.0, 180, 90, 0]]
        # ["index", [-75.0, 170.0, 0.0, 180, 90, 0]]
        # ["index", [-78.0, 170.0, 0.0, 180, 90, 0]]

        t = input()

        try:
            n_pos = ast.literal_eval(t)
            Target_Module["target"][n_pos[0]] = n_pos[1]
        except:
            print("ERROR")
            continue


    Target_Module.stopModule()
