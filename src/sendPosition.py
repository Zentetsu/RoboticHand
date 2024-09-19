from SharedMemory import SharedMemory
import ast

if __name__ == "__main__":
    pos = {
        "wrist": [-100, 100, 250, 0, 0, 45],
        "thumb": [-50.0, 120.0, 25.0, 170, 25, -75],
        "index": [-80.0, 170.0, 15.0, 180, 90, 0],
    }
    C = SharedMemory(name="SOFA_pos", value=pos, client=True, size=1024)

    t = None

    while t != "q":
        t = input()
        # ["wrist", [-100, 100, 250, 0, 0, 45]]
        # ["wrist", [0, 250, 205, 0, 0, 0]]

        print(t)

        try:
            n_pos = ast.literal_eval(t)
            pos[n_pos[0]] = n_pos[1]

            C.setValue(pos)
        except:
            print("ERROR")
            continue


    C.close()
