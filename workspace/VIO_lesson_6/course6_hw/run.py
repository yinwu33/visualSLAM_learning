import build.triangulate as tri
import matplotlib.pyplot as plt
import numpy as np

DEFAULT_STDVAR = 0.1
DEFAULT_START_FRAME = 3


def test():
    ratio = tri.run(0, 3)
    print(f"----\n{ratio}")


def run_compare_by_start_frame():
    startFrame_list = [i for i in range(98)]
    print(startFrame_list)
    ratio_list = [tri.run(DEFAULT_STDVAR, startFrame) for startFrame in startFrame_list]
    
    plt.ylabel("ratio: sigma[3]/sigma[2]")
    plt.xlabel("start Frame")
    plt.plot(startFrame_list, ratio_list)
    plt.show()

def run_compare_by_noise():
    stdvar_list = np.arange(0, 10, 0.01)
    ratio_list = [tri.run(stdvar, DEFAULT_START_FRAME) for stdvar in stdvar_list]

    plt.ylabel("ratio: sigma[3]/sigma[2]")
    plt.xlabel("standard variation")
    plt.plot(stdvar_list, ratio_list)
    plt.show()
    


if __name__ == "__main__":
    # test()
    # run_compare_by_noise()
    run_compare_by_start_frame()
