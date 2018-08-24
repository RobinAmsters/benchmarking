import matplotlib as mpl
import numpy as np
import sys

if sys.version_info[0] < 3:
    import Tkinter as tk
else:
    import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def generate_test_set():
    normalVec = [2, -1, 1]
    xVec = 2 * (np.random.random([30]) - 0.5)
    yVec = 4 * (np.random.random([30]) - 0.5)
    zInput = np.random.random([30]) - 0.5
    zVec = normalVec[0] * xVec + normalVec[1] * yVec + normalVec[2] * zInput
    return np.column_stack((xVec, yVec, zVec))


# Create a canvas
def create_steady_point_selector(datapoints):

    def toggle_marking(points, name, selected, figure):
        def toggle():
            if name in selected:
                selected.remove(name)
                points._facecolor3d = np.array([0, 0, 1, 1])
                points._edgecolor3d = np.array([0, 0, 1, 1])
            else:
                points._facecolor3d = np.array([1, 0, 0, 1])
                points._edgecolor3d = np.array([1, 0, 0, 1])
                selected.append(name)
            figure.canvas.draw()

        return toggle


    window = tk.Tk()

    def close_window():
        window.destroy()
        window.quit()

    window.title("Select static (reference) markers")
    frame = tk.Frame(window)
    frame.pack(side=tk.LEFT)
    buttonframe = tk.Frame(window, width=20)
    buttonframe.pack(side=tk.RIGHT)
    closeframe = tk.Frame(buttonframe, height=10)
    closeframe.pack(side=tk.BOTTOM)

    fig = plt.figure()

    canvas = FigureCanvasTkAgg(fig, master=frame)

    canvas.get_tk_widget().pack(side=tk.LEFT)

    ax = fig.gca(projection='3d')
    ax.axis('equal')

    selected = list()

    for name, dataset in datapoints.iteritems():
        points = ax.scatter(dataset[:, 0], dataset[:, 1], dataset[:, 2])
        tk.Button(buttonframe, text=name, width=8, command=toggle_marking(points, name, selected, fig)).pack(side=tk.BOTTOM)

    tk.Button(closeframe, text="Done", width=10, command=close_window).pack(side=tk.BOTTOM, fill=tk.X,pady=20)
    window.protocol("WM_DELETE_WINDOW", close_window)
    # Let Tk take over
    window.mainloop()

    return selected


if __name__ == '__main__':
    dataset = generate_test_set()
    dataset2 = generate_test_set()
    datapoints = {'first': dataset, 'second': dataset2}
    print(create_steady_point_selector(datapoints))
