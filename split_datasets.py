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
def create_steady_point_selector(dataSet):

    def toggle_marking(ax, dataSet, figure, container):
        def toggle(splitPoint):
            splitPoint = int(splitPoint)
            container[0] = splitPoint
            ax.clear()
            ax.scatter(dataSet[:splitPoint, 0], dataSet[:splitPoint, 1], dataSet[:splitPoint, 2], c='r')
            ax.scatter(dataSet[splitPoint:, 0], dataSet[splitPoint:, 1], dataSet[splitPoint:, 2], c='b')
            figure.canvas.draw()

        return toggle


    window = tk.Tk()


    def close_window():
        window.destroy()
        window.quit()



    window.title("Select static (reference) markers")
    frame = tk.Frame(window)
    frame.pack(side=tk.LEFT)
    closeframe = tk.Frame(frame, height=10)
    closeframe.pack(side=tk.BOTTOM)

    fig = plt.figure()

    canvas = FigureCanvasTkAgg(fig, master=frame)

    canvas.get_tk_widget().pack(side=tk.LEFT)

    ax = fig.gca(projection='3d')
    ax.axis('equal')

    selected = list()

    points = ax.scatter(dataSet[:, 0], dataSet[:, 1], dataSet[:, 2])

    container = [0]
    w = tk.Scale(closeframe, from_=0, to=len(dataSet), orient=tk.HORIZONTAL, command=toggle_marking(ax, dataSet, fig, container))
    w.pack()

    tk.Button(closeframe, text="Done", width=10, command=close_window).pack(side=tk.BOTTOM, fill=tk.X,pady=20)
    window.protocol("WM_DELETE_WINDOW", close_window)
    # Let Tk take over
    window.mainloop()

    return container[0]


if __name__ == '__main__':
    dataset = generate_test_set()
    print(create_steady_point_selector(dataset))
