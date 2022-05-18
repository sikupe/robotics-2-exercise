from matplotlib import pyplot as plt
import numpy as np
import csv


def main():
    for exercise, integrator in enumerate(["Euler", "Runge-Kutta"], start=1):
        for i in range(2, 7):
            try:
                with open(f'output-{exercise}-{i}.csv') as f:
                    reader = csv.reader(f)
                    values = np.array([[float(v) for v in line] for line in reader])
                    plt.title(f"{integrator}: {10 ** -i} step size")
                    plt.plot(values[:, 0], values[:, 1])
                    plt.plot(values[:, 0], values[:, 2])
                    plt.show()
            except:
                print(f"Error for output-{exercise}-{i}.csv")

    pass


if __name__ == '__main__':
    main()
