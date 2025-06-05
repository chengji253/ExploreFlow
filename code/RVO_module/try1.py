import numpy as np
import matplotlib.pyplot as plt


def plot_unique_numbers(ndarray):
    fig, ax = plt.subplots(figsize=(14, 8))  # Adjust the figure size as necessary

    # Flatten the array and get unique values and their positions
    unique_values, indices = np.unique(ndarray, return_index=True)
    rows, cols = np.unravel_index(indices, ndarray.shape)

    # Plot each unique number at its position
    for (row, col, value) in zip(rows, cols, unique_values):
        ax.text(col, row, str(value), ha='center', va='center', fontsize=8)

    # Adjust the limits and invert the y-axis
    ax.set_xlim(-1, ndarray.shape[1])
    ax.set_ylim(-1, ndarray.shape[0])
    ax.invert_yaxis()

    # Hide the axes
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)

    plt.show()


# Example usage:
ndarray = np.random.randint(0, 100, (400, 700))  # Generating a 400x700 array with random integers
plot_unique_numbers(ndarray)
