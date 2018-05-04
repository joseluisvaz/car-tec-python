import numpy as np
from scipy.stats import linregress


def fit_line(array, img_shape):

    # Stack one line
    array = np.concatenate((np.ones((array.shape[0], 1)), array), axis=1)
    slope, intercept, r_value, p_value, std_err = linregress(array[:, 2], array[:, 1])
    return slope, intercept
