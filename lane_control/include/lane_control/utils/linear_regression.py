from scipy.stats import linregress


def fit_line(array):
    slope, intercept, r_value, p_value, std_err = linregress(array[:, 1], array[:, 2])
    return slope, intercept
