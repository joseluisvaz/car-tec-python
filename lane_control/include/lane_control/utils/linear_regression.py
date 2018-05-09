from scipy.stats import linregress
from collections import namedtuple

Line = namedtuple("Line", ["slope", "intercept"])


def fit_line(array):
    """
    Receives array of activated pixels value and returns a fit to the linear regression of this data
    :param array:
    :return: slope and intercept to the array
    """

    if array.size != 0:
        slope, intercept, r_value, p_value, std_err = linregress(array[:, 0], array[:, 1])
        return Line(slope, intercept)

    return None
