from openag_brain.software_modules.topic_filter import EWMA
import numpy as np


def test_EWMA():
    """
    Tests the EWMA class to verify it calculates the expontentially weighted moving average for each new data point.
    :return: None
    """
    data_stream = [3, 2, 6, 4, 3, 4, 7, 4, 8, 5, 23, 4, 45, -90, 23, 45]
    EWMA_expected = [3, 2.8, 3.44, 3.552, 3.442, 3.553, 4.243, 4.194, 4.955, 4.964, 8.571, 7.657, 15.126, -5.899, -0.12]
    EWMA_func = EWMA(0.2)
    for sample, sample_avg_expected in zip(data_stream, EWMA_expected):
        EWMA_func(sample)
        assert round(EWMA_func.average, 3) == sample_avg_expected


if __name__ == '__main__':
    test_EWMA()
