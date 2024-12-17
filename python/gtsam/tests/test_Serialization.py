"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

KalmanFilter unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest
from copy import deepcopy

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam


class TestSerialization(GtsamTestCase):
    """Tests for serialization of various GTSAM objects."""

    def test_PreintegratedImuMeasurements(self):
        """
        Test the serialization of `PreintegratedImuMeasurements` by performing a deepcopy.
        """
        params = gtsam.PreintegrationParams(np.asarray([0, 0, -9.81]))
        pim = gtsam.PreintegratedImuMeasurements(params)

        # If serialization failed, then this will throw an error
        pim2 = deepcopy(pim)
        self.assertEqual(pim, pim2)

    def test_PreintegratedCombinedMeasurements(self):
        """
        Test the serialization of `PreintegratedCombinedMeasurements` by performing a deepcopy.
        """
        params = gtsam.PreintegrationCombinedParams(np.asarray([0, 0, -9.81]))
        pim = gtsam.PreintegratedCombinedMeasurements(params)

        # If serialization failed, then this will throw an error
        pim2 = deepcopy(pim)
        self.assertEqual(pim, pim2)


if __name__ == "__main__":
    unittest.main()
