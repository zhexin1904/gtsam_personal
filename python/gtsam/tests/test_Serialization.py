"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Serialization and deep copy tests.

Author: Varun Agrawal
"""
import unittest

import numpy as np
from gtsam.symbol_shorthand import B, V, X
from gtsam.utils.test_case import GtsamTestCase

import gtsam


class TestDeepCopy(GtsamTestCase):
    """Tests for deep copy of various GTSAM objects."""

    def test_PreintegratedImuMeasurements(self):
        """
        Test the deep copy of `PreintegratedImuMeasurements` by performing a deepcopy.
        """
        params = gtsam.PreintegrationParams(np.asarray([0, 0, -9.81]))
        pim = gtsam.PreintegratedImuMeasurements(params)

        self.assertDeepCopyEquality(pim)

    def test_ImuFactor(self):
        """
        Test the deep copy of `ImuFactor` by performing a deepcopy.
        """
        params = gtsam.PreintegrationParams(np.asarray([0, 0, -9.81]))
        pim = gtsam.PreintegratedImuMeasurements(params)
        imu_factor = gtsam.ImuFactor(X(0), V(0), X(1), V(1), B(0), pim)

        self.assertDeepCopyEquality(imu_factor)

    def test_PreintegratedCombinedMeasurements(self):
        """
        Test the deep copy of `PreintegratedCombinedMeasurements` by performing a deepcopy.
        """
        params = gtsam.PreintegrationCombinedParams(np.asarray([0, 0, -9.81]))
        pim = gtsam.PreintegratedCombinedMeasurements(params)

        self.assertDeepCopyEquality(pim)


if __name__ == "__main__":
    unittest.main()
