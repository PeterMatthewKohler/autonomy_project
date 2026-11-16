import contextlib
import io
import unittest

import analyze_imu_odom as analysis


class Week8ScaffoldTests(unittest.TestCase):
    def test_shared_interval_and_nearest_endpoints(self):
        samples = analysis.BagSamples(
            imu=[analysis.ImuSample(t, 0.0, 0.0, 0.0, 0.0) for t in (0, 10, 20, 30)],
            odom=[analysis.OdomSample(t, 0.0) for t in (5, 15, 25, 35)],
        )
        interval = analysis.select_heading_interval(samples)
        self.assertEqual((interval.start_ns, interval.end_ns), (5, 30))
        self.assertEqual([s.timestamp_ns for s in interval.imu], [10, 20, 30])
        self.assertEqual(interval.odom_start.timestamp_ns, 5)
        self.assertEqual(interval.odom_end.timestamp_ns, 25)

    def test_rejects_missing_disjoint_and_insufficient_data(self):
        for imu_times, odom_times in (([], [0, 10]), ([0, 10], []), ([0, 10], [20, 30]), ([0, 10], [5, 6])):
            samples = analysis.BagSamples(
                imu=[analysis.ImuSample(t, 0.0, 0.0, 0.0, 0.0) for t in imu_times],
                odom=[analysis.OdomSample(t, 0.0) for t in odom_times],
            )
            with self.subTest(imu=imu_times, odom=odom_times):
                with self.assertRaises(ValueError):
                    analysis.select_heading_interval(samples)

    def test_completed_comparison_prints_metrics(self):
        samples = analysis.BagSamples(
            imu=[analysis.ImuSample(t, 0.0, 0.0, 0.0, 0.0) for t in (0, 1_000_000_000)],
            odom=[analysis.OdomSample(t, 0.0) for t in (0, 1_000_000_000)],
        )
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            analysis.print_heading_comparison(samples)
        self.assertIn('Comparison duration: 1.000000 s', output.getvalue())
        self.assertIn('Starting odometry yaw: 0.000000 rad', output.getvalue())
        self.assertIn('Final odometry yaw: 0.000000 rad', output.getvalue())
        self.assertIn('Raw integrated IMU final heading: 0.000000 rad', output.getvalue())
        self.assertIn('IMU intervals integrated: 1', output.getvalue())
        self.assertIn('Wrapped final heading error: 0.000000 rad (0.000000 deg)', output.getvalue())
        self.assertNotIn('TODO (you)', output.getvalue())


if __name__ == '__main__':
    unittest.main()
