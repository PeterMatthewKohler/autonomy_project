import contextlib
import io
import unittest

import analyze_imu_odom as analysis

M_PI = analysis.math.pi

def imu_at(*times):
    return [analysis.ImuSample(t, 0.0, 0.0, 0.0, 0.0) for t in times]


class StreamMatchingTests(unittest.TestCase):
    def test_latest_causal_observation_is_offered_once(self):
        odom = [analysis.OdomSample(t, 0.0) for t in (5, 15, 18, 40)]
        matches = analysis.match_heading_observations(imu_at(0, 10, 20, 30), odom)
        self.assertEqual(
            [m.observation.timestamp_ns if m.observation else None for m in matches],
            [None, 5, 18, None],
        )
        self.assertAlmostEqual(matches[2].observation_age_s, 2e-9)

    def test_repeated_timestamp_is_not_a_new_observation(self):
        odom = [analysis.OdomSample(t, 0.0) for t in (5, 5, 20)]
        matches = analysis.match_heading_observations(imu_at(10, 10, 9, 25), odom)
        self.assertEqual([m.observation is not None for m in matches], [True, False, False, True])

    def test_decreasing_odometry_timestamps_are_reported(self):
        with self.assertRaisesRegex(ValueError, 'timestamps decrease'):
            analysis.match_heading_observations(imu_at(20), [
                analysis.OdomSample(10, 0.0), analysis.OdomSample(5, 0.0)
            ])

    def test_age_policy_is_not_silently_selected(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            analysis.print_corrected_comparison(
                analysis.BagSamples(imu=imu_at(0, 10)), 0.05, None
            )
        printed = output.getvalue()
        self.assertIn("Observation-age limit: None", printed)
        self.assertIn('IMU header period', printed)
        self.assertNotIn('Corrected final heading:', printed)


class YourWeek9EquationTests(unittest.TestCase):
    def test_nonzero_propagation(self):
        step_ns = 400_000_000  # 0.4 seconds
        samples = [
            analysis.ImuSample(i * step_ns, 0.5, 0.0, 0.0, 0.0)
            for i in range(6)
        ]
        heading, intervals = analysis.propagate_imu_heading(samples, 0.0)
        self.assertEqual(intervals, 5)
        self.assertAlmostEqual(heading, 1.0)

    def test_nonpositive_time(self):
        step_ns = 400_000_000  # 0.4 seconds
        samples = [
            analysis.ImuSample(0 * step_ns, 0.5, 0.0, 0.0, 0.0),
            analysis.ImuSample(0 * step_ns, 0.5, 0.0, 0.0, 0.0),
            analysis.ImuSample(-1 * step_ns, 0.5, 0.0, 0.0, 0.0),
            analysis.ImuSample(1 * step_ns, 0.5, 0.0, 0.0, 0.0),
        ]
        heading, intervals = analysis.propagate_imu_heading(samples, 0.0)
        self.assertEqual(intervals, 1)
        self.assertAlmostEqual(heading, 0.4)

    def test_wrapped_innovation(self):
        predicted = M_PI - 0.1
        observed = -M_PI + 0.1
        gain = 0.75
        corrected_heading, wrapped_innovation = analysis.correct_heading(
            predicted,
            observed,
            gain
        )
        self.assertAlmostEqual(corrected_heading, -M_PI + 0.05)
        self.assertAlmostEqual(wrapped_innovation, 0.2)

    def test_gain_endpoints(self):
        heading, _ = analysis.correct_heading(
            0.5,
            0.6,
            0.0
        )
        self.assertEqual(heading, 0.5)
        heading, _ = analysis.correct_heading(
            0.5,
            0.6,
            1.0
        )
        self.assertEqual(heading, 0.6)


if __name__ == '__main__':
    unittest.main()
