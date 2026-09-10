import unittest

from radar_core import CalibrationModel
from synchronized_acquisition import ClockEstimate, SynchronizedAcquisition


class Endpoint:
    def __init__(self):
        self.messages = []

    def write_line(self, line, on_sent=None):
        self.messages.append(line)
        return True


class TimeoutRecoveryTests(unittest.TestCase):
    def make(self):
        endpoints = [Endpoint(), Endpoint()]
        events = []
        a = SynchronizedAcquisition(*endpoints, CalibrationModel(p0=700, k=100),
                                    {'sync_max_age_s': 60}, lambda *e: events.append(e))
        a.start(0)
        a.state = 'running'
        a.clocks = {s: ClockEstimate(0, 0.001, 0, 0) for s in a.endpoints}
        for endpoint in endpoints:
            endpoint.messages.clear()
        return a, endpoints, events

    def test_timeout_keeps_rotating_and_retrying_until_ten_seconds(self):
        a, endpoints, events = self.make()
        a.poll(2.1)
        self.assertEqual(a.state, 'running')
        a.poll(12.09)
        self.assertEqual(a.state, 'running')
        self.assertNotIn('OFF', endpoints[1].messages)
        self.assertTrue(any(m.startswith('SYNC ') for m in endpoints[1].messages))
        self.assertIn('PING', endpoints[1].messages)
        a.poll(12.1)
        self.assertEqual(a.state, 'stopped')
        self.assertIn('OFF', endpoints[1].messages)
        self.assertTrue(any(k == 'sync_error' and '10 秒' in v for k, v, _ in events))

    def test_real_data_recovers_and_next_dropout_gets_new_window(self):
        a, endpoints, events = self.make()
        a.builder.trigger(0, 0, 1)
        a.builder.sample(0.2, 0, 800, 1)
        a.poll(2.1)
        self.assertEqual(a.state, 'running')
        self.assertIsNone(a.builder.anchor)
        a.feed('measurement', f'PIX {a.session} 1 3000000 3000000 800\n'.encode(), 3)
        a.feed('rotation', f'TRIG {a.session} 2 3000000\n'.encode(), 3)
        a.poll(3)
        self.assertIsNone(a.recovery_started_at)
        a.poll(5.1)
        self.assertEqual(a.recovery_started_at, 5.1)
        a.poll(15.09)
        self.assertEqual(a.state, 'running')
        a.poll(15.1)
        self.assertEqual(a.state, 'stopped')

    def test_stale_clock_retries_but_cannot_publish_range(self):
        a, endpoints, events = self.make()
        a.config['sync_max_age_s'] = 8
        a.last_arrival = {s: 9 for s in a.endpoints}
        a.poll(9)
        self.assertEqual(a.state, 'running')
        a.feed('measurement', f'PIX {a.session} 1 9500000 9500000 800\n'.encode(), 9.5)
        a.poll(9.5)
        self.assertFalse(any(k == 'sync_range' for k, _, _ in events))
        a.poll(19)
        self.assertEqual(a.state, 'stopped')

    def test_manual_stop_during_recovery_is_immediate(self):
        a, endpoints, _ = self.make()
        a.poll(2.1)
        a.stop()
        self.assertEqual(a.state, 'stopped')
        self.assertIn('OFF', endpoints[1].messages)
        self.assertIsNone(a.recovery_started_at)

    def test_matching_sync_responses_restore_clock_without_restarting_motor(self):
        a, endpoints, events = self.make()
        a.config['sync_max_age_s'] = 8
        a.last_arrival = {s: 9 for s in a.endpoints}
        a.poll(9)
        self.assertIsNotNone(a.recovery_started_at)
        for source in a.endpoints:
            token = a.outstanding[source][0]
            a.sent_times[(source, token)] = 9
            a.feed(source, f'SYNC {token} 9001000 9001000\n'.encode(), 9.002)
        a.poll(9.002)
        self.assertEqual(a.state, 'running')
        self.assertIsNone(a.recovery_started_at)
        self.assertTrue(all(c.observed_at == 9.002 for c in a.clocks.values()))
        self.assertNotIn('OFF', endpoints[1].messages)
        self.assertFalse(any(m.startswith('ROT ') for m in endpoints[1].messages))


if __name__ == '__main__':
    unittest.main()
