import unittest

class TestHomingBehavior(unittest.TestCase):
    def test_homing_success(self):
        result = simulate_homing_process()
        self.assertTrue(result['success'])
        self.assertEqual(result['position'], 0)

    def test_homing_timeout(self):
        result = simulate_homing_process(timeout=True)
        self.assertFalse(result['success'])
        self.assertEqual(result['error'], 'Timeout during homing')

    def test_homing_obstacle_detection(self):
        result = simulate_homing_process(obstacle_detected=True)
        self.assertFalse(result['success'])
        self.assertEqual(result['error'], 'Obstacle detected during homing')

def simulate_homing_process(timeout=False, obstacle_detected=False):
    if timeout:
        return {'success': False, 'error': 'Timeout during homing'}
    if obstacle_detected:
        return {'success': False, 'error': 'Obstacle detected during homing'}
    return {'success': True, 'position': 0}

if __name__ == '__main__':
    unittest.main()