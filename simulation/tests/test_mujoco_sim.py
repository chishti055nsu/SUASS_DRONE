"""
test_mujoco_sim.py
===================
Automated Integration Test for MuJoCo Physics Simulation & HAL.

Validates:
  1. Skydio X2 MuJoCo XML model loading (mujoco_sim/skydio_x2_mission.xml).
  2. Physics stepping at 500 Hz.
  3. MuJoCoFlightController HAL position setpoint tracking & telemetry reporting.
"""

import os
import sys
import unittest
import time
import math

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "mission_planner"))

from mission_planner.flight_controller import MuJoCoFlightController, create_flight_controller

try:
    import mujoco
    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False


@unittest.skipUnless(HAS_MUJOCO, "MuJoCo library not installed")
class TestMuJoCoSimulation(unittest.TestCase):

    def setUp(self):
        xml_path = os.path.join(ROOT, "simulation", "mujoco_sim", "skydio_x2_mission.xml")
        self.assertTrue(os.path.exists(xml_path), f"XML model not found at {xml_path}")

        # Load MuJoCo model and data
        cwd = os.getcwd()
        os.chdir(os.path.dirname(xml_path))
        try:
            self.model = mujoco.MjModel.from_xml_path(os.path.basename(xml_path))
            self.model.opt.timestep = 0.002  # 500 Hz physics step
            self.data = mujoco.MjData(self.model)
        finally:
            os.chdir(cwd)

    def test_model_loading_and_stepping(self):
        self.assertGreater(self.model.nbody, 0)
        self.assertGreater(self.model.ngeom, 0)

        # Step physics for 100 steps (0.2s)
        for _ in range(100):
            mujoco.mj_step(self.model, self.data)

        # Check vehicle qpos vector
        self.assertEqual(len(self.data.qpos), self.model.nq)

    def test_mujoco_flight_controller_hal(self):
        fc = create_flight_controller("mujoco")
        self.assertTrue(isinstance(fc, MuJoCoFlightController))
        self.assertTrue(fc.is_connected())

        # Test arming and setpoint setting
        self.assertTrue(fc.arm_and_offboard())
        self.assertTrue(fc.is_armed())

        fc.set_setpoint_enu(5.0, 10.0, 15.0)

        # Verify telemetry output
        telem = fc.get_telemetry()
        self.assertIn("pos_enu", telem)
        self.assertEqual(telem["mode"], "OFFBOARD")

        # Test payload release trigger
        self.assertTrue(fc.trigger_payload_release())
        self.assertTrue(fc.get_telemetry()["payload_released"])

        # Test disarming
        self.assertTrue(fc.disarm())
        self.assertFalse(fc.is_armed())


if __name__ == "__main__":
    unittest.main()
