"""Checks against analytic mechanics and integration consistency, not JSBSim validation."""
from dataclasses import replace
from importlib.util import find_spec
import unittest
import numpy as np
from c172sim import Aircraft, Controls, level_trim, simulate_segments
from c172sim.frames import body_velocity, body_to_ned, wind_to_body
from c172sim.dynamics import rhs, rhs_from_loads

class FrameChecks(unittest.TestCase):
    def test_heading_ninety_points_body_x_east(self):
        np.testing.assert_allclose(body_to_ned(0,0,np.pi/2)@np.array([1,0,0]),
                                   [0,1,0],atol=1e-14)
    def test_rotation_matrices_are_proper(self):
        for matrix in (body_to_ned(0.2,-0.3,0.7),wind_to_body(0.08,-0.05)):
            np.testing.assert_allclose(matrix.T@matrix,np.eye(3),atol=1e-14)
            self.assertAlmostEqual(np.linalg.det(matrix),1.0,places=13)
    def test_air_data_velocity_and_wind_axis_agree(self):
        vector=body_velocity(47,0.08,-0.03)
        self.assertAlmostEqual(np.linalg.norm(vector),47)
        self.assertAlmostEqual(np.arctan2(vector[2],vector[0]),0.08)
        self.assertAlmostEqual(np.arcsin(vector[1]/47),-0.03)
        np.testing.assert_allclose(wind_to_body(0.08,-0.03)[:,0],vector/47,atol=1e-14)

class MechanicsChecks(unittest.TestCase):
    @unittest.skipUnless(find_spec("scipy") is not None, "SciPy is required for the analytic integration check")
    def test_gravity_only_matches_analytic_free_motion(self):
        from scipy.integrate import solve_ivp
        x=np.array([50,0.03,0.01,0,0,0,0.1,0.2,0.3,0,0,-1000.0])
        Vned=body_to_ned(*x[6:9])@body_velocity(*x[:3])
        final=2.0
        result=solve_ivp(lambda t,s:rhs_from_loads(s,np.zeros(3),np.zeros(3),10,np.diag([2,3,4])),
                         (0,final),x,method="DOP853",rtol=1e-11,atol=1e-12,max_step=0.05)
        end=result.y[:,-1]
        np.testing.assert_allclose(end[9:12],x[9:12]+Vned*final+0.5*np.array([0,0,9.80665])*final**2,
                                   atol=1e-8,rtol=0)
        np.testing.assert_allclose(body_to_ned(*end[6:9])@body_velocity(*end[:3]),
                                   Vned+np.array([0,0,9.80665])*final,atol=1e-8,rtol=0)
    @unittest.skipUnless(find_spec("scipy") is not None, "SciPy is required for the analytic integration check")
    def test_torque_free_energy_and_inertial_momentum(self):
        from scipy.integrate import solve_ivp
        J=np.array([[2.0,0.1,0.0],[0.1,3.0,0.2],[0.0,0.2,4.0]])
        x=np.array([50,0.02,0.01,0.1,0.2,0.3,0.1,0.15,-0.2,0,0,0],dtype=float)
        result=solve_ivp(lambda t,s:rhs_from_loads(s,np.zeros(3),np.zeros(3),10,J,gravity=0),
                         (0,3),x,method="DOP853",rtol=1e-11,atol=1e-12,max_step=0.03)
        w0=x[3:6]
        energy0=0.5*w0@J@w0
        H0=body_to_ned(*x[6:9])@J@w0
        V0=body_to_ned(*x[6:9])@body_velocity(*x[:3])
        for state in result.y.T:
            self.assertAlmostEqual(0.5*state[3:6]@J@state[3:6],energy0,places=9)
            np.testing.assert_allclose(body_to_ned(*state[6:9])@J@state[3:6],H0,atol=1e-9,rtol=0)
            np.testing.assert_allclose(body_to_ned(*state[6:9])@body_velocity(*state[:3]),V0,atol=1e-8,rtol=0)

@unittest.skipUnless(find_spec("scipy") is not None, "SciPy is required to recompute trim and DOP853 trajectories")
class AircraftChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.aircraft=Aircraft()
        cls.trim=level_trim(cls.aircraft)

    def test_loaded_mass_and_inertia_are_physical(self):
        a=self.aircraft
        self.assertAlmostEqual(a.mass,804.67286438,places=8)
        np.testing.assert_allclose(a.inertia,a.inertia.T,atol=1e-12)
        moments=np.linalg.eigvalsh(a.inertia)
        self.assertTrue(np.all(moments>0))
        self.assertLess(moments[-1],sum(moments[:2]))

    def test_trim_balances_forces_moments_and_altitude_rate(self):
        t=self.trim
        self.assertLess(np.max(np.abs(t.force_residual_N)),1e-5)
        self.assertLess(np.max(np.abs(t.moment_residual_Nm)),1e-5)
        derivative=rhs(0,t.state,self.aircraft,t.controls)
        self.assertLess(np.max(np.abs(derivative[:9])),1e-7)
        self.assertLess(abs(derivative[11]),1e-8)

    def test_trim_stays_steady_for_thirty_seconds(self):
        t=self.trim
        sim=simulate_segments(self.aircraft,t.state,[(30,t.controls)],max_step=0.1)
        np.testing.assert_allclose(sim.states[:,:9],np.tile(t.state[:9],(len(sim.time),1)),atol=1e-6,rtol=0)
        self.assertLess(np.max(np.abs(sim.states[:,11]-t.state[11])),1e-5)

    def test_pulse_converges_when_solver_precision_is_increased(self):
        t=self.trim
        pulse=replace(t.controls,elevator_rad=t.controls.elevator_rad-np.deg2rad(0.5))
        segments=[(2,t.controls),(1,pulse),(12,t.controls)]
        normal=simulate_segments(self.aircraft,t.state,segments,max_step=0.1)
        fine=simulate_segments(self.aircraft,t.state,segments,rtol=1e-11,atol=1e-12,max_step=0.04)
        delta=np.abs(normal.states-fine.states)
        self.assertLess(delta[:,0].max(),1e-4)
        self.assertLess(delta[:,1:9].max(),1e-6)
        self.assertLess(delta[:,9:12].max(),1e-3)
        self.assertGreater(normal.states[np.argmin(np.abs(normal.time-2.1)),4],0)
        self.assertTrue(np.all(np.diff(normal.time)>0))
        self.assertAlmostEqual(normal.time[-1],15.0)

    def test_zero_speed_start_is_rejected(self):
        state=self.trim.state.copy()
        state[0]=0
        with self.assertRaises(ValueError):
            simulate_segments(self.aircraft,state,[(1,self.trim.controls)])

if __name__=="__main__":
    unittest.main()
