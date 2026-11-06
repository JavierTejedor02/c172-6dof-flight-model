"""Run trim, equilibrium, and an elevator-pulse experiment for the reduced C172."""
from __future__ import annotations
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import platform
import os
import tempfile
import numpy as np
import scipy
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "c172sim-matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from c172sim import Aircraft, __version__, level_trim, simulate_segments
from c172sim.dynamics import STATE_NAMES

def save_csv(filename, simulation):
    values = np.column_stack((simulation.time,simulation.states,
                              -simulation.states[:,11],simulation.controls))
    header = ("time_s,"+",".join(STATE_NAMES)+
              ",altitude_m,elevator_rad,aileron_rad,rudder_rad,thrust_N")
    np.savetxt(filename,values,delimiter=",",header=header,comments="",fmt="%.12g")

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration",type=float,default=30.0)
    parser.add_argument("--pulse-deg",type=float,default=-0.5,
                        help="Elevator increment between t=2 and t=3 seconds")
    parser.add_argument("--method", choices=["DOP853", "EULER", "RK4"], default="DOP853")
    parser.add_argument("--step-size", type=float, default=0.05,
                        help="Fixed step for Euler/RK4; maximum adaptive step for DOP853 [s]")
    parser.add_argument("--output-dir",type=Path,default=Path(__file__).resolve().parent/"results")
    args=parser.parse_args()
    if (args.duration <= 3 or not np.isfinite(args.duration) or not np.isfinite(args.pulse_deg)
            or not np.isfinite(args.step_size) or args.step_size <= 0):
        parser.error("Duration must exceed 3 s, step size must be positive, and arguments must be finite")
    aircraft=Aircraft()
    trim=level_trim(aircraft)
    u=trim.controls
    pulse=replace(u,elevator_rad=u.elevator_rad+np.deg2rad(args.pulse_deg))
    solver_options = dict(method=args.method, step_size=args.step_size, max_step=args.step_size)
    equilibrium=simulate_segments(aircraft,trim.state,[(args.duration,u)], **solver_options)
    response=simulate_segments(aircraft,trim.state,[(2.0,u),(1.0,pulse),(args.duration-3.0,u)], **solver_options)
    args.output_dir.mkdir(parents=True,exist_ok=True)
    save_csv(args.output_dir/"equilibrium.csv",equilibrium)
    save_csv(args.output_dir/"elevator_pulse.csv",response)
    report={
        "model":"Educational reduced civil C172P; explicit force/moment and rigid-body model",
        "software_version":__version__,
        "source_commit":aircraft.data["source_commit"],
        "mass_kg":aircraft.mass,"inertia_body_kg_m2":aircraft.inertia.tolist(),
        "density_kg_m3":aircraft.rho,
        "trim_state":dict(zip(STATE_NAMES,trim.state.tolist())),
        "trim_controls":asdict(u),
        "trim_force_residual_N":trim.force_residual_N.tolist(),
        "trim_moment_residual_Nm":trim.moment_residual_Nm.tolist(),
        "trim_scaled_residual_norm":trim.scaled_residual_norm,
        "trim_evaluations":trim.evaluations,
        "equilibrium_max_abs_state_change_first_9":np.max(np.abs(equilibrium.states[:,:9]-trim.state[:9]),axis=0).tolist(),
        "equilibrium_max_altitude_change_m":float(np.max(np.abs(equilibrium.states[:,11]-trim.state[11]))),
        "experiment":{"duration_s":args.duration,"elevator_increment_deg":args.pulse_deg,"pulse_interval_s":[2,3]},
        "integration":{"method":args.method,"step_limit_s":args.step_size,
                       "step_kind":"adaptive_maximum" if args.method == "DOP853" else "fixed",
                       "equilibrium_nfev":equilibrium.nfev,"response_nfev":response.nfev},
        "environment":{"python":platform.python_version(),"numpy":np.__version__,
                       "scipy":scipy.__version__,"matplotlib":matplotlib.__version__},
        "scope_limitations":aircraft.data["omitted_source_terms"],
        "reference_comparison":"Not yet compared quantitatively with a JSBSim execution or measured flight data.",
    }
    (args.output_dir/"trim_and_run.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False})
    fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
    specifications=[
        ("True airspeed [m/s]",lambda x:x[:,0]),
        ("Angle of attack [deg]",lambda x:np.rad2deg(x[:,1])),
        ("Pitch angle [deg]",lambda x:np.rad2deg(x[:,7])),
        ("Pitch rate [deg/s]",lambda x:np.rad2deg(x[:,4])),
        ("Altitude [m]",lambda x:-x[:,11]),
        ("Bank angle [deg]",lambda x:np.rad2deg(x[:,6])),
    ]
    for ax,(label,extract) in zip(axes.flat,specifications):
        ax.plot(response.time,extract(response.states),color="#146b8b",lw=1.8,label="Elevator pulse")
        ax.plot(equilibrium.time,extract(equilibrium.states),color="#737e89",ls="--",lw=1.1,label="Equilibrium")
        ax.axvspan(2,3,color="#e9a23b",alpha=0.20)
        ax.set_ylabel(label)
        ax.grid(alpha=0.18)
        ax.ticklabel_format(axis="y",style="plain",useOffset=False)
    for ax in axes[-1]:ax.set_xlabel("Time [s]")
    axes[0,0].legend(loc="best",frameon=False)
    fig.suptitle(f"Reduced C172P | {args.method} | open-loop elevator response",fontsize=17,x=0.08,ha="left")
    fig.text(0.08,0.935,
             f"50 m/s, 1000 m initial altitude | elevator increment {args.pulse_deg:+.2f} deg at 2-3 s | constant density",
             color="#4b5563",fontsize=10)
    fig.tight_layout(rect=(0,0,1,0.92))
    fig.savefig(args.output_dir/"response.png",dpi=160)
    plt.close(fig)
    summary={"output_dir":str(args.output_dir),"alpha_trim_deg":float(np.rad2deg(trim.state[1])),
             "elevator_trim_deg":float(np.rad2deg(u.elevator_rad)),"thrust_trim_N":u.thrust_N,
             "equilibrium_altitude_drift_m":report["equilibrium_max_altitude_change_m"]}
    print(json.dumps(summary,indent=2))
if __name__=="__main__":
    main()
