#!/usr/bin/env python3
"""
Python reimplementation of the MATLAB script `sim_traj_mpc_curv_off.m`,
now wired to load your controller/vehicle parameters from a simple YAML-like
config file (exactly like the block you sent).

It uses your existing acados-based model & OCP setup in:
  - acados_settings.py
  - bicycle_model.py

Key features:
- Loads track CSV: [s, x, y, kappa, (nl), (nr)] and allows a constant width override.
- Builds the OCP with `acados_settings(...)` and runs a closed-loop MPC.
- Supports input/actuation delay via `steps_delay` + `t_delay` (rounded to whole steps).
- Soft constraints with Zl/Zu/zl/zu as in your params.
- CLI flag `--config` to read the params file; if omitted, sensible defaults are used.

Run (example):
  python sim_traj_mpc_curv_off_v2.py \
      --csv ../../traj/centerline_test_map.csv \
      --config params.yaml \
      --sim-time 80

Where `params.yaml` is the text you posted.
"""
from __future__ import annotations
from pathlib import Path
from collections import deque
from dataclasses import dataclass
import argparse
import signal, sys
import time
import numpy as np
import matplotlib.pyplot as plt
import casadi as cs
from utils.indicies import StateIndex
from aux_func import handler_plots

# ---- dependências do teu projeto (já existentes) ----
from acados_settings import acados_settings, get_parameters
from load_params import load_params_file, load_track_csv, make_configs
from aux_func import frenet_to_global
import select
from mpc_plots import (
    plot_states,  # pós-sim
    plot_track_and_boundaries,  # figura estática extra (opcional)
    init_sim_plot,
    update_sim_plot,  # animação/refresh
)


@dataclass
class SimResult:
    x_hist: np.ndarray
    u_hist: np.ndarray
    status: np.ndarray
    solve_ms: np.ndarray
    dt: float
    freq: float
    mpc_self: MPCSim


class MPCSim:
    """
    Controla fim-a-fim: carregar pista, criar OCP/solver acados, warm start,
    simulação em malha-fechada, e (opcional) visualização.
    """

    # -------------- construção / setup --------------
    def __init__(
        self,
        track_csv: Path,
        cfg_overrides: list[dict] | None = None,
        track_width_override: float | None = None,
        no_plot: bool = False,
        x_init: np.ndarray = None,
    ) -> None:
        self.no_plot = no_plot

        # 1) Pista e limites
        tr = load_track_csv(track_csv, track_width_override=track_width_override)
        self.tr = tr
        self.s_ref = tr["s"].copy()
        self.kappa_ref = tr["kappa"].copy()
        self.d_left = tr["nl"].copy()
        self.d_right = tr["nr"].copy()

        # 2) Configs (stmpc, car, tire)
        self.stmpc, self.car, self.tire = make_configs(cfg_overrides or [])

        # 3) OCP / solver / modelo
        # returns: constraint, model, acados_solver, params(ns)
        self.constraint, self.model, self.solver, self.params_ns = acados_settings(
            self.s_ref,
            self.kappa_ref,
            self.d_left,
            self.d_right,
            self.stmpc,
            self.car,
            self.tire,
        )

        # 4) Parâmetros online (constantes aqui), para todos os estágios
        self.p_vec = get_parameters(self.stmpc).astype(float)
        
        y_ref = np.zeros(self.model.n_x)
        y_ref[2] = self.stmpc.v_x_ref  # v_x_ref
        for stage in range(self.stmpc.N):
            self.solver.set(stage, "p", self.p_vec)
            self.solver.set(stage, "yref", y_ref)  
            
        self.solver.set(self.stmpc.N, "p", self.p_vec)
        self.solver.set(self.stmpc.N, "yref", y_ref[:-2]) # remove control refs from yref at terminal stage
        
       
        

        # 5) Estado inicial (ajusta conforme o teu modelo)
        self.x = x_init
        
        self.Fz_f = []
        self.Fz_r = []
        self.Fx_f = []
        self.Fx_r = []
        self.Fy_f = []
        self.Fy_r = []
        
        self.alpha_f = []
        self.alpha_r = []

        self.xN =[]
        self.uN =[]

        # warm start inicial com estado atual em Frenet (usa (s, n, mu) da self.x)
        self.apply_warm_start(self.x)

        # 6) Plot inicial (opcional)
        self.fig = None
        if not self.no_plot:
            self.fig = init_sim_plot(self.tr, self.d_left, self.d_right, self.x)

        # 7) Delay discreto (em passos): steps_delay + arredondamento de t_delay
        extra_steps = int(
            round(getattr(self.stmpc, "t_delay", 0.0) * self.stmpc.MPC_freq)
        )
        self.delay_steps = max(
            0, int(getattr(self.stmpc, "steps_delay", 0)) + extra_steps
        )
        self.u_buffer: deque[np.ndarray] = deque(
            [np.zeros(self.model.n_u) for _ in range(self.delay_steps)],
            maxlen=max(self.delay_steps, 1),
        )

        # 8) Logging
        self.dt = 1.0 / float(self.stmpc.MPC_freq)
        self.u_prev = np.zeros(self.model.n_u)

    # -------------- warm start --------------

    def apply_warm_start(self, x: np.ndarray) -> None:
        """
        Usa o helper do teu projeto (get_warm_start) que devolve por passo [x|u] concatenado.
        """
        ws = self.get_warm_start(
            x=x, const_steer_vel=0.0 , d_acc=0.05,
        )
        for i in range(self.stmpc.N + 1):
            self.solver.set(i, "x", ws[i][: self.model.n_x])
            if i < self.stmpc.N:
                self.solver.set(i, "u", ws[i][self.model.n_x :])

        # self.x = ws[0][:8] # atualiza só os 3 primeiros estados

    def get_warm_start(
        self, x: np.ndarray, d_acc: float, const_steer_vel: float
    ) -> np.array:
        """
        Returns a warm start trajectory for the MPC. This is done by propagating the current state with a constant acceleration and steering angle.

        Input:  const_acc   : Constant acceleration
                const_steer : Constant steering angle

        Returns: np.array    : Warm start trajectory
        """
        v_x_min_ws = 0.5    # [m/s] mínimo para evitar singularidades
        warm_start = np.zeros(
            (self.stmpc.N + 1, 8)
        )  # TODO: hardcoded state space dimension
        warm_start[0] = np.array(
            [
                x[0],
                x[1],
                x[2],
                x[3],
                x[4],
                x[5],
                const_steer_vel,
                d_acc,
            ]
        )  # TODO setting the velocity to current actual velocity might remove slowing down
        for i in range(1, self.stmpc.N + 1):
            xdot = self._dynamics_of_car(0, warm_start[i - 1])
            """  warm_start[i] = (
                warm_start[i - 1] + np.array(xdot) / self.stmpc.MPC_freq
            ) """  # ugly euler integration
            warm_start[i]=warm_start[i - 1]
        return warm_start

    # -------------- um passo de MPC --------------
    def _solve_one_step(self) -> tuple[int, list[np.ndarray], list[np.ndarray], float]:
        """
        Define condição inicial, resolve e recolhe (x_N, u_N).
        Returns: (status, x_list, u_list, solve_ms)
        """
        self.solver.set(0, "lbx", self.x)
        self.solver.set(0, "ubx", self.x)

        t0 = time.perf_counter()
        status = int(self.solver.solve())
        solve_ms = float((time.perf_counter() - t0) * 1e3)

        x_list, u_list = [], []
        if status == 0:
            for s in range(self.stmpc.N + 1):
                x_list.append(self.solver.get(s, "x").reshape(-1))
            for s in range(self.stmpc.N):
                u_list.append(self.solver.get(s, "u").reshape(-1))
        else:
            x_list = [self.x.copy()]
            u_list = [self.u_prev.copy() for _ in range(self.stmpc.N)]
        return status, x_list, u_list, solve_ms

    # -------------- integração fallback --------------
    def _integrate_one_step(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Integra Euler explícito um passo com f_expl_func do modelo.
        """
        f = (
            self.model.f_expl_func(
                x[0],
                x[1],
                x[2],
                x[3],
                x[4],
                x[5],
                float(u[0]),
                float(u[1]),
                self.p_vec,
            )
            .full()
            .reshape(-1)
        )
        return (x + self.dt * f).astype(float)

    def _dynamics_of_car(self, t, x0) -> list:
        """
        Used for forward propagation. This function takes the dynamics from the acados model.
        """

        s = x0[0]
        n = x0[1]
        theta = x0[2]
        v_x = x0[3]
        yaw_rate = x0[4]
        delta = x0[5]
        derDelta = x0[6]
        derVx = x0[7]
        xdot = self.model.f_expl_func(
            s, n, theta, v_x, yaw_rate, delta, derDelta, derVx, self.p_vec
        )

        xdot = [
            float(xdot[0]),
            float(xdot[1]),
            float(xdot[2]),
            float(xdot[3]),
            float(xdot[4]),
            float(xdot[5]),
            derDelta,
            derVx,
        ]

        return xdot

    # -------------- loop principal --------------
    def run(self, sim_time: float) -> SimResult:
        n_steps = int(sim_time * self.stmpc.MPC_freq)
        x_hist = []
        u_hist = []
        status_hist = []
        solve_ms = []
        lap_times = []
        start_lap_time = 0
        s_ref_lap = self.s_ref[-1]

        #x_hist.append(self.x.copy())

        for k in range(n_steps):
            
            # Check for user input without blocking
            print("Press 'p' to pause/break simulation, or Enter to continue.")
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.readline().strip().lower()
                if user_input == "p":
                    print("Simulation paused by user.")
                    break
            if self.x[0] > s_ref_lap:
                lap_time = k * self.dt - start_lap_time
                lap_times.append(lap_time)
                print(f"Lap completed in {lap_time:.2f} seconds.")
                start_lap_time = k * self.dt
                s_ref_lap += self.s_ref[-1]
                if len(lap_times) == 2:  # Limit to 2 laps for brevity
                    print("Maximum number of laps reached. Ending simulation.")
                    log_dir = Path(__file__).parent / "Log_Data"
                    log_dir.mkdir(exist_ok=True)
                    filename = log_dir / f"lap_data_mpc_{time.strftime('%d_%m_%Y_%H_%M_%S')}.npz"
                    np.savez(
                        filename,
                        x_hist=np.array(x_hist),
                        u_hist=np.array(u_hist),
                        xN=self.xN,
                        uN=self.uN
                    )
                    print(f"Lap data saved to {filename.resolve()}")
                    break
                else:
                    x_hist = x_hist[-2:]
                    u_hist = u_hist[-2:]
                    self.xN = self.xN[-2:]
                    self.uN = self.uN[-2:]

            
            status, xN, uN, ms = self._solve_one_step()
            status_hist.append(status)
            solve_ms.append(ms)

            if status != 0:
                sys.stderr.write(
                    f"\033[91m[WARN] Solver falhou no passo {k} (status {status}). Integro 1 passo.\033[0m\n"
                )
                #self.solver.print_statistics()
                u_cmd = self.u_prev.copy()
                if self.delay_steps > 0:
                    self.u_buffer.append(u_cmd)
                    u_applied = self.u_buffer[0]
                else:
                    u_applied = u_cmd
                x_next = self._integrate_one_step(self.x, u_applied)
                xN = [self.x, x_next]
                uN = [self.u_prev, u_cmd]
            else:
                print(
                    f"\033[92m[INFO] Passo {k} resolvido em {ms:.1f} ms (status {status})\033[0m"
                )
                u_applied = uN[0]
                x_next = xN[1].astype(float)

                """ # controlo aplicado (com atraso se existir)
                if self.delay_steps > 0:
                    self.u_buffer.append(uN[0])
                    u_applied = self.u_buffer[0]
                else:
                    u_applied = uN[0]

                # estado seguinte: usa previsão do otimizador se sem atraso (mais rápido)
                if self.delay_steps == 0 and len(xN) >= 2:
                    x_next = xN[1].astype(float)
                else:
                    x_next = self._integrate_one_step(self.x, u_applied) """

            print(
                f"Actual state: {np.round(self.x, 2).tolist()} (s, n, mu, vx, delta, r) "
            )
            print(f"   Applied u: {np.round(u_applied, 2)} (derDelta , derVx) ")
            """ Fx_f, Fx_r, Fy_f, Fy_r, Fz_f, Fz_r = self.model.forces_func(
                self.x[0],
                self.x[1],
                self.x[2],
                self.x[3],
                self.x[4],
                self.x[5],
                self.x[6],
                self.x[7],
                float(u_applied[0]),
                float(u_applied[1]),
                self.p_vec,
            )
            Fx_f = Fx_f.full().ravel()[0]
            Fx_r = Fx_r.full().ravel()[0]
            Fy_f = Fy_f.full().ravel()[0]
            Fy_r = Fy_r.full().ravel()[0]
            Fz_f = Fz_f.full().ravel()[0]
            Fz_r = Fz_r.full().ravel()[0]
            print(
                f"   Forces: Fx_f={Fx_f:.2f}, Fx_r={Fx_r:.2f}, Fy_f={Fy_f:.2f}, Fy_r={Fy_r:.2f}, Fz_f={Fz_f:.2f}, Fz_r={Fz_r:.2f}"
            )
            
            self.Fz_f.append(Fz_f)
            self.Fz_r.append(Fz_r)
            self.Fx_f.append(Fx_f)
            self.Fx_r.append(Fx_r)
            self.Fy_f.append(Fy_f)
            self.Fy_r.append(Fy_r) """
            
            """  alpha_f, alpha_r = self.model.alpha_func(
                self.x[StateIndex.VELOCITY_V_X], self.x[StateIndex.VELOCITY_V_Y], self.x[StateIndex.YAW_RATE], self.x[StateIndex.DELTA]
            )
            self.alpha_f.append(float(alpha_f.full().ravel()[0]))
            self.alpha_r.append(float(alpha_r.full().ravel()[0])) """

            print(
                f"  Next state: {np.round(x_next, 2).tolist()} (s, n, mu, vx, delta, r) "
            )

            if status != 0:
                signal.signal(signal.SIGINT, handler_plots)  # agora Ctrl+C chama handler
                wait = input("Press Enter to continue or esc to exit...")
                if wait.lower() in ["esc", "exit", "quit", "q"]:
                    sys.exit(1)
                elif wait.lower() in ["p", "space"]:
                    print("Exiting sim loop.")
                    break
                else:
                    print("Continuing sim loop.")

            # plot em tempo-real (não bloqueante) – mantém a tua API
            if not self.no_plot and self.fig is not None:
                self.fig = update_sim_plot(
                    self.fig, self.tr, self.d_left, self.d_right, np.array(xN), status, k
                )
                

            # logging e avanço
            x_hist.append(x_next)
            u_hist.append(u_applied)
            self.xN.append(xN)
            self.uN.append(uN)
            self.u_prev = u_applied
            self.x = x_next
            
            print("LAP TIMES:", lap_times)

        return SimResult(
            x_hist=np.array(x_hist),
            u_hist=np.array(u_hist),
            status=np.array(status_hist),
            solve_ms=np.array(solve_ms),
            dt=self.dt,
            freq=np.array(float(self.stmpc.MPC_freq)),
            mpc_self=self
        )


# ---------------------------- CLI ----------------------------


def main():

    #traj_default = "./traj/centerline_0.10_test_map_v2.csv"
    #traj_default ='traj/centerline_map_2025-09-09_10-52-29.csv'
    #traj_default ='traj/centerline_0.05_map_2025-10-11_12-41-37.csv'
    traj_default ='traj/centerline_0.05_map_2026-01-21_15-14-39_v2.csv'
    #traj_default = Path("./traj/track_data.csv")

    # Exemplo: [s, n, mu, vx, r, delta] 27
    x_init = np.array([1.0, 0.1, 0.1 , 1.0, 0.01, 0.0], dtype=float)

    ap = argparse.ArgumentParser(
        description="Closed-loop MPC sim (acados) — class-based"
    )
    ap.add_argument(
        "--csv",
        type=Path,
        default=traj_default,
        help="Path to track CSV [s,x,y,kappa,(nl),(nr)]",
    )
    ap.add_argument(
        "--mpc_config",
        type=Path,
        default=Path(__file__).parent / "mpc_config.yaml",
        help="YAML-like MPC params file",
    )
    ap.add_argument(
        "--car_config",
        type=Path,
        default=Path(__file__).parent / "car_model.yaml",
        help="YAML-like car params file",
    )
    ap.add_argument("--sim-time", type=float, default=40.0, help="Total sim time [s]")
    ap.add_argument(
        "--track-width",
        type=float,
        default=None,
        help="Override track half-widths nl,nr with this value [m]",
    )
    ap.add_argument("--no-plot", action="store_true", help="Disable plotting/animation")
    args = ap.parse_args()

    # overrides carregados dos YAML (se existirem)
    overrides: list[dict] = []
    if args.car_config and args.car_config.exists():
        overrides.append(load_params_file(args.car_config))
    if args.mpc_config and args.mpc_config.exists():
        overrides.append(load_params_file(args.mpc_config))

    # plot estático opcional (antes do loop)
    if not args.no_plot and False:
        try:
            tr_preview = load_track_csv(args.csv, track_width_override=args.track_width)
            plot_track_and_boundaries(tr_preview, tr_preview["nl"], tr_preview["nr"])
        except Exception as e:
            print("[WARN] preview plot falhou:", e)

    sim = MPCSim(
        track_csv=args.csv,
        cfg_overrides=overrides,
        track_width_override=args.track_width,
        no_plot=args.no_plot,
        x_init=x_init,
    )

    res = sim.run(sim_time=args.sim_time)

    print(
        f"Done. Steps: {res.u_hist.shape[0]}, avg solve time: {res.solve_ms.mean():.2f} ms at {res.freq:.1f} Hz"
    )


    # pós-sim (figuras bloqueantes, só no fim)
    print("Plotting states & inputs...")
    plot_states(res.mpc_self, res.x_hist, res.u_hist, res.dt, save_prefix=None, show=True)



if __name__ == "__main__":
    main()
