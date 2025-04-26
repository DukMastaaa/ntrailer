import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray
import casadi as cas
import do_mpc
import pygame
import os


def wrap_to_pi(theta):
    return (theta + np.pi) % (2*np.pi) - np.pi


class GeneralNTrailer1Steer:
    def __init__(self, pos0: NDArray, wheelbase0: float, track0: float):
        self.pos_front = pos0.astype(np.float32)
        self.thetas = [0.0]
        self.wheelbases = [wheelbase0]
        self.tracks = [track0]
        self.bar_dists = [wheelbase0]
        self.hitch_dists = [0]
        self.steering_angle = 0.0
    
    def add_trailer(self, wheelbase: float, track: float, bar_dist: float, hitch_dist: float):
        self.thetas.append(self.thetas[-1])  # line up with car in front
        self.wheelbases.append(wheelbase)
        self.tracks.append(track)
        self.bar_dists.append(bar_dist)
        self.hitch_dists.append(hitch_dist)
    
    def reset_state(self):
        self.pos_front = [0.0] * len(self.pos_front)
        self.thetas = [0.0] * len(self.thetas)
        self.steering_angle = 0.0
    
    def tick(self, dt: float, v: float, steering_angle: float):
        self.steering_angle = steering_angle
        self.pos_front += v * dt * np.array([np.cos(self.thetas[0]), np.sin(self.thetas[0])])

        vs = np.zeros_like(self.thetas)
        dthetas = np.zeros_like(self.thetas)

        vs[0] = v
        dthetas[0] = v / self.bar_dists[0] * np.tan(self.steering_angle)
        
        for i in range(len(self.thetas)-1):
            vs[i+1] = (
                vs[i] * np.cos(self.thetas[i]-self.thetas[i+1])
                +
                self.hitch_dists[i+1] * np.sin(self.thetas[i]-self.thetas[i+1]) * dthetas[i]
            )
            dthetas[i+1] = 1 / self.bar_dists[i+1] * (
                vs[i] * np.sin(self.thetas[i]-self.thetas[i+1])
                -
                self.hitch_dists[i+1] * dthetas[i] * np.cos(self.thetas[i]-self.thetas[i+1])
            )

        for i in range(len(self.thetas)):
            self.thetas[i] += dthetas[i] * dt
        
    def render(self, screen: pygame.Surface, zoom: float, pos=None):
        def world_to_screen(p):
            return pygame.Vector2(zoom*p[0] + screen.get_width()/2, screen.get_height()/2 - zoom*p[1])
        # Draw goal
        if pos is not None:
            pygame.draw.circle(screen, "blue", world_to_screen(pos), 5)
        p_hitch_prev = self.pos_front
        p_this_prev = p_hitch_prev
        n = len(self.thetas)
        for i in range(n):
            if i > 0:
                p_this = p_hitch_prev - self.bar_dists[i] * np.array([np.cos(self.thetas[i]), np.sin(self.thetas[i])])
            else:
                p_this = p_hitch_prev
            theta = self.thetas[i]
            track = self.tracks[i]
            wheelbase = self.wheelbases[i]
            R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            bl = world_to_screen(p_this + R @ np.array([0, track/2]))
            br = world_to_screen(p_this + R @ np.array([0, -track/2]))
            fl = world_to_screen(p_this + R @ np.array([wheelbase, track/2]))
            fr = world_to_screen(p_this + R @ np.array([wheelbase, -track/2]))
            pygame.draw.line(screen, "white", bl, fl)
            pygame.draw.line(screen, "white", fl, fr)
            pygame.draw.line(screen, "white", fr, br)
            pygame.draw.line(screen, "white", br, bl)
            if i > 0:
                pygame.draw.line(screen, "cyan", world_to_screen(p_this), world_to_screen(p_hitch_prev))
                pygame.draw.line(screen, "white", world_to_screen(p_hitch_prev), world_to_screen(p_this_prev))
            if i == 0:
                front = p_this + R @ np.array([wheelbase, 0])
                d = np.array([np.cos(theta+self.steering_angle), np.sin(theta+self.steering_angle)])
                pygame.draw.line(screen, "red", world_to_screen(front), world_to_screen(front+2*d))
            p_this_prev = p_this
            p_hitch_prev = p_this - (self.hitch_dists[i+1] if i+1<n else 0) * np.array([np.cos(self.thetas[i]), np.sin(self.thetas[i])])
    

def setup_model(rt: GeneralNTrailer1Steer, pos_set, theta_all_set) -> do_mpc.model.Model:
    model = do_mpc.model.Model(model_type="continuous", symvar_type="SX")
    n = len(rt.thetas)

    # States
    pos = model.set_variable("_x", "pos", (2,1))
    theta = model.set_variable("_x", "theta", (n,1))

    # Inputs
    v0 = model.set_variable("_u", "v0")
    phi = model.set_variable("_u", "phi")

    # State derivatives
    vs = []
    vs.append(v0)
    dthetas = []
    dthetas.append(v0/rt.bar_dists[0] * cas.tan(phi[0]))

    for i in range(n-1):
        vs.append(
            vs[i] * cas.cos(theta[i]-theta[i+1])
            +
            rt.hitch_dists[i+1] * cas.sin(theta[i]-theta[i+1]) * dthetas[i]
        )
        dthetas.append(
            1 / rt.bar_dists[i+1] * (
                vs[i] * cas.sin(theta[i]-theta[i+1])
                -
                rt.hitch_dists[i+1] * dthetas[i] * cas.cos(theta[i]-theta[i+1])
            )
        )

    dthetas = cas.vertcat(*dthetas)
    dpos = v0 * cas.vertcat(cas.cos(theta[0]), cas.sin(theta[0]))
    model.set_rhs("pos", dpos)
    model.set_rhs("theta", dthetas)

    print("dpos", dpos)
    print("dtheta", dthetas)

    # Setpoint
    # pos_set = model.set_variable("_tvp", "pos_set", (2,1))
    # theta_all_set = model.set_variable("_tvp", "theta_all_set")
    model.set_expression("pos_set", cas.SX(pos_set.flatten()))
    model.set_expression("theta_all_set", cas.SX(theta_all_set))

    # Build the model
    model.setup()

    return model



def wrapped_angle_radians(theta):
    # https://stackoverflow.com/a/78649131
    return cas.arctan2(cas.sin(theta), cas.cos(theta))


def setup_mpc(rt: GeneralNTrailer1Steer, model: do_mpc.model.Model, silence_solver: bool,
              dt: float, pred_horizon: int, max_iter: int):
    mpc = do_mpc.controller.MPC(model)

    # Set up MPC

    mpc.settings.n_horizon = pred_horizon  # Prediction horizon
    mpc.settings.t_step = dt  # Closed-loop control period
    mpc.settings.n_robust = 0  # Disable robust MPC
    mpc.settings.open_loop = True
    # Solver settings
    mpc.settings.state_discretization = "collocation"
    mpc.settings.collocation_type = "radau"
    mpc.settings.collocation_deg = 3
    mpc.settings.collocation_ni = 1

    # No animation of predictions so we turn this off for storage
    # mpc.settings.store_full_solution = False
    mpc.settings.store_full_solution = True

    mpc.settings.nlpsol_opts["ipopt"] = {"max_iter": max_iter}

    # IPOPT debug prints
    if silence_solver:
        mpc.settings.supress_ipopt_output()
    
    # TODO: need WSL, https://github.com/do-mpc/do-mpc/issues/28
    mpc.settings.set_linear_solver("MA27")

    # Set up running and terminal costs for MPC objective

    n = model.x["theta"].rows()
    proper_pos = model.aux["pos_set"]
    proper_theta = model.aux["theta_all_set"]

    # state_error = cas.vertcat(
    #     model.x["pos"]-proper_pos, 
    #     model.x["theta"]-proper_theta*cas.GenSX_ones(b)
    # )  # pos then theta
    state_error = cas.vertcat(
        model.x["pos"]-proper_pos,
        wrapped_angle_radians(model.x["theta"]-proper_theta*cas.GenSX_ones(n))
    )  # pos then theta

    # Q_running = cas.diag([0.5, 0.5] + [0.2] + [100] * (b-1))

    # Q_running = cas.diag([0, 0, 0, 0])

    # Q_terminal = cas.diag([0, 0, 1, 1])
    Q_terminal = cas.diag([0, 0] + [1] * n)

    R_running_scalar = cas.DM(0.1)

    # No input-dependent terminal cost allowed
    # R_terminal = 0.1 * cas.DM_eye(1+m)  # +1 for v0

    total_pos_error = 0
    actual_pos = model.x["pos"]
    for i in range(n):
        if i > 0:
            proper_pos = proper_pos - (rt.bar_dists[i] + rt.hitch_dists[i]) * np.array([np.cos(proper_theta), np.sin(proper_theta)])
            actual_pos = (
                actual_pos
                - rt.hitch_dists[i] * np.array([np.cos(model.x["theta"][i-1]), np.sin(model.x["theta"][i-1])])
                - rt.bar_dists[i] * np.array([np.cos(model.x["theta"][i]), np.sin(model.x["theta"][i])])
            )
        if i == 0:
            total_pos_error += 1 * cas.sumsqr(proper_pos-actual_pos)
        else:
            total_pos_error += 3 * cas.sumsqr(proper_pos-actual_pos)
    
    # running_cost = cas.bilin(Q_running, state_error)
    # running_cost = cas.transpose(state_error) @ (Q_running @ state_error)

    # running_cost = total_pos_error * 0.001
    running_cost = cas.DM(0)

    # running_cost = cas.bilin(Q_running, state_error) + total_pos_error
    # terminal_cost = cas.bilin(Q_terminal, state_error) + total_pos_error  # + cas.bilin(R_terminal, control_effort)
    # terminal_cost = cas.bilin(Q_terminal, state_error)
    terminal_cost = cas.transpose(state_error) @ (Q_terminal @ state_error) + total_pos_error
    # terminal_cost = cas.DM(0)

    mpc.set_objective(lterm=running_cost, mterm=terminal_cost)
    mpc.set_rterm(R_running_scalar)

    # Set input bounds

    speed_limit = 8

    mpc.bounds["lower", "_u", "v0"] = -speed_limit  # Can drive backward
    mpc.bounds["upper", "_u", "v0"] = speed_limit
    # mpc.bounds["lower", "_u", "phi"] = -cas.pi/4 * cas.GenDM_ones(m)
    # mpc.bounds["upper", "_u", "phi"] = +cas.pi/4 * cas.GenDM_ones(m)
    mpc.bounds["lower", "_u", "phi"] = -np.deg2rad(60)
    mpc.bounds["upper", "_u", "phi"] = +np.deg2rad(60)

    # differences = []
    # for i in range(1, b):
    #     differences.append(cas.norm_1(model.x["theta"][i] - model.x["theta"][i-1]))
    # if len(differences) > 0:
    #     mpc.set_nl_cons("collision", cas.vertcat(*differences), np.deg2rad(90))
    for i in range(1, n):
        mpc.set_nl_cons(f"collision_{i}", cas.fabs(wrapped_angle_radians(model.x["theta"][i-1] - model.x["theta"][i])), np.deg2rad(90))

    # for i in range(m):
    #     mpc.bounds["lower", "_u", f"phi_{i}"] = -cas.pi/4
    #     mpc.bounds["upper", "_u", f"phi_{i}"] = +cas.pi/4
    
    # Set obstacle constraint

    # mpc.set_nl_cons('obstacles', -model.aux['obstacle_distance'], 0)

    # Configure setpoint values

    # tvp_template = mpc.get_tvp_template()
    # def tvp_fun(t_ind):
    #     tvp_template["_tvp", :, "pos_set"] = pos
    #     tvp_template["_tvp", :, "theta_all_set"] = theta
    #     return tvp_template
    # mpc.set_tvp_fun(tvp_fun)

    # Build MPC

    mpc.setup()

    return mpc


def interactive(screen: pygame.Surface, rt: GeneralNTrailer1Steer, ZOOM: float):
    goal_pos = np.array([0.1, 10])
    goal_theta = 0

    clock = pygame.time.Clock()
    running = True
    dt = 0

    mouse_down_pos = None
    v = 0
    w = 0
    w2 = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill("black")

        keys = pygame.key.get_pressed()
        if keys[pygame.K_q]:
            running = False
        v = 0
        if keys[pygame.K_UP]:
            v = 5
        if keys[pygame.K_DOWN]:
            v = -5
        if keys[pygame.K_RIGHT]:
            w -= 1 * dt
        if keys[pygame.K_LEFT]:
            w += 1 * dt
        if keys[pygame.K_d]:
            w2 -= 1 * dt
        if keys[pygame.K_a]:
            w2 += 1 * dt

        if keys[pygame.K_SPACE]:
            w = 0
            w2 = 0
        if keys[pygame.K_0]:
            rt.reset_state()

        left, mid, right = pygame.mouse.get_pressed()

        if left and mouse_down_pos is None:
            mouse_down_pos = pygame.mouse.get_pos()
        if not left and mouse_down_pos is not None:
            mouse_down_pos = None
            v = 0
            # w = 0
        if mouse_down_pos is not None:
            mouse_pos = pygame.mouse.get_pos()
            w = -1/50 * (mouse_pos[0] - mouse_down_pos[0])
            v = -1 * 1/5 * (mouse_pos[1] - mouse_down_pos[1])
            pygame.draw.circle(screen, "red", mouse_down_pos, 10)
        
        w = np.clip(w, -np.pi/4, np.pi/4)
        w2 = np.clip(w2, -np.pi/4, np.pi/4)

        rt.tick(dt, v, w)

        rt.render(screen, zoom=ZOOM, pos=goal_pos)

        # flip() the display to put your work on screen
        pygame.display.flip()

        # limits FPS to 60
        # dt is delta time in seconds since last frame, used for framerate-
        # independent physics.
        dt = clock.tick(60) / 1000
    
    return None


def control(screen: pygame.Surface, rt: GeneralNTrailer1Steer, ZOOM: float):
    sim_duration = 60
    goal_pos = np.array([1, 20])
    goal_theta = np.deg2rad(0)

    dt = 0.1
    # pred_horizon = 80 if len(rt.thetas) == 2 else 120
    pred_horizon = 140
    max_iter = 1200
    model = setup_model(rt, goal_pos, goal_theta)
    mpc = setup_mpc(rt, model, silence_solver=False, 
                    dt=dt, pred_horizon=pred_horizon, max_iter=max_iter)

    mpc.x0["pos"] = rt.pos_front
    mpc.x0["theta"] = rt.thetas
    mpc.set_initial_guess()

    clock = pygame.time.Clock()

    # fig = plt.figure(figsize=(5,5))
    # plt.ion()
    # plt.show()

    # mpc_graphics = do_mpc.graphics.Graphics(mpc.data)

    # ax1 = plt.subplot(5, 1, 1)
    # ax1.set_ylabel('Pos')
    # mpc_graphics.add_line(var_type='_x', var_name='pos', axis=ax1)

    # ax2 = plt.subplot(5, 1, 2)
    # ax2.set_ylabel('Pos set')
    # # mpc_graphics.add_line(var_type='_tvp', var_name='pos_set', axis=ax2)

    # ax3 = plt.subplot(5, 1, 3)
    # ax3.set_ylabel('Theta')
    # mpc_graphics.add_line(var_type='_x', var_name='theta', axis=ax3)

    # ax4 = plt.subplot(5, 1, 4)
    # ax4.set_ylabel('phi')
    # mpc_graphics.add_line(var_type='_u', var_name='phi', axis=ax4)

    # ax5 = plt.subplot(5, 1, 5)
    # ax5.set_ylabel("v0")
    # mpc_graphics.add_line(var_type='_u', var_name='v0', axis=ax5)

    # for ax in [ax1, ax2, ax3, ax4, ax5]:
    #     ax.yaxis.set_label_position("right")
    #     ax.yaxis.tick_right()
    # ax5.set_xlabel('time [s]')    
    # fig.align_ylabels()
    # fig.tight_layout()

    screen.fill("black")
    rt.render(screen, zoom=ZOOM, pos=goal_pos)

    horizon = mpc.settings.n_horizon
    # horizon = 1

    for i in range(int(sim_duration/dt/horizon)):
        # Update controller state
        mpc.x0["pos"] = rt.pos_front
        mpc.x0["theta"] = rt.thetas
        mpc.set_initial_guess()
        # Calculate control
        _ = mpc.make_step(mpc.x0)
        # Apply control over next timesteps
        for i in range(horizon):
            u = (mpc.opt_x_num['_u', i, 0]*mpc._u_scaling).full()
            u = list(u.flatten())
            # Apply control
            v0 = u[0]
            phi = u[1]
            rt.tick(dt, v0, phi)
            # Plot results
            # mpc_graphics.plot_results()
            # mpc_graphics.plot_predictions()
            # mpc_graphics.reset_axes()
            # plt.show()
            # plt.pause(0.01)
            # Display on pygame
            screen.fill("black")
            rt.render(screen, zoom=ZOOM, pos=goal_pos)
            pygame.display.flip()
            clock.tick(24)
            # print("execute")
        
        # print((mpc.opt_x_num['_u', :, 0]))
    
    # n_steps = int(sim_duration/dt)
    # for k in range(n_steps):
    #     # Update controller state
    #     mpc.x0["pos"] = rt.pos_front
    #     mpc.x0["theta"] = rt.thetas
    #     # Calculate control
    #     _ = mpc.make_step(mpc.x0)
    #     # Apply control over next timesteps
    #     # for i in range(mpc.settings.n_horizon):
    #     for i in range(1):
    #         u = (mpc.opt_x_num['_u', i, 0]*mpc._u_scaling).full()
    #         u = list(u.flatten())
    #         # Apply control
    #         v0 = u[0]
    #         phi = u[1:]
    #         rt.tick(dt, v0, phi)
    #     # Plot results
    #     mpc_graphics.plot_results()
    #     mpc_graphics.plot_predictions()
    #     mpc_graphics.reset_axes()
    #     plt.show()
    #     plt.pause(dt)
    #     # Display on pygame
    #     screen.fill("black")
    #     rt.render(screen, zoom=ZOOM, pos=goal_pos)
    #     pygame.display.flip()
    #     if k == 0:
    #         input()

    # plt.ioff()
    # plt.show()



def main():

    INTERACTIVE = False

    os.environ['SDL_VIDEO_WINDOW_POS'] = "800,400"
    pygame.init()
    # screen = pygame.display.set_mode((1280, 720))
    screen = pygame.display.set_mode((1600, 1600))

    rt = GeneralNTrailer1Steer(np.array([0, 0]), wheelbase0=4, track0=3)
    rt.add_trailer(wheelbase=6, track=3, bar_dist=8, hitch_dist=1)
    rt.add_trailer(wheelbase=6, track=3, bar_dist=8, hitch_dist=2)
    # rt.add_trailer(wheelbase=3, track=3, hitch_dist=5, has_steering=False)
    # rt.add_trailer(wheelbase=6, track=3, hitch_dist=8, has_steering=False)
    # rt.add_trailer(wheelbase=3, track=3, hitch_dist=5, has_steering=False)

    ZOOM = 20

    if INTERACTIVE:
        interactive(screen, rt, ZOOM)
    else:
        control(screen, rt, ZOOM)

    pygame.quit()
    pygame.quit()


if __name__ == "__main__":
    main()
