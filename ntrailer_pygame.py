import numpy as np
from numpy.typing import NDArray
import pygame

def wrap_to_pi(theta):
    return (theta + np.pi) % (2*np.pi) - np.pi


class RoadTrain:
    def __init__(self, pos0: NDArray, wheelbase0: float, track0: float):
        self.pos0 = pos0.astype(np.float32)
        self.thetas = [0.0]
        self.wheelbases = [wheelbase0]
        self.tracks = [track0]
        self.hitch_dists = [wheelbase0]
        self.idx_to_steering_idx = {0: 0}
        self.steering_angles = [0.0]
    
    def add_trailer(self, wheelbase: float, track: float, hitch_dist: float, has_steering: bool):
        new_i = len(self.thetas)
        self.thetas.append(self.thetas[-1])  # line up with car in front
        self.wheelbases.append(wheelbase)
        self.tracks.append(track)
        self.hitch_dists.append(hitch_dist)
        if has_steering:
            self.idx_to_steering_idx[new_i] = len(self.steering_angles) 
            self.steering_angles.append(0.0)
    
    def reset(self):
        self.pos0 = [0.0] * len(self.pos0)
        self.thetas = [0.0] * len(self.thetas)
        self.steering_angles = [0.0] * len(self.steering_angles)
    
    def tick(self, dt: float, v: float, steering_angles: list[float]):
        assert len(self.steering_angles) == len(steering_angles), "Different number of steering angles than expected"
        self.steering_angles = steering_angles
        self.pos0 += v * dt * np.array([np.cos(self.thetas[0]), np.sin(self.thetas[0])])
        for i in range(len(self.thetas)):
            if i <= 1:
                v_prev = v
            else:
                v_prev = v_prev * np.cos(self.thetas[i-1] - self.thetas[i])
            if i == 0:
                dtheta = v / self.hitch_dists[0] * np.tan(steering_angles[0])
            elif (j := self.idx_to_steering_idx.get(i, None)) is not None:
                dtheta = v_prev / self.hitch_dists[i] * (
                    np.sin(self.thetas[i-1] - self.thetas[i])
                    + np.tan(self.steering_angles[j]) * np.cos(self.thetas[i-1] - self.thetas[i])
                )
            else:
                dtheta = v_prev / self.hitch_dists[i] * np.sin(self.thetas[i-1] - self.thetas[i])
            self.thetas[i] += dtheta * dt
            # diff = wrap_to_pi(relative_angle)
            # if diff < -np.deg2rad(90):
            #     self.thetas[i] = self.thetas[i-1] + np.deg2rad(90)
            # elif diff > np.deg2rad(90):
            #     self.thetas[i] = self.thetas[i-1] - np.deg2rad(90)
            # else:
            #     self.thetas[i] = wrap_to_pi(self.thetas[i])
        
    def render(self, screen: pygame.Surface, zoom: float):
        def world_to_screen(p):
            return pygame.Vector2(zoom*p[0] + screen.get_width()/2, screen.get_height()/2 - zoom*p[1])
        p_prev = self.pos0
        steering_angles = self.steering_angles
        for i in range(len(self.thetas)):
            if i > 0:
                p_this = p_prev - self.hitch_dists[i] * np.array([np.cos(self.thetas[i]), np.sin(self.thetas[i])])
            else:
                p_this = p_prev
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
                pygame.draw.line(screen, "white", world_to_screen(p_this), world_to_screen(p_prev))
            if i in self.idx_to_steering_idx:
                front = p_this + R @ np.array([wheelbase, 0])
                d = np.array([np.cos(theta+steering_angles[0]), np.sin(theta+steering_angles[0])])
                steering_angles = steering_angles[1:]
                pygame.draw.line(screen, "red", world_to_screen(front), world_to_screen(front+2*d))
            p_prev = p_this


def main():
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    clock = pygame.time.Clock()
    running = True
    dt = 0

    model = RoadTrain(np.array([0, 0]), wheelbase0=4, track0=3)
    model.add_trailer(wheelbase=6, track=3, hitch_dist=8, has_steering=False)
    model.add_trailer(wheelbase=3, track=3, hitch_dist=5, has_steering=True)
    model.add_trailer(wheelbase=6, track=3, hitch_dist=8, has_steering=False)
    model.add_trailer(wheelbase=3, track=3, hitch_dist=5, has_steering=False)

    ZOOM = 8

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
            model.reset()

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

        model.tick(dt, v, [w, w2])

        model.render(screen, zoom=ZOOM)

        # flip() the display to put your work on screen
        pygame.display.flip()

        # limits FPS to 60
        # dt is delta time in seconds since last frame, used for framerate-
        # independent physics.
        dt = clock.tick(60) / 1000

    pygame.quit()


if __name__ == "__main__":
    main()
