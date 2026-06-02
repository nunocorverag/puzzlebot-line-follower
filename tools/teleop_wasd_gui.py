#!/usr/bin/env python3
"""Laptop-side WASD teleop with TRUE simultaneous keys (pygame).

Reads real key state every frame, so holding w+a together is a genuine
forward-left CURVE (not a one-wheel pivot, and no terminal auto-repeat limits).
Releasing a key stops that motion instantly. Sends "v w" UDP datagrams to the
Jetson bridge (tools/cmd_vel_udp_bridge.py), which republishes /cmd_vel.

Keys (only while this window is focused):
  W / S   forward / back
  A / D   steer left / right   (held with W = curve)
  Q / E   pivot left / right   (rotate in place)
  SPACE   stop
  - / =   slower / faster (scale)
  ESC / close window   quit (sends zero)

Env: JETSON_HOST (10.10.0.100), UDP_PORT (5005),
     V_MAX (0.15), STEER_W (0.5), PIVOT_W (1.2)
"""

from __future__ import annotations

import os
import socket

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402


def main() -> int:
    host = os.environ.get("JETSON_HOST", "10.10.0.100")
    port = int(os.environ.get("UDP_PORT", "5005"))
    v_max = float(os.environ.get("V_MAX", "0.15"))
    steer_w = float(os.environ.get("STEER_W", "0.5"))
    pivot_w = float(os.environ.get("PIVOT_W", "1.2"))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, port)

    def send(v: float, w: float) -> None:
        try:
            sock.sendto(f"{v:.4f} {w:.4f}".encode(), addr)
        except OSError:
            pass

    pygame.init()
    screen = pygame.display.set_mode((520, 210))
    pygame.display.set_caption(f"WASD teleop -> {host}:{port}")
    font = pygame.font.SysFont("monospace", 18)
    clock = pygame.time.Clock()

    scale = 1.0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_MINUS:
                    scale = max(0.2, round(scale - 0.1, 2))
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    scale = min(1.5, round(scale + 0.1, 2))

        keys = pygame.key.get_pressed()
        lin = 0.0
        ang = 0.0
        if keys[pygame.K_w]:
            lin += v_max
        if keys[pygame.K_s]:
            lin -= v_max
        if keys[pygame.K_a]:
            ang += steer_w
        if keys[pygame.K_d]:
            ang -= steer_w
        if keys[pygame.K_q]:
            ang += pivot_w
        if keys[pygame.K_e]:
            ang -= pivot_w
        if keys[pygame.K_SPACE]:
            lin = 0.0
            ang = 0.0

        v = lin * scale
        w = ang * scale
        send(v, w)

        screen.fill((20, 20, 26))
        rows = [
            "WASD teleop  (focus this window to drive)",
            f"v = {v:+.3f} m/s     w = {w:+.2f} rad/s     scale = {scale:.1f}",
            "W/S fwd/back   A/D steer (W+A = curve)   Q/E pivot",
            "SPACE stop     - / = speed     ESC quit",
        ]
        y = 22
        for i, text in enumerate(rows):
            color = (120, 230, 120) if i == 0 else (220, 220, 220)
            screen.blit(font.render(text, True, color), (16, y))
            y += 40
        pygame.display.flip()
        clock.tick(60)

    for _ in range(10):
        send(0.0, 0.0)
    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
