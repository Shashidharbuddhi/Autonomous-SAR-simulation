
import pygame
import sys
import numpy as np
from environment import START, TARGETS, build_true_map, build_known_map
# Window and grid configuration
CELL_SIZE = 25
ROWS = 24
COLS = 32
GRID_WIDTH = COLS * CELL_SIZE
GRID_HEIGHT = ROWS * CELL_SIZE
PANEL_WIDTH = 240

WINDOW_WIDTH = GRID_WIDTH + PANEL_WIDTH
WINDOW_HEIGHT = GRID_HEIGHT

BACKGROUND = (24, 28, 36)
GRID_COLOR = (65, 72, 85)
OBSTACLE_COLOR = (100, 110, 125)
START_COLOR = (50, 200, 120)
TARGET_COLOR = (240, 190, 60)
TEXT_COLOR = (235, 238, 245)

pygame.init()

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Autonomous Search and Rescue Robot")
clock = pygame.time.Clock()
font = pygame.font.Font(None, 25)
small_font = pygame.font.Font(None, 20)

# The grid stores obstacle locations as (row, col) tuples.
def reset_environment():
    global true_map, known_map

    true_map = build_true_map()
    known_map = build_known_map(true_map)


reset_environment()

start = START
targets = TARGETS


def draw_text(text, x, y, font_obj=font, color=TEXT_COLOR):
    image = font_obj.render(text, True, color)
    screen.blit(image, (x, y))


def draw_grid():
    for row in range(ROWS):
        for col in range(COLS):
            cell = pygame.Rect(
                col * CELL_SIZE,
                row * CELL_SIZE,
                CELL_SIZE,
                CELL_SIZE
            )

            color = BACKGROUND

            if known_map[row, col] == 1:
                color = OBSTACLE_COLOR
            elif (row, col) == start:
                color = START_COLOR
            elif (row, col) in targets:
                color = TARGET_COLOR

            pygame.draw.rect(screen, color, cell)
            pygame.draw.rect(screen, GRID_COLOR, cell, 1)


def draw_panel():
    panel_x = GRID_WIDTH
    pygame.draw.rect(
        screen, (35, 41, 52),
        (panel_x, 0, PANEL_WIDTH, WINDOW_HEIGHT)
    )

    x = panel_x + 16
    draw_text("RESCUE ROBOT", x, 20)
    draw_text("Grid: 24 x 32", x, 60, small_font)
    draw_text(
    f"Known obstacles: {int(np.sum(known_map))}",
    x, 88, small_font
)

    draw_text("CONTROLS", x, 145)
    draw_text("Left-click: add obstacle", x, 180, small_font)
    draw_text("Left-click again: remove", x, 205, small_font)
    draw_text("R: reset obstacles", x, 245, small_font)
    draw_text("ESC: exit", x, 275, small_font)

    draw_text("LEGEND", x, 335)
    draw_text("Green: robot start", x, 370, small_font,
              START_COLOR)
    draw_text("Yellow: rescue target", x, 400, small_font,
              TARGET_COLOR)
    draw_text("Grey: obstacle", x, 430, small_font,
              OBSTACLE_COLOR)


running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_r:
                obstacles.clear()

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                mouse_x, mouse_y = event.pos

                # Only grid clicks should modify the map.
                if mouse_x < GRID_WIDTH and mouse_y < GRID_HEIGHT:
                    col = mouse_x // CELL_SIZE
                    row = mouse_y // CELL_SIZE
                    position = (row, col)

                    # Never let the user block the start or targets.
                    if position != start and position not in targets:
                        if position in obstacles:
                            obstacles.remove(position)
                        else:
                            obstacles.add(position)

    screen.fill(BACKGROUND)
    draw_grid()
    draw_panel()

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()
