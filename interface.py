
import sys

import numpy as np
import pygame

from environment import START, TARGETS, build_true_map, build_known_map


# -------------------- Configuration --------------------

ROWS = 24
COLS = 32

CELL_SIZE = 25
GRID_WIDTH = COLS * CELL_SIZE
GRID_HEIGHT = ROWS * CELL_SIZE

PANEL_WIDTH = 260
WINDOW_WIDTH = GRID_WIDTH + PANEL_WIDTH
WINDOW_HEIGHT = GRID_HEIGHT

FPS = 60

# Colors
BACKGROUND = (245, 247, 250)
GRID_COLOR = (210, 216, 224)
OBSTACLE_COLOR = (75, 85, 99)
START_COLOR = (34, 197, 94)
TARGET_COLOR = (250, 204, 21)
PANEL_COLOR = (230, 235, 242)
TEXT_COLOR = (31, 41, 55)
MUTED_TEXT = (90, 100, 115)
BUTTON_COLOR = (59, 130, 246)
BUTTON_HOVER_COLOR = (37, 99, 235)
BUTTON_TEXT_COLOR = (255, 255, 255)
BORDER_COLOR = (180, 190, 205)


# -------------------- Initialization --------------------

pygame.init()

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Autonomous Search and Rescue Robot")

clock = pygame.time.Clock()

title_font = pygame.font.SysFont("arial", 21, bold=True)
font = pygame.font.SysFont("arial", 17)
small_font = pygame.font.SysFont("arial", 14)


# -------------------- Environment --------------------

def reset_environment():
    """Restore the original simulation map."""
    global true_map, known_map

    true_map = build_true_map()
    known_map = build_known_map(true_map)


reset_environment()

start = START
targets = list(TARGETS)


# -------------------- Drawing Helpers --------------------

def draw_text(text, x, y, font_object, color=TEXT_COLOR):
    """Draw text at the given screen coordinates."""
    image = font_object.render(str(text), True, color)
    screen.blit(image, (x, y))


def draw_grid():
    """Draw the map, known obstacles, start, and rescue targets."""
    for row in range(ROWS):
        for col in range(COLS):
            rect = pygame.Rect(
                col * CELL_SIZE,
                row * CELL_SIZE,
                CELL_SIZE,
                CELL_SIZE,
            )

            color = BACKGROUND

            if known_map[row, col] == 1:
                color = OBSTACLE_COLOR
            elif (row, col) == start:
                color = START_COLOR
            elif (row, col) in targets:
                color = TARGET_COLOR

            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRID_COLOR, rect, 1)

    # Draw a white S inside the starting cell.
    sx, sy = start[1] * CELL_SIZE, start[0] * CELL_SIZE
    start_label = small_font.render("S", True, (255, 255, 255))
    screen.blit(start_label, start_label.get_rect(
        center=(sx + CELL_SIZE // 2, sy + CELL_SIZE // 2)
    ))

    # Label each rescue target with its number.
    for index, (row, col) in enumerate(targets, start=1):
        tx, ty = col * CELL_SIZE, row * CELL_SIZE
        label = small_font.render(str(index), True, TEXT_COLOR)
        screen.blit(label, label.get_rect(
            center=(tx + CELL_SIZE // 2, ty + CELL_SIZE // 2)
        ))


def draw_button(rect, text, mouse_position):
    """Draw a button with a hover effect."""
    color = (
        BUTTON_HOVER_COLOR
        if rect.collidepoint(mouse_position)
        else BUTTON_COLOR
    )

    pygame.draw.rect(screen, color, rect, border_radius=7)

    label = font.render(text, True, BUTTON_TEXT_COLOR)
    screen.blit(label, label.get_rect(center=rect.center))


def draw_panel(mouse_position):
    """Draw the information panel and reset button."""
    panel_x = GRID_WIDTH

    pygame.draw.rect(
        screen,
        PANEL_COLOR,
        (panel_x, 0, PANEL_WIDTH, WINDOW_HEIGHT),
    )

    x = panel_x + 18

    draw_text("RESCUE SIMULATOR", x, 22, title_font)
    pygame.draw.line(
        screen, BORDER_COLOR,
        (x, 56), (WINDOW_WIDTH - 18, 56), 2
    )

    draw_text("Environment", x, 72, font)

    draw_text(f"Grid: {ROWS} x {COLS}", x, 104, small_font)
    draw_text(f"Start: {start}", x, 128, small_font)
    draw_text(f"Rescue targets: {len(targets)}", x, 152, small_font)

    known_obstacles = int(np.count_nonzero(known_map))
    draw_text(
        f"Known obstacles: {known_obstacles}",
        x, 176, small_font
    )

    hidden_obstacles = int(
        np.count_nonzero(true_map) - np.count_nonzero(known_map)
    )
    draw_text(
        f"Hidden obstacles: {max(0, hidden_obstacles)}",
        x, 200, small_font
    )

    pygame.draw.line(
        screen, BORDER_COLOR,
        (x, 235), (WINDOW_WIDTH - 18, 235), 2
    )

    draw_text("Controls", x, 252, font)
    draw_text("Left-click: add/remove obstacle", x, 282, small_font)
    draw_text("R: reset original environment", x, 306, small_font)
    draw_text("ESC: quit simulation", x, 330, small_font)

    draw_text("Legend", x, 370, font)

    legend_items = [
        (START_COLOR, "Robot start"),
        (TARGET_COLOR, "Rescue target"),
        (OBSTACLE_COLOR, "Known obstacle"),
        (BACKGROUND, "Free cell"),
    ]

    for index, (color, label) in enumerate(legend_items):
        y = 402 + index * 29

        pygame.draw.rect(
            screen,
            color,
            (x, y, 20, 20),
            border_radius=3,
        )
        pygame.draw.rect(
            screen,
            BORDER_COLOR,
            (x, y, 20, 20),
            1,
            border_radius=3,
        )

        draw_text(label, x + 30, y + 2, small_font)

    reset_button = pygame.Rect(
        x, WINDOW_HEIGHT - 58, PANEL_WIDTH - 36, 38
    )
    draw_button(reset_button, "Reset Environment", mouse_position)

    return reset_button


# -------------------- Map Editing --------------------

def edit_obstacle(mouse_position):
    """Toggle an obstacle in both the known and true maps."""
    x, y = mouse_position

    # Ignore clicks outside the grid.
    if x < 0 or x >= GRID_WIDTH or y < 0 or y >= GRID_HEIGHT:
        return

    col = x // CELL_SIZE
    row = y // CELL_SIZE
    position = (row, col)

    # Never allow obstacles on the start or target cells.
    if position == start or position in targets:
        return

    # Toggle the selected cell.
    new_value = 0 if known_map[row, col] == 1 else 1

    known_map[row, col] = new_value
    true_map[row, col] = new_value


# -------------------- Main Loop --------------------

def main():
    running = True

    while running:
        clock.tick(FPS)
        mouse_position = pygame.mouse.get_pos()

        screen.fill(BACKGROUND)

        draw_grid()
        reset_button = draw_panel(mouse_position)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_r:
                    reset_environment()

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    if reset_button.collidepoint(event.pos):
                        reset_environment()
                    else:
                        edit_obstacle(event.pos)

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
