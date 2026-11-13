
"""Interactive search-and-rescue robot simulation using Pygame."""

import sys

import numpy as np
import pygame

from environment import START, TARGETS, build_true_map, build_known_map
from robot import RescueRobot


# -------------------- Configuration --------------------

ROWS = 24
COLS = 32

CELL_SIZE = 25
GRID_WIDTH = COLS * CELL_SIZE
GRID_HEIGHT = ROWS * CELL_SIZE

PANEL_WIDTH = 280
WINDOW_WIDTH = GRID_WIDTH + PANEL_WIDTH
WINDOW_HEIGHT = GRID_HEIGHT

FPS = 60
STEP_INTERVAL_MS = 180

# Colors
BACKGROUND = (245, 247, 250)
GRID_COLOR = (210, 216, 224)
OBSTACLE_COLOR = (75, 85, 99)
START_COLOR = (34, 197, 94)
TARGET_COLOR = (250, 204, 21)
VISITED_COLOR = (45, 190, 180)
ROBOT_COLOR = (37, 99, 235)
PATH_COLOR = (147, 197, 253)
PANEL_COLOR = (230, 235, 242)
TEXT_COLOR = (31, 41, 55)
MUTED_TEXT = (90, 100, 115)
BUTTON_COLOR = (59, 130, 246)
BUTTON_HOVER_COLOR = (37, 99, 235)
BUTTON_TEXT_COLOR = (255, 255, 255)
BORDER_COLOR = (180, 190, 205)


# -------------------- Pygame Initialization --------------------

pygame.init()

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Autonomous Search and Rescue Robot")

clock = pygame.time.Clock()

title_font = pygame.font.SysFont("arial", 20, bold=True)
font = pygame.font.SysFont("arial", 16)
small_font = pygame.font.SysFont("arial", 13)


# -------------------- Simulation State --------------------

start = START
targets = list(TARGETS)

true_map = None
known_map = None
robot = None

mission_running = False
target_index = 0
mission_status = "Ready"
last_step_time = 0


def reset_environment():
    """Reset the map, robot, and mission progress."""
    global true_map, known_map, robot
    global mission_running, target_index, mission_status

    true_map = build_true_map()
    known_map = build_known_map(true_map)

    robot = RescueRobot(
        true_map=true_map,
        known_map=known_map,
        start=start,
        sensor_radius=2,
    )

    # Use the robot's map as the shared known map.
    known_map = robot.known_map

    mission_running = False
    target_index = 0
    mission_status = "Ready"


reset_environment()


# -------------------- Drawing Helpers --------------------

def draw_text(text, x, y, font_object, color=TEXT_COLOR):
    image = font_object.render(str(text), True, color)
    screen.blit(image, (x, y))


def cell_center(cell):
    """Convert a grid cell (row, column) to a pixel center."""
    row, col = cell

    return (
        col * CELL_SIZE + CELL_SIZE // 2,
        row * CELL_SIZE + CELL_SIZE // 2,
    )


def draw_grid():
    """Draw obstacles, targets, the robot's path, and the robot."""
    for row in range(ROWS):
        for col in range(COLS):
            cell = (row, col)

            rect = pygame.Rect(
                col * CELL_SIZE,
                row * CELL_SIZE,
                CELL_SIZE,
                CELL_SIZE,
            )

            color = BACKGROUND

            if known_map[row, col] == 1:
                color = OBSTACLE_COLOR
            elif cell == start:
                color = START_COLOR
            elif cell in robot.visited_targets:
                color = VISITED_COLOR
            elif cell in targets:
                color = TARGET_COLOR

            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRID_COLOR, rect, 1)

    # Draw the robot's traveled path.
    if len(robot.path_history) > 1:
        points = [cell_center(cell) for cell in robot.path_history]

        pygame.draw.lines(
            screen,
            PATH_COLOR,
            False,
            points,
            3,
        )

    # Draw target numbers.
    for index, target in enumerate(targets, start=1):
        center = cell_center(target)

        if target in robot.visited_targets:
            label_text = "✓"
            label_color = (255, 255, 255)
        else:
            label_text = str(index)
            label_color = TEXT_COLOR

        label = font.render(label_text, True, label_color)
        screen.blit(label, label.get_rect(center=center))

    # Draw the start marker.
    start_label = small_font.render("S", True, (255, 255, 255))
    screen.blit(
        start_label,
        start_label.get_rect(center=cell_center(start)),
    )

    # Draw the robot last so it remains visible over the path.
    robot_center = cell_center(robot.position)

    pygame.draw.circle(
        screen,
        ROBOT_COLOR,
        robot_center,
        CELL_SIZE // 2 - 3,
    )

    pygame.draw.circle(
        screen,
        (255, 255, 255),
        robot_center,
        CELL_SIZE // 2 - 3,
        2,
    )


def draw_button(rect, label, mouse_position, enabled=True):
    """Draw a clickable button."""
    if not enabled:
        color = (155, 165, 180)
    elif rect.collidepoint(mouse_position):
        color = BUTTON_HOVER_COLOR
    else:
        color = BUTTON_COLOR

    pygame.draw.rect(screen, color, rect, border_radius=7)

    text_surface = font.render(label, True, BUTTON_TEXT_COLOR)

    screen.blit(
        text_surface,
        text_surface.get_rect(center=rect.center),
    )


def draw_panel(mouse_position):
    """Draw mission controls and live statistics."""
    panel_x = GRID_WIDTH
    x = panel_x + 16
    content_width = PANEL_WIDTH - 32

    pygame.draw.rect(
        screen,
        PANEL_COLOR,
        (panel_x, 0, PANEL_WIDTH, WINDOW_HEIGHT),
    )

    draw_text("RESCUE SIMULATOR", x, 17, title_font)

    pygame.draw.line(
        screen,
        BORDER_COLOR,
        (x, 48),
        (WINDOW_WIDTH - 16, 48),
        2,
    )

    draw_text("Mission Controls", x, 60, font)

    button_y = 88
    button_height = 34
    button_gap = 8
    button_width = (content_width - button_gap) // 2

    start_button = pygame.Rect(
        x, button_y, button_width, button_height
    )
    pause_button = pygame.Rect(
        x + button_width + button_gap,
        button_y,
        button_width,
        button_height,
    )

    reset_button = pygame.Rect(
        x,
        button_y + button_height + 8,
        content_width,
        button_height,
    )

    draw_button(
        start_button,
        "Start",
        mouse_position,
        enabled=not mission_running
        and mission_status not in ("Completed", "Failed"),
    )

    draw_button(
        pause_button,
        "Pause",
        mouse_position,
        enabled=mission_running,
    )

    draw_button(
        reset_button,
        "Reset Mission",
        mouse_position,
    )

    pygame.draw.line(
        screen,
        BORDER_COLOR,
        (x, 178),
        (WINDOW_WIDTH - 16, 178),
        2,
    )

    draw_text("Live Statistics", x, 190, font)

    completed = len(robot.visited_targets)
    total_targets = len(targets)

    current_target = (
        targets[target_index]
        if target_index < len(targets)
        else None
    )

    next_target_label = (
        str(current_target)
        if current_target is not None
        else "None"
    )

    stats = [
        f"Status: {mission_status}",
        f"Robot position: {robot.position}",
        f"Targets visited: {completed}/{total_targets}",
        f"Next target: {next_target_label}",
        f"Path steps: {len(robot.path_history) - 1}",
        f"Known obstacles: {int(np.count_nonzero(known_map))}",
        f"Discovered obstacles: {len(robot.discovered_obstacles)}",
        f"Planning calls: {robot.planning_calls}",
        f"Replan events: {robot.replans}",
    ]

    for index, item in enumerate(stats):
        draw_text(item, x, 222 + index * 23, small_font)

    pygame.draw.line(
        screen,
        BORDER_COLOR,
        (x, 440),
        (WINDOW_WIDTH - 16, 440),
        2,
    )

    draw_text("Map Legend", x, 451, font)

    legend = [
        (ROBOT_COLOR, "Robot"),
        (START_COLOR, "Start"),
        (TARGET_COLOR, "Unvisited target"),
        (VISITED_COLOR, "Visited target"),
        (OBSTACLE_COLOR, "Known obstacle"),
    ]

    for index, (color, label) in enumerate(legend):
        y = 480 + index * 21

        pygame.draw.rect(
            screen,
            color,
            (x, y, 15, 15),
            border_radius=3,
        )

        draw_text(label, x + 23, y, small_font)

    draw_text(
        "Edit obstacles only while paused.",
        x,
        WINDOW_HEIGHT - 20,
        small_font,
        MUTED_TEXT,
    )

    return start_button, pause_button, reset_button


# -------------------- Mission Controls --------------------

def start_mission():
    """Start or resume navigation."""
    global mission_running, mission_status, last_step_time

    if mission_status in ("Completed", "Failed"):
        return

    if target_index >= len(targets):
        mission_status = "Completed"
        mission_running = False
        return

    mission_running = True
    mission_status = "Running"
    last_step_time = pygame.time.get_ticks()


def pause_mission():
    """Pause robot movement."""
    global mission_running, mission_status

    mission_running = False

    if mission_status == "Running":
        mission_status = "Paused"


def update_mission():
    """Perform at most one robot movement step per update."""
    global target_index, mission_running, mission_status

    if not mission_running:
        return

    if target_index >= len(targets):
        mission_running = False
        mission_status = (
            "Completed"
            if len(robot.visited_targets) == len(targets)
            else "Failed"
        )
        return

    target = targets[target_index]
    result = robot.step_toward(target)

    if result in ("reached", "unreachable"):
        target_index += 1

        if result == "unreachable":
            # Continue the mission to test the remaining targets.
            mission_status = "Running"

        if target_index >= len(targets):
            mission_running = False
            mission_status = (
                "Completed"
                if len(robot.visited_targets) == len(targets)
                else "Failed"
            )


# -------------------- Obstacle Editing --------------------

def edit_obstacle(mouse_position):
    """Toggle a map cell while the mission is not running."""
    global known_map

    if mission_running:
        return

    x, y = mouse_position

    if not (0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT):
        return

    col = x // CELL_SIZE
    row = y // CELL_SIZE
    cell = (row, col)

    if cell == start or cell in targets:
        return

    new_value = 0 if known_map[row, col] == 1 else 1

    true_map[row, col] = new_value
    known_map[row, col] = new_value
    robot.known_map[row, col] = new_value


# -------------------- Main Event Loop --------------------

def main():
    global last_step_time

    running = True

    while running:
        clock.tick(FPS)
        now = pygame.time.get_ticks()
        mouse_position = pygame.mouse.get_pos()

        screen.fill(BACKGROUND)

        draw_grid()

        start_button, pause_button, reset_button = draw_panel(
            mouse_position
        )

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_r:
                    reset_environment()

                elif event.key == pygame.K_SPACE:
                    if mission_running:
                        pause_mission()
                    else:
                        start_mission()

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button != 1:
                    continue

                if start_button.collidepoint(event.pos):
                    start_mission()

                elif pause_button.collidepoint(event.pos):
                    pause_mission()

                elif reset_button.collidepoint(event.pos):
                    reset_environment()

                else:
                    edit_obstacle(event.pos)

        if mission_running and now - last_step_time >= STEP_INTERVAL_MS:
            update_mission()
            last_step_time = now

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
