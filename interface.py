
"""Interactive rescue robot with BFS/A* selection and comparison."""

import sys
import time

import numpy as np
import pygame

from environment import START, TARGETS, build_true_map, build_known_map
from path_planning import astar, bfs
from robot import RescueRobot


# -------------------- Configuration --------------------

ROWS, COLS = 24, 32
CELL_SIZE = 25

GRID_WIDTH = COLS * CELL_SIZE
GRID_HEIGHT = ROWS * CELL_SIZE
PANEL_WIDTH = 340

WINDOW_WIDTH = GRID_WIDTH + PANEL_WIDTH
WINDOW_HEIGHT = GRID_HEIGHT

FPS = 60
STEP_INTERVAL_MS = 180

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
BUTTON_HOVER = (37, 99, 235)
BUTTON_DISABLED = (155, 165, 180)
BUTTON_TEXT = (255, 255, 255)
BORDER_COLOR = (180, 190, 205)


# -------------------- Pygame Initialization --------------------

pygame.init()

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Autonomous Search and Rescue Robot")

clock = pygame.time.Clock()

title_font = pygame.font.SysFont("arial", 19, bold=True)
font = pygame.font.SysFont("arial", 15)
small_font = pygame.font.SysFont("arial", 12)


# -------------------- Simulation State --------------------

start = START
targets = list(TARGETS)

true_map = None
known_map = None
robot = None

algorithm = "A*"
mission_running = False
target_index = 0
mission_status = "Ready"
last_step_time = 0

comparison_results = None


def reset_environment():
    """Restore the original map and initialize a fresh robot."""
    global true_map, known_map, robot
    global mission_running, target_index, mission_status

    true_map = build_true_map()
    known_map = build_known_map(true_map)

    robot = RescueRobot(
        true_map=true_map,
        known_map=known_map,
        start=start,
        sensor_radius=2,
        algorithm=algorithm,
    )

    known_map = robot.known_map

    mission_running = False
    target_index = 0
    mission_status = "Ready"


reset_environment()


# -------------------- Drawing Helpers --------------------

def draw_text(text, x, y, font_object=small_font, color=TEXT_COLOR):
    surface = font_object.render(str(text), True, color)
    screen.blit(surface, (x, y))


def cell_center(cell):
    row, col = cell
    return (
        col * CELL_SIZE + CELL_SIZE // 2,
        row * CELL_SIZE + CELL_SIZE // 2,
    )


def draw_grid():
    """Draw map cells, robot history, targets, and current position."""
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

    if len(robot.path_history) > 1:
        points = [cell_center(cell) for cell in robot.path_history]
        pygame.draw.lines(screen, PATH_COLOR, False, points, 3)

    for index, target in enumerate(targets, start=1):
        label_text = "✓" if target in robot.visited_targets else str(index)
        label_color = (
            (255, 255, 255)
            if target in robot.visited_targets
            else TEXT_COLOR
        )
        label = font.render(label_text, True, label_color)
        screen.blit(label, label.get_rect(center=cell_center(target)))

    start_label = small_font.render("S", True, (255, 255, 255))
    screen.blit(
        start_label,
        start_label.get_rect(center=cell_center(start)),
    )

    pygame.draw.circle(
        screen,
        ROBOT_COLOR,
        cell_center(robot.position),
        CELL_SIZE // 2 - 3,
    )
    pygame.draw.circle(
        screen,
        (255, 255, 255),
        cell_center(robot.position),
        CELL_SIZE // 2 - 3,
        2,
    )


def draw_button(rect, label, mouse_position, enabled=True):
    if not enabled:
        color = BUTTON_DISABLED
    elif rect.collidepoint(mouse_position):
        color = BUTTON_HOVER
    else:
        color = BUTTON_COLOR

    pygame.draw.rect(screen, color, rect, border_radius=6)

    label_surface = font.render(label, True, BUTTON_TEXT)
    screen.blit(label_surface, label_surface.get_rect(center=rect.center))


# -------------------- BFS vs A* Benchmark --------------------

def benchmark_algorithms():
    """
    Compare both planners on the same known map and same endpoints.
    This benchmark does not move or modify the robot.
    """
    results = {}

    comparison_map = known_map.copy()
    source = start
    destination = targets[0]

    for name, planner in (("BFS", bfs), ("A*", astar)):
        path, explored, elapsed = planner(
            comparison_map,
            source,
            destination,
        )

        results[name] = {
            "found": bool(path),
            "path_length": max(0, len(path) - 1),
            "explored": int(explored),
            "time_ms": elapsed * 1000,
        }

    return results


# -------------------- Side Panel --------------------

def draw_panel(mouse_position):
    x = GRID_WIDTH + 15
    content_width = PANEL_WIDTH - 30

    pygame.draw.rect(
        screen,
        PANEL_COLOR,
        (GRID_WIDTH, 0, PANEL_WIDTH, WINDOW_HEIGHT),
    )

    draw_text("RESCUE ROBOT SIMULATOR", x, 14, title_font)

    pygame.draw.line(
        screen,
        BORDER_COLOR,
        (x, 43),
        (WINDOW_WIDTH - 15, 43),
        2,
    )

    # Algorithm selector.
    draw_text("Path-planning algorithm", x, 53, font)

    selector_y = 76
    selector_gap = 8
    selector_width = (content_width - selector_gap) // 2

    bfs_button = pygame.Rect(
        x, selector_y, selector_width, 30
    )
    astar_button = pygame.Rect(
        x + selector_width + selector_gap,
        selector_y,
        selector_width,
        30,
    )

    draw_button(
        bfs_button,
        "BFS" if algorithm != "BFS" else "BFS [Selected]",
        mouse_position,
        enabled=not mission_running,
    )
    draw_button(
        astar_button,
        "A*" if algorithm != "A*" else "A* [Selected]",
        mouse_position,
        enabled=not mission_running,
    )

    # Mission controls.
    controls_y = 117
    button_gap = 8
    half_width = (content_width - button_gap) // 2

    start_button = pygame.Rect(
        x, controls_y, half_width, 31
    )
    pause_button = pygame.Rect(
        x + half_width + button_gap,
        controls_y,
        half_width,
        31,
    )
    reset_button = pygame.Rect(
        x, controls_y + 37, content_width, 31
    )
    compare_button = pygame.Rect(
        x, controls_y + 74, content_width, 31
    )

    draw_button(
        start_button,
        "Start / Resume",
        mouse_position,
        enabled=(
            not mission_running
            and mission_status not in ("Completed", "Failed")
        ),
    )
    draw_button(
        pause_button,
        "Pause",
        mouse_position,
        enabled=mission_running,
    )
    draw_button(reset_button, "Reset Mission", mouse_position)
    draw_button(
        compare_button,
        "Compare BFS vs A*",
        mouse_position,
        enabled=not mission_running,
    )

    # Live mission statistics.
    section_y = 264

    pygame.draw.line(
        screen,
        BORDER_COLOR,
        (x, section_y),
        (WINDOW_WIDTH - 15, section_y),
        2,
    )

    draw_text("Live Mission Statistics", x, section_y + 8, font)

    current_target = (
        targets[target_index]
        if target_index < len(targets)
        else None
    )

    stats = [
        f"Status: {mission_status}",
        f"Algorithm: {algorithm}",
        f"Robot: {robot.position}",
        f"Target: {current_target}",
        f"Targets rescued: {len(robot.visited_targets)}/{len(targets)}",
        f"Path steps: {len(robot.path_history) - 1}",
        f"Planning calls: {robot.planning_calls}",
        f"Explored cells total: {robot.total_explored}",
        f"Latest explored count: {robot.last_explored}",
        f"Planning time total: {robot.total_planning_time * 1000:.2f} ms",
        f"Discovered obstacles: {len(robot.discovered_obstacles)}",
        f"Replan events: {robot.replans}",
    ]

    for index, item in enumerate(stats):
        draw_text(item, x, section_y + 35 + index * 19, small_font)

    # Benchmark results.
    result_y = 520

    pygame.draw.line(
        screen,
        BORDER_COLOR,
        (x, result_y),
        (WINDOW_WIDTH - 15, result_y),
        2,
    )

    draw_text("BFS vs A* (first target)", x, result_y + 7, font)

    if comparison_results is None:
        draw_text(
            "Click Compare to run benchmark.",
            x,
            result_y + 30,
            small_font,
            MUTED_TEXT,
        )
    else:
        for index, name in enumerate(("BFS", "A*")):
            result = comparison_results[name]
            y = result_y + 28 + index * 36

            draw_text(
                f"{name}: "
                f"path={result['path_length']}, "
                f"explored={result['explored']}",
                x,
                y,
                small_font,
            )

            draw_text(
                f"Time={result['time_ms']:.3f} ms | "
                f"Found={result['found']}",
                x,
                y + 15,
                small_font,
            )

    return (
        bfs_button,
        astar_button,
        start_button,
        pause_button,
        reset_button,
        compare_button,
    )


# -------------------- Mission Controls --------------------

def start_mission():
    global mission_running, mission_status, last_step_time

    if mission_status in ("Completed", "Failed"):
        return

    if target_index >= len(targets):
        mission_running = False
        mission_status = "Completed"
        return

    mission_running = True
    mission_status = "Running"
    last_step_time = pygame.time.get_ticks()


def pause_mission():
    global mission_running, mission_status

    mission_running = False

    if mission_status == "Running":
        mission_status = "Paused"


def update_mission():
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

    result = robot.step_toward(targets[target_index])

    if result in ("reached", "unreachable"):
        target_index += 1

        if target_index >= len(targets):
            mission_running = False
            mission_status = (
                "Completed"
                if len(robot.visited_targets) == len(targets)
                else "Failed"
            )


def select_algorithm(name):
    global algorithm

    if mission_running:
        return

    algorithm = name
    robot.set_algorithm(name)


# -------------------- Obstacle Editing --------------------

def edit_obstacle(mouse_position):
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


# -------------------- Main Loop --------------------

def main():
    global last_step_time, comparison_results

    running = True

    while running:
        clock.tick(FPS)
        now = pygame.time.get_ticks()
        mouse_position = pygame.mouse.get_pos()

        screen.fill(BACKGROUND)
        draw_grid()

        (
            bfs_button,
            astar_button,
            start_button,
            pause_button,
            reset_button,
            compare_button,
        ) = draw_panel(mouse_position)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_r:
                    reset_environment()
                    comparison_results = None

                elif event.key == pygame.K_SPACE:
                    if mission_running:
                        pause_mission()
                    else:
                        start_mission()

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button != 1:
                    continue

                if bfs_button.collidepoint(event.pos):
                    select_algorithm("BFS")

                elif astar_button.collidepoint(event.pos):
                    select_algorithm("A*")

                elif start_button.collidepoint(event.pos):
                    start_mission()

                elif pause_button.collidepoint(event.pos):
                    pause_mission()

                elif reset_button.collidepoint(event.pos):
                    reset_environment()
                    comparison_results = None

                elif compare_button.collidepoint(event.pos):
                    comparison_results = benchmark_algorithms()

                else:
                    edit_obstacle(event.pos)

        if (
            mission_running
            and now - last_step_time >= STEP_INTERVAL_MS
        ):
            update_mission()
            last_step_time = now

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
