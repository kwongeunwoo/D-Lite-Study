import heapq
import pygame

from graph import Node, Graph
# from grid import GridWorld
from utils import stateNameToCoords
from d_star_lite import initDStarLite, moveAndRescan

# 색상 정의
BLACK = (20, 20, 20)
WHITE = (235, 235, 235)
GREEN = (63, 186, 1)
GRAY1 = (113, 112, 117)
GRAY2 = (29, 30, 31)
BLUE = (8, 172, 201)
LIGHT_BLUE = (173, 216, 230)

colors = {
    0: WHITE,
    1: GREEN,
    -1: GRAY1,
    -2: GRAY2
}
colors['expanded'] = LIGHT_BLUE  # 확장된 노드의 색상을 라이트 블루로 설정

# 각 그리드 위치의 너비와 높이 설정
WIDTH = 90
HEIGHT = 80

# 각 셀 사이의 마진 설정
MARGIN = 3

# pygame 초기화
pygame.init()

X_DIM = 10
Y_DIM = 10
VIEWING_RANGE = 2

# 화면의 너비와 높이 설정, 추가된 공간을 고려하여 너비를 확장
WINDOW_SIZE = [(WIDTH + MARGIN) * X_DIM + MARGIN + 400,  # 추가된 공간으로 U의 정보 표시 공간 확보
               (HEIGHT + MARGIN) * Y_DIM + MARGIN]
screen = pygame.display.set_mode(WINDOW_SIZE)

# 우선순위 큐(U)의 내용을 화면에 그리는 함수
def draw_priority_queue(screen, queue, font, offset_x, offset_y):
    x = (WIDTH + MARGIN) * X_DIM + MARGIN + offset_x  # U의 정보를 그리기 시작할 x 위치
    line_height = 20  # 각 항목의 높이 설정
    for i, (key1, comp, coords) in enumerate(queue):
        x_coord, y_coord = stateNameToCoords(coords)  # 상태 이름을 좌표로 변환
        coords_formatted = f'({x_coord},{y_coord})'  # 좌표 포맷팅
        text = f'{coords_formatted}: k1={key1:.3f}, k2={comp:.3f}'  # 텍스트 포맷팅
        text_surface = font.render(text, True, WHITE)  # 텍스트 렌더링
        screen.blit(text_surface, (x, offset_y + i * line_height))  # 텍스트 화면에 그리기

# 화면 타이틀 설정
pygame.display.set_caption("D* Lite Path Planning")

# 사용자가 닫기 버튼을 클릭할 때까지 루프
done = False

goal_set = False

# 화면 업데이트 속도 관리를 위한 clock 객체 생성
clock = pygame.time.Clock()

if __name__ == "__main__":
    graph = Graph(X_DIM, Y_DIM)
    s_start = 'x1y2'  # 시작 위치 설정
    s_current = s_start  # 현재 위치를 시작 위치로 초기화
    pos_coords = stateNameToCoords(s_current)  # 현재 위치의 좌표를 얻음

    # 화면 배경을 검은색으로 설정
    screen.fill(BLACK)
    # 그리드 그리기
    for row in range(Y_DIM):
        for column in range(X_DIM):
            color = WHITE  # 기본 색상을 흰색으로 설정
            pygame.draw.rect(screen, color,
                             [(MARGIN + WIDTH) * column + MARGIN,
                              (MARGIN + HEIGHT) * row + MARGIN, WIDTH, HEIGHT])
            node_name = 'x' + str(column) + 'y' + str(row)
    # 이동하는 로봇을 현재 위치에 기반하여 그림
    robot_center = [int(pos_coords[0] * (WIDTH + MARGIN) + WIDTH / 2) +
                    MARGIN, int(pos_coords[1] * (HEIGHT + MARGIN) + HEIGHT / 2) + MARGIN]
    pygame.draw.circle(screen, BLUE, robot_center, int(WIDTH / 2) - 2)
    # 로봇의 시야 범위 그리기
    pygame.draw.rect(
        screen, GRAY2, [robot_center[0] - VIEWING_RANGE * (WIDTH + MARGIN), robot_center[1] - VIEWING_RANGE * (HEIGHT + MARGIN), 2 * VIEWING_RANGE * (WIDTH + MARGIN), 2 * VIEWING_RANGE * (HEIGHT + MARGIN)], 3)
    # 그려진 내용을 화면에 업데이트
    pygame.display.flip()

    # 목표 위치 설정을 위한 사용자 입력 대기
    print('Click to choose goal location')
    while not goal_set and not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # 사용자가 닫기 버튼 클릭
                done = True
            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                column = pos[0] // (WIDTH + MARGIN)
                row = pos[1] // (HEIGHT + MARGIN)
                if(graph.cells[row][column] == 0):
                    graph.cells[row][column] = 1
                    goal_set = True
                    s_goal = 'x' + str(column) + 'y' + str(row)
                    goal_coords = stateNameToCoords(s_goal)
    # 목표 위치를 초록색으로 채움
    pygame.draw.rect(screen, GREEN, [(MARGIN + WIDTH) * goal_coords[0] + MARGIN,
                                     (MARGIN + HEIGHT) * goal_coords[1] + MARGIN, WIDTH, HEIGHT])
    pygame.display.flip()

    graph.setStart(s_start)  # 그래프의 시작 위치 설정
    graph.setGoal(s_goal)  # 그래프의 목표 위치 설정
    k_m = 0  # 이동 비용 변경을 추적하기 위한 변수
    queue = []  # 우선순위 큐 초기화

    graph, queue, k_m = initDStarLite(graph, queue, s_start, s_goal, k_m)  # D* Lite 알고리즘 초기화

    basicfont = pygame.font.SysFont('avenirnext', 20)  # 텍스트 폰트 설정
    print('Click to create obstacles or press spacebar to iterate through simulation')

    # 메인 프로그램 루프
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                s_new, k_m = moveAndRescan(graph, queue, s_current, VIEWING_RANGE, k_m)  # 다음 위치로 이동 및 환경 스캔
                if s_new == 'goal':
                    print('Goal Reached!')  # 목표 도달 시 메시지 출력
                    done = True
                else:
                    s_current = s_new  # 현재 위치 업데이트
                    pos_coords = stateNameToCoords(s_current)  # 새로운 현재 위치의 좌표 얻기

            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                column = pos[0] // (WIDTH + MARGIN)
                row = pos[1] // (HEIGHT + MARGIN)
                if(graph.cells[row][column] == 0):
                    graph.cells[row][column] = -1  # 장애물 추가

        # 화면 배경을 검은색으로 재설정
        screen.fill(BLACK)

        # 그리드를 다시 그림
        for row in range(Y_DIM):
            for column in range(X_DIM):
                node_name = f'x{column}y{row}'
                node = graph.graph[node_name]

                # 노드 상태에 따라 색상 결정
                if node.g != float('inf') and node.g == node.rhs and graph.cells[row][column] >= 0:  # 확장된 상태이며 장애물이 아닌 경우
                    color = colors['expanded']
                else:  # 기본 셀 색상 또는 장애물
                    color = colors[graph.cells[row][column]]
                
                pygame.draw.rect(screen, color,
                                [(MARGIN + WIDTH) * column + MARGIN,
                                (MARGIN + HEIGHT) * row + MARGIN, WIDTH, HEIGHT])

                # g값과 rhs값에 대한 텍스트를 생성하고 화면에 표시
                g_value_text = f'g: {"inf" if node.g == float("inf") else round(node.g, 1)}'
                rhs_value_text = f'rhs: {"inf" if node.rhs == float("inf") else round(node.rhs, 1)}'
                g_value_rendered = basicfont.render(g_value_text, True, (0, 0, 0))
                g_value_rect = g_value_rendered.get_rect()
                g_value_rect.topleft = ((MARGIN + WIDTH) * column + MARGIN, (MARGIN + HEIGHT) * row + MARGIN)
                screen.blit(g_value_rendered, g_value_rect)
                rhs_value_rendered = basicfont.render(rhs_value_text, True, (0, 0, 0))
                rhs_value_rect = rhs_value_rendered.get_rect()
                rhs_value_rect.topleft = ((MARGIN + WIDTH) * column + MARGIN, g_value_rect.bottom)
                screen.blit(rhs_value_rendered, rhs_value_rect)

                # 좌표 정보를 표시하는 텍스트를 생성하고 화면에 표시
                coord_text = f'({column},{row})'
                coord_rendered = basicfont.render(coord_text, True, (0, 0, 0))
                coord_rect = coord_rendered.get_rect()
                coord_rect.topleft = ((MARGIN + WIDTH) * column + MARGIN, rhs_value_rect.bottom)
                screen.blit(coord_rendered, coord_rect)

        # 목표 위치를 초록색으로 다시 채움
        pygame.draw.rect(screen, GREEN, [(MARGIN + WIDTH) * goal_coords[0] + MARGIN,
                                         (MARGIN + HEIGHT) * goal_coords[1] + MARGIN, WIDTH, HEIGHT])
        # 이동하는 로봇을 현재 위치에 기반하여 다시 그림
        robot_center = [int(pos_coords[0] * (WIDTH + MARGIN) + WIDTH / 2) +
                        MARGIN, int(pos_coords[1] * (HEIGHT + MARGIN) + HEIGHT / 2) + MARGIN]
        pygame.draw.circle(screen, BLUE, robot_center, int(WIDTH / 2) - 2)

        # 로봇의 시야 범위를 다시 그림
        pygame.draw.rect(
            screen, GRAY2, [robot_center[0] - VIEWING_RANGE * (WIDTH + MARGIN), robot_center[1] - VIEWING_RANGE * (HEIGHT + MARGIN), 2 * VIEWING_RANGE * (WIDTH + MARGIN), 2 * VIEWING_RANGE * (HEIGHT + MARGIN)], 3)

        # 우선순위 큐(U)의 내용을 화면 오른쪽에 그림
        draw_priority_queue(screen, queue, basicfont, 10, 10)

        # 화면 업데이트
        pygame.display.flip()

        # 프레임 속도 조절
        clock.tick(20)

    # 프로그램 종료 시 정리 작업
    pygame.quit()
