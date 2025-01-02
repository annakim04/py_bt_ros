import pygame

class BaseTask:
    def __init__(self, task_id, position):
        self.task_id = task_id
        self.position = pygame.Vector2(position)
        self.completed = False

    def set_done(self):
        self.completed = True

    def draw(self, screen):
        if not self.completed:
            pygame.draw.circle(screen, self.color, self.position, int(5))

    def draw_task_id(self, screen):
        if not self.completed:
            font = pygame.font.Font(None, 15)
            text_surface = font.render(f"task_id {self.task_id}", True, (250, 250, 250))
            screen.blit(text_surface, (self.position[0], self.position[1]))

