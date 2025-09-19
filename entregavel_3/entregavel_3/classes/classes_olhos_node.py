import pygame

WHITE = (255, 255, 255)
BLUE = (50, 130, 230)
DARK_BLUE = (30, 100, 200)
BLACK = (0, 0, 0)


class Botao:
    def __init__(self, texto, x, y, largura, altura, cor=WHITE, ativo=False):
        self.texto = texto
        self.rect = pygame.Rect(x, y, largura, altura)
        self.ativo = ativo
        self.cor = BLUE if ativo else cor

    def draw(self, surface):
        pygame.draw.rect(surface, self.cor, self.rect)
        pygame.draw.rect(surface, BLACK, self.rect, 3)
        font = pygame.font.SysFont(None, 36)
        txt_surf = font.render(self.texto, True, BLACK)
        txt_rect = txt_surf.get_rect(center=self.rect.center)
        surface.blit(txt_surf, txt_rect)

    def checar_clique(self, mouse_pos, mouse_click):
        return self.rect.collidepoint(mouse_pos) and mouse_click

    def ativar(self):
        self.ativo = True
        self.cor = BLUE

    def desativar(self):
        self.ativo = False
        self.cor = WHITE

class InputBox:
    def __init__(self, x, y, w, h, text=''):
        self.rect = pygame.Rect(x, y, w, h)
        self.color = pygame.Color('white')
        self.text = text
        self.txt_surface = pygame.font.SysFont(None, 36).render(text, True, BLACK)
        self.active = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.active = not self.active
            else:
                self.active = False
        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN:
                return self.text
            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            else:
                if len(self.text) < 20:
                    self.text += event.unicode
            self.txt_surface = pygame.font.SysFont(None, 36).render(self.text, True, BLACK)
        return None

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.rect)             # Preenche a caixa
        pygame.draw.rect(screen, BLACK, self.rect, 2)      
        #pygame.draw.rect(screen, self.color, self.rect, 2)
        screen.blit(self.txt_surface, (self.rect.x + 5, self.rect.y + 5))