
from imutils.video import VideoStream
from imutils.video import FPS
import pickle
import cv2
import face_recognition
import pygame
import imutils
import time
import os
import subprocess

# Cores usadas
WHITE = (255, 255, 255)
BLUE = (50, 130, 230)
DARK_BLUE = (30, 100, 200)
BLACK = (0, 0, 0)

# --- Ajuste de caminhos: define diretório base do script e subpasta de imagens ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR = os.path.join(BASE_DIR, "olhos_redimensionados")

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
        pygame.draw.rect(screen, self.color, self.rect, 2)
        screen.blit(self.txt_surface, (self.rect.x + 5, self.rect.y + 5))

# Inicializa o Pygame
game_direction = pygame.init()
screen = pygame.display.set_mode((1024, 600))
pygame.display.set_caption("Olhinhos de Raposa")
font = pygame.font.SysFont(None, 36)
current_page = "menu"

# Carregamento de imagens:
# Versão original comentada:
# image_1r = pygame.image.load("olhos_redimensionados/1r.jpeg")
# image_2r = pygame.image.load("olhos_redimensionados/2r.jpeg")
# image_4r = pygame.image.load("olhos_redimensionados/4r.jpeg")
# image_5r = pygame.image.load("olhos_redimensionados/5r.jpeg")
# Nova versão (usa caminho absoluto baseado no script):
image_1r = pygame.image.load(os.path.join(IMAGES_DIR, "1r.jpeg"))
image_2r = pygame.image.load(os.path.join(IMAGES_DIR, "2r.jpeg"))
image_4r = pygame.image.load(os.path.join(IMAGES_DIR, "4r.jpeg"))
image_5r = pygame.image.load(os.path.join(IMAGES_DIR, "5r.jpeg"))

agent_x = 0
agent_y = 0
start_time = pygame.time.get_ticks()
image_changed_time = None
current_image = "5r"

# Inicializa reconhecimento facial
print("[INFO] carregando encodings...")
data = pickle.loads(open("encodings.pickle", "rb").read())
vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()
currentname = "Unknown"

# Função de desenho de botão simples
def draw_button(text, rect, mouse_pos, mouse_click):
    if rect.collidepoint(mouse_pos):
        color = DARK_BLUE
        if mouse_click:
            return True
    else:
        color = BLUE
    pygame.draw.rect(screen, color, rect)
    txt_surf = font.render(text, True, WHITE)
    txt_rect = txt_surf.get_rect(center=rect.center)
    screen.blit(txt_surf, txt_rect)
    return False

running = True
while running:
    eventos = pygame.event.get()
    mouse_click = any(event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 for event in eventos)

    frame = vs.read()
    frame = imutils.resize(frame, width=500)
    boxes = face_recognition.face_locations(frame)
    encodings = face_recognition.face_encodings(frame, boxes)
    names = []

    # Versão original de exibição pelo OpenCV (comentada):
    # for ((top, right, bottom, left), name) in zip(boxes, names):
    #     cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 225), 2)
    #     y = top - 15 if top - 15 > 15 else top + 15
    #     cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
    #                 0.8, (0, 255, 255), 2)
    # cv2.imshow("Facial Recognition is Running", frame)
    # key = cv2.waitKey(1) & 0xFF
    # if key == ord("q"):
    #     running = False

    # Atualiza Pygame com base na posição do rosto
    face_center_x = None
    if len(boxes) > 0:
        top, right, bottom, left = boxes[0]
        face_center_x = (left + right) // 2

    current_time = pygame.time.get_ticks()
    if face_center_x is None:
        new_image = "5r"
    elif face_center_x > 350:
        new_image = "1r"
    elif face_center_x < 150:
        new_image = "2r"
    else:
        if current_image not in ["4r", "5r"] or image_changed_time is None:
            new_image = "5r"
            if current_time - start_time >= 5000:
                new_image = "4r"
                image_changed_time = current_time
        elif current_image == "4r" and current_time - image_changed_time >= 1000:
            new_image = "5r"
            image_changed_time = None
            start_time = pygame.time.get_ticks()
        else:
            new_image = current_image

    # Desenha no Pygame
    screen.fill((200, 200, 200))
    img = {"1r": image_1r, "2r": image_2r, "4r": image_4r, "5r": image_5r}[new_image]
    screen.blit(img, (agent_x, agent_y))

    if current_page == "menu":
        if currentname != "Unknown":
            button_rect = pygame.Rect(800, 500, 200, 60)
            if draw_button("configurações", button_rect, pygame.mouse.get_pos(), mouse_click):
                current_page = "pagina_2"
                config_buttons_created = False

    # ... restante da lógica de páginas, botões e fluxo de cadastro ...

    pygame.display.flip()
    current_image = new_image
    fps.update()

# Finalização\pygame.quit()
cv2.destroyAllWindows()
vs.stop()
fps.stop()
print(f"[INFO] Tempo decorrido: {fps.elapsed():.2f}")
print(f"[INFO] FPS aproximado: {fps.fps():.2f}")
