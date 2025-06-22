from imutils.video import VideoStream
from imutils.video import FPS
import pickle
import cv2
import face_recognition
import pygame
import imutils
import time

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
        pygame.draw.rect(surface, BLACK, self.rect, 3)  # Borda preta
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

botoes_config = []
config_buttons_created = False

# Inicializa o Pygame
pygame.init()
screen = pygame.display.set_mode((1024, 600))
pygame.display.set_caption("Olhinhos de Raposa")
font = pygame.font.SysFont(None, 36)
current_page = "menu"


# Carrega imagens da raposa
image_1r = pygame.image.load("olhos_redimensionados/1r.jpeg")
image_2r = pygame.image.load("olhos_redimensionados/2r.jpeg")
image_4r = pygame.image.load("olhos_redimensionados/4r.jpeg")
image_5r = pygame.image.load("olhos_redimensionados/5r.jpeg")

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
currentname = "unknown"

# botao
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
    frame = vs.read()
    frame = imutils.resize(frame, width=500)
    boxes = face_recognition.face_locations(frame)
    encodings = face_recognition.face_encodings(frame, boxes)
    names = []

    face_center_x = None
    if len(boxes) > 0:
        (top, right, bottom, left) = boxes[0]
        face_center_x = (left + right) // 2

    # Reconhecimento facial
    for encoding in encodings:
        matches = face_recognition.compare_faces(data["encodings"], encoding)
        

    currentname = "Unknown"  # ← RESETA o nome atual a cada frame
    for encoding in encodings:
        matches = face_recognition.compare_faces(data["encodings"], encoding)
        name = "Unknown"
        if True in matches:
            matchedIdxs = [i for (i, b) in enumerate(matches) if b]
            counts = {}
            for i in matchedIdxs:
                name = data["names"][i]
                counts[name] = counts.get(name, 0) + 1
            name = max(counts, key=counts.get)
            currentname = name  # ← agora é atualizado corretamente
            print(currentname)
        names.append(name)


    mouse_pos = pygame.mouse.get_pos()
    mouse_click = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mouse_click = True  
              

    # Desenha caixa no rosto e nome
    for ((top, right, bottom, left), name) in zip(boxes, names):
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 225), 2)
        y = top - 15 if top - 15 > 15 else top + 15
        cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 255), 2)

    # Atualiza janela OpenCV
    cv2.imshow("Facial Recognition is Running", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        running = False

    # Atualiza janela Pygame com base na posição do rosto
    current_time = pygame.time.get_ticks()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

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

    # Atualiza tela do Pygame (sempre redesenha tudo)
    screen.fill((200, 200, 200))
    img = {"1r": image_1r, "2r": image_2r, "4r": image_4r, "5r": image_5r}[new_image]
    screen.blit(img, (agent_x, agent_y))

    # Desenha o botão ou mensagem, conforme a página
    if current_page == "menu":
        if currentname != "Unknown":
            button_rect = pygame.Rect(800, 500, 200, 60)
            if draw_button("configurações", button_rect, mouse_pos, mouse_click):
                current_page = "pagina_2"
                config_buttons_created = False 
        #else:
        #    msg = font.render("Aguardando reconhecimento facial...", True, BLACK)
        #    screen.blit(msg, (150, 150))

    elif current_page == "pagina_2":
        txt = font.render("Configurações", True, BLACK)
        screen.blit(txt, (400, 50))

        if not config_buttons_created:
            botao_orelha = Botao("Orelha", 100, 100, 200, 60)
            botao_braco = Botao("Braço", 100, 200, 200, 60)
            botao_recepcionista = Botao("Recepcionista", 100, 300, 200, 60, ativo=True)  # começa ativo
            botao_limpadora = Botao("Limpadora", 100, 400, 200, 60)
            botao_tourista = Botao("Turista", 100, 500, 200, 60)
            botao_voltar = Botao("Voltar", 800, 500, 200, 60)
            botoes_config = [
                botao_orelha, botao_braco, botao_recepcionista,
                botao_limpadora, botao_tourista, botao_voltar
            ]
            config_buttons_created = True

        for botao in botoes_config:
            botao.draw(screen)

        for botao in botoes_config:
            if botao.checar_clique(mouse_pos, mouse_click):
                if botao.texto == "Voltar":
                    current_page = "menu"
                    config_buttons_created = False

                elif botao.texto == "Recepcionista":
                    botao_recepcionista.ativar()
                    botao_limpadora.desativar()
                    botao_tourista.desativar()

                elif botao.texto == "Limpadora":
                    botao_limpadora.ativar()
                    botao_recepcionista.desativar()
                    botao_tourista.desativar()

                elif botao.texto == "Turista":
                    botao_tourista.ativar()
                    botao_recepcionista.desativar()
                    botao_limpadora.desativar()
                    # current_page = "pagina_tour"

                elif botao.texto == "Orelha":
                    # Apenas alterna o botão, sem interferir nos outros
                    if botao_orelha.ativo:
                        botao_orelha.desativar()
                    else:
                        botao_orelha.ativar()

                elif botao.texto == "Braço":
                    if botao_braco.ativo:
                        botao_braco.desativar()
                    else:
                        botao_braco.ativar()


    pygame.display.flip()
    current_image = new_image

    fps.update()

# Finalização
pygame.quit()
cv2.destroyAllWindows()
vs.stop()
fps.stop()
print("[INFO] Tempo decorrido: {:.2f}".format(fps.elapsed()))
print("[INFO] FPS aproximado: {:.2f}".format(fps.fps()))
