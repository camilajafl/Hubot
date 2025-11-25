import os
import pickle
import rclpy
import pygame
import cv2
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_srvs.srv import Empty
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


#pinos SERVOS   
import RPi.GPIO as GPIO
import time
import threading
GPIO.setmode(GPIO.BCM)
pins = {
    "orelha_esq": 21,
    "orelha_dir": 20,
    "braco_esq": 16,
    "braco_dir": 12
    }

servo_angles = {}   # novo dicionário GLOBAL para armazenar ângulos de cada servo

servos = {}
for nome, pin in pins.items():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)  # 50 Hz p/ servo

    inicial = 90
    duty = 2.5 + (inicial * 10.0 / 180.0)
    pwm.start(duty)

    servos[nome] = pwm
    servo_angles[nome] = inicial   # ← Armazena aqui, NÃO no objeto PWM



WHITE = (255, 255, 255)
BLUE = (50, 130, 230)
DARK_BLUE = (30, 100, 200)
BLACK = (0, 0, 0)
RED = (190, 40, 40)
GREEN = (30, 150, 70)

class Botao:
    def __init__(self, texto, x, y, largura, altura, cor=WHITE, ativo=False):
        self.texto = texto
        self.rect = pygame.Rect(x, y, largura, altura)
        self.ativo = ativo
        self.cor = BLUE if ativo else cor
        self.pressed_until = 0  # feedback visual breve após clique

    def draw(self, surface):
        now = pygame.time.get_ticks()
        # cor base
        base_color = self.cor
        # se estiver no "flash" de clique, escurece temporariamente
        if now < self.pressed_until:
            draw_color = DARK_BLUE
        else:
            draw_color = base_color
        pygame.draw.rect(surface, draw_color, self.rect)
        pygame.draw.rect(surface, BLACK, self.rect, 3)
        font = pygame.font.SysFont(None, 36)
        txt_surf = font.render(self.texto, True, BLACK)
        txt_rect = txt_surf.get_rect(center=self.rect.center)
        surface.blit(txt_surf, txt_rect)

    def checar_clique(self, mouse_pos, mouse_click):
        clicked = self.rect.collidepoint(mouse_pos) and mouse_click
        if clicked:
            # ativa flash de clique por 150ms
            self.pressed_until = pygame.time.get_ticks() + 250
        return clicked

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
        pygame.draw.rect(screen, self.color, self.rect)
        pygame.draw.rect(screen, BLACK, self.rect, 2)
        screen.blit(self.txt_surface, (self.rect.x + 5, self.rect.y + 5))

class OlhosNode(Node):
    def __init__(self):
        super().__init__('olhos_node')

        # --- estado geral ---
        self.username = "Unknown"
        self.current_page = "menu"
        self.config_buttons_created = False
        self.avancado_buttons_created = False
        self.cadastro_buttons_created = False
        self.botao_voltar_cadastro = False

        #servos
        self.botao_orelha_ativo = False
        self.botao_braco_ativo = False


        # painel de cadastros / edição
        self.encodings_path = os.path.expanduser('~/.ros/encodings.pickle')
        self.painel_buttons_created = False
        self.painel_names = []      # nomes únicos
        self.painel_scroll = 0      # paginação simples
        self.confirm_delete_name = None
        self.editing_name = None
        self.edit_input = None

        # cliente para recarregar encodings (mesmo do EnrollNode)
        self.reload_cli = self.create_client(Empty, 'reload_encodings')

        # live preview da câmera
        self.bridge = CvBridge()
        self.last_frame_bgr = None
        self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._img_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # status do cadastro (texto do EnrollNode)
        self.last_status_text = ""
        self.last_status_color = BLACK
        self.status_show_until_ms = 0
        self.captura_em_andamento = False  # controla ciclo de envio/callback
        self.create_subscription(String, 'enroll_status', self._status_cb, 10)

        self.twist = Twist()

        pygame.font.init()
        self.font = pygame.font.SysFont(None, 36)
        self.font_small = pygame.font.SysFont(None, 28)

        # modo de estado
        self.botao_recepcionista_ativo = True
        self.botao_limpadora_ativo = False
        self.botao_tourista_ativo = False

        # SUBSCRIBERS
        self.sub_user = self.create_subscription(String, 'recognized_user', self.cb_user, 10)
        self.sub_face_center = self.create_subscription(
            String,
            'primeiro_face_location',
            self.cb_face_center,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # PUBLISHERS
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_captura = self.create_publisher(String, 'captura_nome', 10)

        # Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((1024, 600), pygame.FULLSCREEN)
        pygame.display.set_caption("Olhinhos de Raposa")

        # imagens
        this_dir = os.path.dirname(__file__)
        folder   = os.path.join(this_dir, 'olhos_redimensionados')
        self.images = {
            'left':    pygame.image.load(os.path.join(folder, '1r.jpeg')),
            'right':   pygame.image.load(os.path.join(folder, '2r.jpeg')),
            'forward': pygame.image.load(os.path.join(folder, '5r.jpeg')),
            'blink':   pygame.image.load(os.path.join(folder, '4r.jpeg')),
        }
        self.current_image = 'forward'
        self.current_direction = 'forward'

        # timers de piscar
        self.blink_interval    = 5000
        self.blink_duration    = 1000
        self.last_blink_time   = pygame.time.get_ticks()
        self.blinking          = False
        self.blink_start_time  = None

        self._draw(self.current_direction)

    # util
    def _draw(self, key):
        self.screen.fill((200,200,200))
        self.screen.blit(self.images[key], (0,0))

    def angulo_para_duty(self, angulo):
        # mapeia 0..180 graus para duty (~2.5 .. 12.5). Ajuste se seu servo precisar.
        return 2.5 + (angulo * 10.0 / 180.0)

    def mover_servo_suave(self, pwm, angulo_final, passos=30, dt=0.03):
        """Move o servo do pwm suavemente até angulo_final.
           Usa pwm.last_angle quando disponível; mantém o último duty (não zera).
        """
        # ângulo inicial = atributo pwm.last_angle se existir, senão 90°
        angulo_inicial = getattr(pwm, 'last_angle', 90.0)
        atual = float(angulo_inicial)

        passos = max(2, int(passos))
        delta = (float(angulo_final) - atual) / passos

        for i in range(1, passos + 1):
            ang = atual + delta * i
            duty = self.angulo_para_duty(ang)
            pwm.ChangeDutyCycle(duty)
            pwm.last_angle = ang
            time.sleep(dt)

        # garante exato final (sem zerar o PWM)
        pwm.last_angle = float(angulo_final)
        pwm.ChangeDutyCycle(self.angulo_para_duty(pwm.last_angle))

    def animar_orelhas(self):
        while self.botao_orelha_ativo:
            self.mover_servo_suave(servos["orelha_esq"], 0)
            self.mover_servo_suave(servos["orelha_dir"], 90)
            time.sleep(0.1)
            if not self.botao_orelha_ativo:
                break
            self.mover_servo_suave(servos["orelha_esq"], 90)
            self.mover_servo_suave(servos["orelha_dir"], 0)
            time.sleep(0.1)
        # mantém a última posição (não zera) — evita tremores

    def animar_bracos(self):
        while self.botao_braco_ativo:
            self.mover_servo_suave(servos["braco_esq"], 0)
            self.mover_servo_suave(servos["braco_dir"], 90)
            time.sleep(0.1)
            if not self.botao_braco_ativo:
                break
            self.mover_servo_suave(servos["braco_esq"], 90)
            self.mover_servo_suave(servos["braco_dir"], 0)
            time.sleep(0.1)
        # mantém última posição


    def _draw_button_simple(self, text, x, y, w=200, h=60, mouse_pos=None, mouse_click=False):
        rect = pygame.Rect(x, y, w, h)
        # feedback de clique/hover para botões "simples"
        color = BLUE
        if mouse_pos is not None and rect.collidepoint(mouse_pos):
            color = DARK_BLUE if mouse_click else BLUE
        pygame.draw.rect(self.screen, color, rect)
        pygame.draw.rect(self.screen, BLACK, rect, 3)
        txt_surf = self.font.render(text, True, WHITE)
        txt_rect = txt_surf.get_rect(center=rect.center)
        self.screen.blit(txt_surf, txt_rect)
        return rect

    # administrador
    def cb_user(self, msg: String):
        self.username = msg.data

    def cb_face_center(self, msg: String):
        if self.botao_limpadora_ativo:
            return
        direction_map = {"esquerda": "left", "direita": "right", "centralizado": "forward"}
        ros_direction = msg.data
        new_direction = direction_map.get(ros_direction, "forward")
        if new_direction != self.current_direction:
            self.current_direction = new_direction
            self.current_image = new_direction
            self.blinking = False
            self.last_blink_time = pygame.time.get_ticks()
        if ros_direction == "esquerda":
            self.twist.angular.z = -0.3
        elif ros_direction == "direita":
            self.twist.angular.z = 0.3
        else:
            self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def update_image(self):
        self.current_image = self.current_direction

    # ---------- Callbacks auxiliares ----------
    def _img_cb(self, msg: CompressedImage):
        try:
            self.last_frame_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            self.last_frame_bgr = None

    def _status_cb(self, msg: String):
        # guarda texto de status por 5s e limpa placeholder de "Enviando..."
        txt = msg.data or ""
        color = BLACK
        if "Falha" in txt or "erro" in txt.lower():
            color = RED
            self.captura_em_andamento = False
        elif "cadastrado" in txt.lower():
            color = GREEN
            self.captura_em_andamento = False
        elif "OK" in txt:
            color = GREEN
        self.last_status_text = txt
        self.last_status_color = color
        self.status_show_until_ms = pygame.time.get_ticks() + 5000
        # ao receber qualquer status, some com o placeholder
        self.erro = ""

    # ---------- painel de cadastros helpers ----------
    def _read_pickle(self):
        if os.path.exists(self.encodings_path):
            with open(self.encodings_path, 'rb') as f:
                return pickle.load(f)
        return {"encodings": [], "names": []}

    def _write_pickle(self, data):
        tmp = self.encodings_path + '.tmp'
        with open(tmp, 'wb') as f:
            pickle.dump(data, f)
        os.replace(tmp, self.encodings_path)

    def _load_unique_names(self):
        data = self._read_pickle()
        self.painel_names = sorted(list(dict.fromkeys(data.get('names', []))))
        if self.painel_scroll >= len(self.painel_names):
            self.painel_scroll = 0

    def _reload_encodings_service(self):
        if self.reload_cli.wait_for_service(timeout_sec=0.5):
            fut = self.reload_cli.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.5)

    def _delete_name(self, name):
        data = self._read_pickle()
        names = data.get('names', [])
        encs  = data.get('encodings', [])
        keep_idx = [i for i,n in enumerate(names) if n != name]
        data['names'] = [names[i] for i in keep_idx]
        data['encodings'] = [encs[i] for i in keep_idx]
        self._write_pickle(data)
        self._reload_encodings_service()
        self._load_unique_names()

    def _rename_name(self, old, new):
        if not new or new.strip()=="":
            return
        data = self._read_pickle()
        names = data.get('names', [])
        data['names'] = [new if n==old else n for n in names]
        self._write_pickle(data)
        self._reload_encodings_service()
        self._load_unique_names()

    # ---------- run loop ----------
    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                eventos = pygame.event.get()
                mouse_click = False
                mouse_pos = pygame.mouse.get_pos()
                for event in eventos:
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        return
                    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                        mouse_click = True

                self.update_image()

                # --------------- MENU PRINCIPAL ---------------
                if self.current_page == "menu":
                    # desenha olho + botão de configurações
                    self._draw(self.current_direction)
                    if self.username != "Unknown":
                        button_rect = pygame.Rect(800, 500, 200, 60)
                        if self._draw_button("Configuraçõesss", button_rect, mouse_pos, mouse_click):
                            self.current_page = "pagina_2"
                            self.config_buttons_created = False
                            # ao sair do menu, garante que não pisque mais
                            self.blinking = False

                    # piscar APENAS no menu (impede flicker nas páginas de configuração)
                    if not self.blinking:
                        now = pygame.time.get_ticks()
                        if now - self.last_blink_time >= self.blink_interval:
                            self.blinking = True
                            self.blink_start_time = now
                            self._draw('blink')
                    else:
                        now = pygame.time.get_ticks()
                        if now - self.blink_start_time >= self.blink_duration:
                            self.blinking = False
                            self.last_blink_time = now
                            self._draw(self.current_direction)

                # --------------- CONFIGURAÇÕES (pagina_2) ---------------
                elif self.current_page == "pagina_2":
                    # fundo estável (sem piscar, sem olhos)
                    self.screen.fill(WHITE)
                    txt = self.font.render("Configurações", True, BLACK)
                    self.screen.blit(txt, (350, 50))

                    if not self.config_buttons_created:
                        self.botao_orelha = Botao("Orelha", 100, 100, 200, 60)
                        self.botao_braco = Botao("Braço", 100, 200, 200, 60)
                        self.botao_recepcionista = Botao("Recepcionista", 100, 300, 200, 60, ativo=False)
                        self.botao_limpadora = Botao("Limpadora", 100, 400, 200, 60)
                        self.botao_tourista = Botao("Turista", 100, 500, 200, 60)
                        self.botao_voltar = Botao("Voltar", 800, 500, 200, 60)
                        self.botao_cadastro = Botao("Cadastro", 800, 400, 200, 60)
                        self.botoes_config = [
                            self.botao_orelha, self.botao_braco, self.botao_recepcionista,
                            self.botao_limpadora, self.botao_tourista, self.botao_voltar, self.botao_cadastro
                        ]
                        self.config_buttons_created = True

                    for botao in self.botoes_config:
                        clicked = botao.checar_clique(mouse_pos, mouse_click)
                        if clicked:
                            if botao.texto == "Voltar":
                                self.current_page = "menu"
                                self.config_buttons_created = False
                                self.blinking = False  # garante sem piscar ao entrar
                            elif botao.texto == "Recepcionista":
                                self.botao_recepcionista.ativar(); self.botao_limpadora.desativar(); self.botao_tourista.desativar()
                                self.botao_recepcionista_ativo = True; self.botao_limpadora_ativo = False; self.botao_tourista_ativo = False
                            elif botao.texto == "Limpadora":
                                self.botao_limpadora.ativar(); self.botao_recepcionista.desativar(); self.botao_tourista.desativar()
                                self.botao_recepcionista_ativo = False; self.botao_limpadora_ativo = True; self.botao_tourista_ativo = False
                            elif botao.texto == "Turista":
                                self.botao_tourista.ativar(); self.botao_recepcionista.desativar(); self.botao_limpadora.desativar()
                                self.botao_recepcionista_ativo = False; self.botao_limpadora_ativo = False; self.botao_tourista_ativo = True
                            
                            #SERVO
                            elif botao.texto == "Orelha":
                                if self.botao_orelha.ativo:
                                    # DESLIGA
                                    self.botao_orelha.desativar()
                                    self.botao_orelha_ativo = False
                                else:
                                    # LIGA
                                    self.botao_orelha.ativar()
                                    self.botao_orelha_ativo = True
                                    threading.Thread(target=self.animar_orelhas, daemon=True).start()

    
                            elif botao.texto == "Braço":
                                if self.botao_braco.ativo:
                                    self.botao_braco.desativar()
                                    self.botao_braco_ativo = False
                                else:
                                    self.botao_braco.ativar()
                                    self.botao_braco_ativo = True
                                    threading.Thread(target=self.animar_bracos, daemon=True).start()


                            elif botao.texto == "Cadastro":
                                self.current_page = "Cadastro"
                                # zera estados para não sobrepor telas
                                self.avancado_buttons_created = False
                                self.cadastro_buttons_created = False
                                self.painel_buttons_created = False
                                self.blinking = False

                    for botao in self.botoes_config:
                        botao.draw(self.screen)

                # --------------- CADASTRO ---------------
                elif self.current_page == "Cadastro":
                    # limpa fundo para não sobrepor elementos da tela anterior
                    self.screen.fill(WHITE)
                    txt = self.font.render("Configurações avançadas", True, BLACK)
                    self.screen.blit(txt, (300, 40))

                    if not self.avancado_buttons_created:
                        self.botao_novo = Botao("Novo cadastro", 100, 120, 260, 60)
                        self.botao_painel = Botao("Painel de cadastros", 100, 200, 260, 60)
                        self.botao_voltar = Botao("Voltar", 800, 500, 200, 60)
                        self.botoes_avancado = [self.botao_novo, self.botao_painel, self.botao_voltar]
                        self.avancado_buttons_created = True

                    for botao in self.botoes_avancado:
                        clicked = botao.checar_clique(mouse_pos, mouse_click)
                        if clicked:
                            if botao.texto == "Voltar":
                                self.current_page = "pagina_2"
                                self.avancado_buttons_created = False
                                self.config_buttons_created = False
                                self.blinking = False
                            elif botao.texto == "Novo cadastro":
                                self.current_page = "pagina_novo_cadastro"
                                self.avancado_buttons_created = False
                                self.cadastro_buttons_created = False
                                self.blinking = False
                            elif botao.texto == "Painel de cadastros":
                                # carrega nomes e vai para o painel
                                self._load_unique_names()
                                self.painel_scroll = 0
                                self.current_page = "painel_cadastros"
                                self.painel_buttons_created = False
                                self.blinking = False

                    for botao in self.botoes_avancado:
                        botao.draw(self.screen)

                # --------------- NOVO CADASTRO ---------------
                elif self.current_page == "pagina_novo_cadastro":
                    self.screen.fill(WHITE)
                    if not self.cadastro_buttons_created:
                        self.botao_voltar_cadastro = Botao("Voltar", 800, 500, 200, 60)
                        self.botao_capturar = Botao("Tirar Foto", 100, 250, 200, 50)
                        self.input_box = InputBox(100, 100, 300, 40)
                        self.nome_digitado = ""
                        self.erro = ""
                        self.cadastro_buttons_created = True
                    txt = self.font.render("Digite o nome:", True, BLACK)
                    self.screen.blit(txt, (100, 60))
                    for event in eventos:
                        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                            return
                        else:
                            resultado = self.input_box.handle_event(event)
                            if resultado:
                                self.nome_digitado = resultado
                    self.input_box.draw(self.screen)
                    nome = self.input_box.text.strip()
                    nome_txt = self.font.render(f"Nome: {nome}", True, BLACK)
                    self.screen.blit(nome_txt, (100, 150))

                    # --- PREVIEW AO VIVO acima do botão Voltar ---
                    preview_rect = pygame.Rect(680, 240, 320, 240)  # x,y,w,h (fica acima do Voltar em 800,500)
                    pygame.draw.rect(self.screen, (235,235,235), preview_rect)
                    pygame.draw.rect(self.screen, BLACK, preview_rect, 2)
                    if self.last_frame_bgr is not None:
                        # BGR -> RGB, resize, blit
                        rgb = cv2.cvtColor(self.last_frame_bgr, cv2.COLOR_BGR2RGB)
                        frame_resized = cv2.resize(rgb, (preview_rect.w-4, preview_rect.h-4))
                        surf = pygame.image.frombuffer(frame_resized.tobytes(), (frame_resized.shape[1], frame_resized.shape[0]), 'RGB')
                        self.screen.blit(surf, (preview_rect.x+2, preview_rect.y+2))
                    label = self.font_small.render("Pré-visualização", True, BLACK)
                    self.screen.blit(label, (preview_rect.x, preview_rect.y-24))

                    # botões
                    # processa cliques antes de desenhar (feedback imediato)
                    cap_clicked = self.botao_capturar.checar_clique(mouse_pos, mouse_click)
                    back_clicked = self.botao_voltar_cadastro.checar_clique(mouse_pos, mouse_click)

                    # status de cadastro (mensagem do EnrollNode)
                    now_ms = pygame.time.get_ticks()
                    if self.last_status_text and now_ms <= self.status_show_until_ms:
                        status_surf = self.font_small.render(self.last_status_text, True, self.last_status_color)
                        self.screen.blit(status_surf, (100, 320))
                    elif self.captura_em_andamento:
                        # se ainda não chegou status novo mas o placeholder expirou, mantém mensagem neutra curta
                        hold_txt = "Aguardando amostras..."
                        status_surf = self.font_small.render(hold_txt, True, BLACK)
                        self.screen.blit(status_surf, (100, 320))

                    if cap_clicked:
                        if not nome:
                            self.erro = "Digite um nome válido."
                            self.last_status_text = self.erro
                            self.last_status_color = RED
                            self.status_show_until_ms = pygame.time.get_ticks() + 3000
                            self.captura_em_andamento = False
                        else:
                            self.erro = "Enviando pedido de cadastro..."
                            self.pub_captura.publish(String(data=nome))
                            self.last_status_text = self.erro
                            self.last_status_color = BLACK
                            self.status_show_until_ms = pygame.time.get_ticks() + 2000
                            self.captura_em_andamento = True

                    if back_clicked:
                        self.current_page = "Cadastro"
                        self.cadastro_buttons_created = False

                    # desenha botões após processar clique (mostra flash imediatamente)
                    self.botao_capturar.draw(self.screen)
                    self.botao_voltar_cadastro.draw(self.screen)

                    if self.erro and not (self.last_status_text and now_ms <= self.status_show_until_ms):
                        erro_txt = self.font_small.render(self.erro, True, RED)
                        self.screen.blit(erro_txt, (100, 350))

                # --------------- PAINEL DE CADASTROS ---------------
                elif self.current_page == "painel_cadastros":
                    self.screen.fill(WHITE)
                    title = self.font.render("Painel de cadastros", True, BLACK)
                    self.screen.blit(title, (320, 30))

                    btn_voltar = self._draw_button_simple("Voltar", 800, 500, mouse_pos=mouse_pos, mouse_click=mouse_click)
                    btn_prev   = self._draw_button_simple("<", 100, 500, 80, 50, mouse_pos=mouse_pos, mouse_click=mouse_click)
                    btn_next   = self._draw_button_simple(">", 200, 500, 80, 50, mouse_pos=mouse_pos, mouse_click=mouse_click)

                    if not self.painel_names:
                        self._load_unique_names()

                    start = self.painel_scroll
                    end   = min(start + 5, len(self.painel_names))
                    y = 110
                    self.row_buttons = []
                    for i in range(start, end):
                        name = self.painel_names[i]
                        name_txt = self.font.render(name, True, BLACK)
                        self.screen.blit(name_txt, (120, y))
                        r_edit = self._draw_button_simple("Editar", 600, y-10, 120, 40, mouse_pos=mouse_pos, mouse_click=mouse_click)
                        r_del  = self._draw_button_simple("Apagar", 740, y-10, 120, 40, mouse_pos=mouse_pos, mouse_click=mouse_click)
                        self.row_buttons.append((name, r_edit, r_del))
                        y += 70

                    if btn_voltar.collidepoint(mouse_pos) and mouse_click:
                        self.current_page = "Cadastro"
                        self.avancado_buttons_created = False
                        self.painel_buttons_created = False
                    if btn_prev.collidepoint(mouse_pos) and mouse_click:
                        if self.painel_scroll - 5 >= 0:
                            self.painel_scroll -= 5
                    if btn_next.collidepoint(mouse_pos) and mouse_click:
                        if self.painel_scroll + 5 < len(self.painel_names):
                            self.painel_scroll += 5

                    for name, r_edit, r_del in self.row_buttons:
                        if r_edit.collidepoint(mouse_pos) and mouse_click:
                            self.editing_name = name
                            self.edit_input = InputBox(120, 450, 300, 40, text=name)
                            self.current_page = "editar_cadastro"
                            break
                        if r_del.collidepoint(mouse_pos) and mouse_click:
                            self.confirm_delete_name = name

                    if self.confirm_delete_name is not None:
                        pygame.draw.rect(self.screen, (240,240,240), (200, 200, 600, 180))
                        pygame.draw.rect(self.screen, BLACK, (200, 200, 600, 180), 2)
                        t1 = self.font.render(f"Apagar '{self.confirm_delete_name}'?", True, BLACK)
                        self.screen.blit(t1, (220, 230))
                        r_yes = self._draw_button_simple("Sim", 260, 320, 120, 50, mouse_pos=mouse_pos, mouse_click=mouse_click)
                        r_no  = self._draw_button_simple("Cancelar", 420, 320, 160, 50, mouse_pos=mouse_pos, mouse_click=mouse_click)
                        if r_yes.collidepoint(mouse_pos) and mouse_click:
                            self._delete_name(self.confirm_delete_name)
                            self.confirm_delete_name = None
                        elif r_no.collidepoint(mouse_pos) and mouse_click:
                            self.confirm_delete_name = None

                # --------------- EDITAR CADASTRO (renomear) ---------------
                elif self.current_page == "editar_cadastro":
                    self.screen.fill(WHITE)
                    t = self.font.render(f"Editar: {self.editing_name}", True, BLACK)
                    self.screen.blit(t, (100, 60))
                    if self.edit_input is None:
                        self.edit_input = InputBox(100, 120, 300, 40, text=self.editing_name or "")
                    for event in eventos:
                        res = self.edit_input.handle_event(event)
                        if res is not None:
                            pass
                    self.edit_input.draw(self.screen)
                    r_salvar = self._draw_button_simple("Salvar", 420, 120, 140, 50, mouse_pos=mouse_pos, mouse_click=mouse_click)
                    r_cancel = self._draw_button_simple("Cancelar", 580, 120, 160, 50, mouse_pos=mouse_pos, mouse_click=mouse_click)
                    if r_salvar.collidepoint(mouse_pos) and mouse_click:
                        novo = self.edit_input.text.strip()
                        if novo and novo != self.editing_name:
                            self._rename_name(self.editing_name, novo)
                        self.current_page = "painel_cadastros"
                        self.editing_name = None
                        self.edit_input = None
                    if r_cancel.collidepoint(mouse_pos) and mouse_click:
                        self.current_page = "painel_cadastros"
                        self.editing_name = None
                        self.edit_input = None

                # flip
                pygame.display.flip()
        finally:
            #SERVO
            for servo in servos.values():
                servo.ChangeDutyCycle(0)
                servo.stop()
            GPIO.cleanup()
            pygame.quit()
            rclpy.shutdown()

    # botão com hover/click (reutilizado do código original)
    def _draw_button(self, text, rect, mouse_pos, mouse_click):
        # escurece quando hover/click para feedback
        if rect.collidepoint(mouse_pos):
            color = DARK_BLUE if mouse_click else BLUE
        else:
            color = BLUE
        pygame.draw.rect(self.screen, color, rect)
        txt_surf = self.font.render(text, True, WHITE)
        txt_rect = txt_surf.get_rect(center=rect.center)
        self.screen.blit(txt_surf, txt_rect)
        return rect.collidepoint(mouse_pos) and mouse_click

def main(args=None):
    rclpy.init(args=args)
    node = OlhosNode()
    node.run()

if __name__=='__main__':
    main()