# entregavel_3/entregavel_3/olhos_node.py

import os
import rclpy
import pygame
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile


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

class OlhosNode(Node):
    def __init__(self):
        super().__init__('olhos_node')

        self.username = "Unknown"
        self.current_page = "menu"
        self.config_buttons_created = False
        self.avancado_buttons_created = False
        self.cadastro_buttons_created = False
        self.botao_voltar_cadastro = False

        self.twist = Twist()


        pygame.font.init()
        self.font = pygame.font.SysFont(None, 36)

        #modo de estado
        self.botao_recepcionista_ativo = True  # default
        self.botao_limpadora_ativo = False
        self.botao_tourista_ativo = False

        # SUBSCRIBERS
        
        #reconhecimento de usuário
        self.sub_user = self.create_subscription(
            String, 'recognized_user', self.cb_user, 10)
        self.tracked_names = []
        self.boxes = []

        self.sub_face_center = self.create_subscription(
            String, 'primeiro_face_location', self.cb_face_center, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        #PUBLISHERS
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_captura = self.create_publisher(String, 'captura_nome', 10)

        #acompanhamento do usuário

        self.current_image = "5r"
        self.image_changed_time = None
        self.start_time = pygame.time.get_ticks()

        

        # --- Pygame fullscreen ---
        pygame.init()
        # PARA DEBUG
        # self.screen = pygame.display.set_mode((1024, 600))

        #PRO PROJETO
        self.screen = pygame.display.set_mode((1024, 600), pygame.FULLSCREEN)

        pygame.display.set_caption("Olhinhos de Raposa")

        # --- Carrega imagens pelo caminho relativo ao módulo ---
        this_dir = os.path.dirname(__file__)
        folder   = os.path.join(this_dir, 'olhos_redimensionados')
        self.images = {
            'left':    pygame.image.load(os.path.join(folder, '1r.jpeg')),
            'right':   pygame.image.load(os.path.join(folder, '2r.jpeg')),
            'forward': pygame.image.load(os.path.join(folder, '5r.jpeg')),
            'blink':   pygame.image.load(os.path.join(folder, '4r.jpeg')),
        }
        self.current_image = 'forward' 

        # estado inicial e timers de piscar
        self.current_direction = 'forward'
        self.blink_interval    = 5000   # ms até piscar
        self.blink_duration    = 1000   # ms olhos fechados
        self.last_blink_time   = pygame.time.get_ticks()
        self.blinking          = False
        self.blink_start_time  = None

        # desenha estado inicial
        self._draw(self.current_direction)

    def draw_button(self, text, rect, mouse_pos, mouse_click):
        if rect.collidepoint(mouse_pos):
            color = (30, 100, 200)  # azul escuro quando hover
            if mouse_click:
                return True
        else:
            color = (50, 130, 230)  # azul normal
        pygame.draw.rect(self.screen, color, rect)
        txt_surf = self.font.render(text, True, (255, 255, 255))
        txt_rect = txt_surf.get_rect(center=rect.center)
        self.screen.blit(txt_surf, txt_rect)
        return False

    def _draw(self, key):
        self.screen.fill((200,200,200))
        self.screen.blit(self.images[key], (0,0))
    
    #administrador
    def cb_user(self, msg: String):
        self.username = msg.data


    def cb_face_center(self, msg: String):
        # Atualiza a imagem APENAS quando Limpadora for False
        if self.botao_limpadora_ativo:
            return  # Sai sem atualizar se Limpadora estiver ativa

        direction_map = {
            "esquerda": "left",
            "direita": "right",
            "centralizado": "forward"
        }

        ros_direction = msg.data
        new_direction = direction_map.get(ros_direction, "forward")

        if new_direction != self.current_direction:
            # Fonte única de verdade: apenas estado + timers
            self.current_direction = new_direction
            self.current_image = new_direction  # mantém compatibilidade com quem ainda lê current_image
            self.blinking = False
            self.last_blink_time = pygame.time.get_ticks()

        if ros_direction == "esquerda":
            self.twist.angular.z = -0.3   # gira para a esquerda
        elif ros_direction == "direita":
            self.twist.angular.z = 0.3  # gira para a direita
        else:  # centralizado
            self.twist.angular.z = 0.0   # para de girar
        self.cmd_vel_pub.publish(self.twist)


    def update_image(self):
    
        self.current_image = self.current_direction

    def run(self):
        try:
            while rclpy.ok():
                # processa callbacks ROS
                rclpy.spin_once(self, timeout_sec=0.1)
                eventos = pygame.event.get()

                mouse_click = False
                mouse_pos = pygame.mouse.get_pos()
                for event in eventos:
                    if event.type == pygame.QUIT or (
                        event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        return
                    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                        mouse_click = True

                self.update_image()
                # processa eventos Pygame (p.ex. ESC sai)
                for e in eventos:
                    if e.type == pygame.QUIT or (
                       e.type==pygame.KEYDOWN and e.key==pygame.K_ESCAPE):
                        return

                now = pygame.time.get_ticks()

                #logica de reconhecimento
                if self.current_page == "menu":
                    self._draw(self.current_direction)
                    if self.username != "Unknown":
                        button_rect = pygame.Rect(800, 500, 200, 60)
                        if self.draw_button("Configurações", button_rect, mouse_pos, mouse_click):
                            self.current_page = "pagina_2"
                            self.config_buttons_created = False
                
                #segunda pagina
                elif self.current_page == "pagina_2":
                    #self.screen.fill(WHITE)
                    txt = self.font.render("Configurações", True, BLACK)
                    self.screen.blit(txt, (350, 50))
                    if not self.config_buttons_created:
                        self.botao_orelha = Botao("Orelha", 100, 100, 200, 60)
                        self.botao_braco = Botao("Braço", 100, 200, 200, 60)
                        self.botao_recepcionista = Botao("Recepcionista", 100, 300, 200, 60, ativo=True)  # começa ativo
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
                        botao.draw(self.screen)

                    for botao in self.botoes_config:
                        if botao.checar_clique(mouse_pos, mouse_click):
                            if botao.texto == "Voltar":
                                self.current_page = "menu"
                                self.config_buttons_created = False

                            elif botao.texto == "Recepcionista":
                                self.botao_recepcionista.ativar()
                                self.botao_limpadora.desativar()
                                self.botao_tourista.desativar()
                                self.botao_recepcionista_ativo = True
                                self.botao_limpadora_ativo = False
                                self.botao_tourista_ativo = False


                            elif botao.texto == "Limpadora":
                                self.botao_limpadora.ativar()
                                self.botao_recepcionista.desativar()
                                self.botao_tourista.desativar()
                                self.botao_recepcionista_ativo = False
                                self.botao_limpadora_ativo = True
                                self.botao_tourista_ativo = False
                                # self.limpador_ativo = True 
  

                            elif botao.texto == "Turista":
                                self.botao_tourista.ativar()
                                self.botao_recepcionista.desativar()
                                self.botao_limpadora.desativar()
                                # current_page = "pagina_tour"
                                self.botao_recepcionista_ativo = False
                                self.botao_limpadora_ativo = False
                                self.botao_tourista_ativo = True

                            elif botao.texto == "Orelha":
                                # Apenas alterna o botão, sem interferir nos outros
                                if self.botao_orelha.ativo:
                                    self.botao_orelha.desativar()
                                else:
                                    self.botao_orelha.ativar()

                            elif botao.texto == "Braço":
                                if self.botao_braco.ativo:
                                    self.botao_braco.desativar()
                                else:
                                    self.botao_braco.ativar()

                            elif botao.texto == "Cadastro":
                                self.current_page = "Cadastro"
                                self.config_buttons_created = False
                
                elif self.current_page == "Cadastro":
                    #self.screen.fill(WHITE)
                    txt = self.font.render("Configurações avançadas", True, BLACK)
                    self.screen.blit(txt, (350, 50))

                    if not self.avancado_buttons_created:
                        self.botao_novo = Botao("Novo cadastro", 100, 100, 200, 60)
                        self.botao_painel = Botao("painel de cadastros", 100, 200, 200, 60)
                        self.botao_voltar = Botao("Voltar", 800, 500, 200, 60)
                        self.botoes_avancado = [
                            self.botao_novo, self.botao_painel, self.botao_voltar]
                        self.avancado_buttons_created = True

                    for botao in self.botoes_avancado:
                        botao.draw(self.screen)

                    for botao in self.botoes_avancado:
                        if botao.checar_clique(mouse_pos, mouse_click):
                            if botao.texto == "Voltar":
                                self.current_page = "pagina_2"
                                self.avancado_buttons_created = False
                                self.config_buttons_created = False
                            elif botao.texto == "Novo cadastro":
                                self.current_page = "pagina_novo_cadastro"
                                self.avancado_buttons_created = False
                                self.config_buttons_created = False
                            elif botao.texto == "painel de cadastros":
                                print('oi2')
                
                elif self.current_page == "pagina_novo_cadastro":
                    self.screen.fill(WHITE)

                    if not self.cadastro_buttons_created:
                        self.botao_voltar_cadastro = Botao("Voltar", 800, 500, 200, 60)
                        self.botao_capturar = Botao("Tirar Foto", 100, 250, 200, 50)
                        self.input_box = InputBox(100, 100, 300, 40)
                        self.nome_digitado = ""
                        self.erro = ""
                        self.captura_iniciada = False
                        self.img_counter = 0
                        self.max_fotos = 5
                        self.etapa_captura = None
                        self.processo_em_execucao = None
                        self.cadastro_buttons_created = True

                    # Título
                    txt = self.font.render("Digite o nome:", True, BLACK)
                    self.screen.blit(txt, (100, 60))

                    # Eventos
                    
                    for event in eventos:
                        if event.type == pygame.QUIT or (
                            event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                            return
                        else:
                            resultado = self.input_box.handle_event(event)
                            if resultado:
                                self.nome_digitado = resultado

                    # Caixa de entrada
                    self.input_box.draw(self.screen)

                    # Exibe nome digitado
                    nome = self.input_box.text.strip()
                    nome_txt = self.font.render(f"Nome: {nome}", True, BLACK)
                    self.screen.blit(nome_txt, (100, 150))

                    self.botao_capturar.draw(self.screen)
                    self.botao_voltar_cadastro.draw(self.screen)
                    
                    # Clique "Tirar Foto"
                    if self.botao_capturar.checar_clique(mouse_pos, mouse_click):
                        nome = self.input_box.text.strip()
                        if not nome:
                            self.erro = "Digite um nome válido."
                        else:
                            self.erro = "Enviando pedido de cadastro..."
                            self.pub_captura.publish(String(data=nome))


                    # Clique "Voltar"
                    if self.botao_voltar_cadastro.checar_clique(mouse_pos, mouse_click):
                        self.current_page = "Cadastro"
                        self.cadastro_buttons_created = False
                
                if not self.blinking:
                    if now - self.last_blink_time >= self.blink_interval:
                        self.blinking         = True
                        self.blink_start_time = now
                        self._draw('blink')
                else:
                    if now - self.blink_start_time >= self.blink_duration:
                        self.blinking         = False
                        self.last_blink_time  = now
                        self._draw(self.current_direction)                

                
                pygame.display.flip()

        finally:
            pygame.quit()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OlhosNode()
    node.run()

if __name__=='__main__':
    main()