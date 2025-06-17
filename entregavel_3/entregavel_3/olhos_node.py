# entregavel_3/entregavel_3/olhos_node.py

import os
import rclpy
import pygame
from rclpy.node import Node
from std_msgs.msg import String

class OlhosNode(Node):
    def __init__(self):
        super().__init__('olhos_node')

        # subscription ao tópico eye_direction
        self.sub = self.create_subscription(
            String, 'eye_direction', self.cb_direction, 10)

        # --- Pygame fullscreen ---
        pygame.init()
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

        # estado inicial e timers de piscar
        self.current_direction = 'forward'
        self.blink_interval    = 5000   # ms até piscar
        self.blink_duration    = 1000   # ms olhos fechados
        self.last_blink_time   = pygame.time.get_ticks()
        self.blinking          = False
        self.blink_start_time  = None

        # desenha estado inicial
        self._draw(self.current_direction)

    def cb_direction(self, msg: String):
        # troca de direção interrompe piscar
        if msg.data in ('left','right','forward'):
            self.current_direction = msg.data
            self.blinking         = False
            self.last_blink_time  = pygame.time.get_ticks()
            self._draw(self.current_direction)

    def _draw(self, key):
        self.screen.fill((200,200,200))
        self.screen.blit(self.images[key], (0,0))
        pygame.display.flip()

    def run(self):
        try:
            while rclpy.ok():
                # processa callbacks ROS
                rclpy.spin_once(self, timeout_sec=0.1)

                # processa eventos Pygame (p.ex. ESC sai)
                for e in pygame.event.get():
                    if e.type == pygame.QUIT or (
                       e.type==pygame.KEYDOWN and e.key==pygame.K_ESCAPE):
                        return

                # lógica de piscar
                now = pygame.time.get_ticks()
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

        finally:
            pygame.quit()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OlhosNode()
    node.run()

if __name__=='__main__':
    main()
