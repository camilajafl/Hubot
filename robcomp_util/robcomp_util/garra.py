from std_msgs.msg import Float64

class Garra:
    def __init__(self, node):
        # Inicialização do node e publishers
        self.node = node
        self.ombro_pub = self.node.create_publisher(
            Float64,
            '/joint1_position_controller/command',
            10
        )
        self.garra_pub = self.node.create_publisher(
            Float64,
            '/joint2_position_controller/command',
            10
        )

    def controla_garra(self, command: str):
        """
        Comandos válidos:
          - 'open': abre a garra
          - 'close': fecha a garra
          - 'up': levanta o braço
          - 'mid': posiciona o braço no meio
          - 'down': baixa o braço
        """
        msg = Float64()

        if command == 'open':
            msg.data = -1.0
            self.garra_pub.publish(msg)

        elif command == 'close':
            msg.data = 0.0
            self.garra_pub.publish(msg)

        elif command == 'up':
            msg.data = 1.5
            self.ombro_pub.publish(msg)

        elif command == 'mid':
            msg.data = 0.0
            self.ombro_pub.publish(msg)

        elif command == 'down':
            msg.data = -1.0
            self.ombro_pub.publish(msg)

        else:
            self.node.get_logger().warn(f"Comando de garra desconhecido: {command}")

        # Nota: remova quaisquer delays bloqueantes daqui
        # e gerencie o timing (hora_fixa + offset) na lógica de estados do nó principal.


