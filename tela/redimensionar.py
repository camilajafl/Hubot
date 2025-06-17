import pygame
import os

pygame.init()

# Caminhos relativos a partir de /home/ubuntu
pasta_origem = "turtlebot3_ws/src/Hubot/tela/olhos"
pasta_destino = "turtlebot3_ws/src/Hubot/tela/olhos_redimensionados"
os.makedirs(pasta_destino, exist_ok=True)

arquivos = ["1r.jpeg", "2r.jpeg", "4r.jpeg", "5r.jpeg"]
nova_largura = 1024
nova_altura = 600

for nome in arquivos:
    caminho = os.path.join(pasta_origem, nome)
    imagem = pygame.image.load(caminho)
    imagem_redimensionada = pygame.transform.scale(imagem, (nova_largura, nova_altura))
    caminho_salvar = os.path.join(pasta_destino, nome)
    pygame.image.save(imagem_redimensionada, caminho_salvar)

print("Imagens redimensionadas com sucesso.")
