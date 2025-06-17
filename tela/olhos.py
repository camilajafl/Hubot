#Teste de olhos usando startx. Para funcionar (NO TERMINAL DA RASP): CTRL + ALT + F1, "startx", depois botao direito, "terminal emulator" e procurar pelos olhos.py. Quando encontra-lo, para rodar: python3 olhos.py 

import os
import pygame

pygame.init()

# Definição da tela
screen = pygame.display.set_mode((1024, 600))
pygame.display.set_caption("Olhinhos de Raposa")

# Imagens da raposa
image_1r = pygame.image.load("olhos_redimensionados/1r.jpeg")
image_2r = pygame.image.load("olhos_redimensionados/2r.jpeg")
image_4r = pygame.image.load("olhos_redimensionados/4r.jpeg")
image_5r = pygame.image.load("olhos_redimensionados/5r.jpeg")

import os
print("Diretório atual:", os.getcwd())
print("Conteúdo da pasta olhos_redimensionados:", os.listdir("olhos_redimensionados"))


# Posição e estados iniciais
agent_x = 0
agent_y = 0
start_time = pygame.time.get_ticks()
image_changed_time = None
current_image = "5r"

# Mostrar a imagem inicial (5r)
screen.fill((200, 200, 200))
screen.blit(image_5r, (agent_x, agent_y))
pygame.display.flip()

# Imagens na Tela sob Loop
running = True
while running:
    print("Imagem atual:", current_image)  # 👈 aqui

    current_time = pygame.time.get_ticks()
    mouse_x, _ = pygame.mouse.get_pos()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # --- Lógica com base na posição do mouse ---
    if mouse_x < 100:
        if current_image != "1r":
            screen.fill((200, 200, 200))
            screen.blit(image_1r, (agent_x, agent_y))
            pygame.display.flip()
            current_image = "1r"
            image_changed_time = None 
            start_time = pygame.time.get_ticks() 
            # print("Imagem atual:", current_image) 


    elif mouse_x > 300:
        if current_image != "2r":
            screen.fill((200, 200, 200))
            screen.blit(image_2r, (agent_x, agent_y))
            pygame.display.flip()
            current_image = "2r"
            image_changed_time = None
            start_time = pygame.time.get_ticks()
            # print("Imagem atual:", current_image) 

    else:
        if current_image not in ["4r", "5r"]:
            screen.fill((200, 200, 200))
            screen.blit(image_5r, (agent_x, agent_y))
            pygame.display.flip()
            current_image = "5r"
            print("Imagem atual:", current_image)
            start_time = pygame.time.get_ticks()
            image_changed_time = None

        elif image_changed_time is None and current_time - start_time >= 5000:
            screen.fill((200, 200, 200))
            screen.blit(image_4r, (agent_x, agent_y))
            pygame.display.flip()
            current_image = "4r"
            print("Imagem atual:", current_image)
            image_changed_time = current_time

        elif current_image == "4r" and current_time - image_changed_time >= 1000:
            screen.fill((200, 200, 200))
            screen.blit(image_5r, (agent_x, agent_y))
            pygame.display.flip()
            current_image = "5r"
            print("Imagem atual:", current_image)
            start_time = pygame.time.get_ticks()
            image_changed_time = None

        # print("Imagem atual:", current_image)

pygame.quit()
