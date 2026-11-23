from controller import Robot
import cv2
import numpy as np

# deixa False para rodar sem as janelas do opencv
MOSTRAR_JANELAS = True

# velocidade das rodas em rad/s, na reta e na curva
VEL_MAX = 9.0
VEL_MIN = 4.0

# KP vira o carro para a linha, KD segura para não ficar balançando
KP = 5.0
KD = 0.25

# a linha dá uns 30 de cinza e a sombra do carro uns 90, por isso 60
LIMIAR_PRETO = 60
AREA_MINIMA = 60

FRACAO_CHEGADA = 0.15
TEMPO_CHEGADA = 0.6
TEMPO_PERDIDO = 3.0


def mascara_vermelha(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # no hsv o vermelho fica no começo e no fim da escala, então precisa de duas faixas
    baixo = cv2.inRange(hsv, (0, 120, 60), (10, 255, 255))
    alto = cv2.inRange(hsv, (170, 120, 60), (180, 255, 255))
    return cv2.bitwise_or(baixo, alto)


def mascara_linha(frame, vermelho):
    cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cinza = cv2.GaussianBlur(cinza, (5, 5), 0)
    _, linha = cv2.threshold(cinza, LIMIAR_PRETO, 255, cv2.THRESH_BINARY_INV)
    # em cinza o vermelho fica escuro e parece linha, então tira
    linha[vermelho > 0] = 0
    return linha


def centro_da_linha(linha, topo, base):
    # olha só um pedaço da imagem e pega o meio do maior contorno
    faixa = linha[topo:base, :]
    contornos, _ = cv2.findContours(faixa, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contornos:
        return None
    maior = max(contornos, key=cv2.contourArea)
    m = cv2.moments(maior)
    if m["m00"] < AREA_MINIMA:
        return None
    return int(m["m10"] / m["m00"]), topo + int(m["m01"] / m["m00"])


robot = Robot()
timestep = int(robot.getBasicTimeStep())

motorE = robot.getDevice('motorE')
motorD = robot.getDevice('motorD')
for motor in (motorE, motorD):
    # posição infinita para controlar a roda pela velocidade
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

camera = robot.getDevice('camera')
camera.enable(timestep)
largura = camera.getWidth()
altura = camera.getHeight()

# parte da imagem usada para seguir a linha
TOPO = int(altura * 0.50)
BASE = int(altura * 0.90)

erro_anterior = 0.0
# último lado que a linha apareceu, -1 esquerda e 1 direita
ultimo_lado = 1.0
perdido_desde = None
chegada_em = None

print("Seguidor de linha iniciado")

while robot.step(timestep) != -1:
    bruto = camera.getImage()
    if not bruto:
        continue

    # a câmera manda em bgra
    imagem = np.frombuffer(bruto, np.uint8).reshape((altura, largura, 4))
    frame = cv2.cvtColor(imagem, cv2.COLOR_BGRA2BGR)

    vermelho = mascara_vermelha(frame)
    linha = mascara_linha(frame, vermelho)
    centro = centro_da_linha(linha, TOPO, BASE)
    viu_chegada = np.count_nonzero(vermelho) > AREA_MINIMA
    chegada = np.mean(vermelho[BASE:, :] > 0)

    if MOSTRAR_JANELAS:
        visao = frame.copy()
        cv2.rectangle(visao, (0, TOPO), (largura - 1, BASE), (255, 160, 0), 1)
        if centro:
            cv2.circle(visao, centro, 4, (0, 200, 0), -1)
        cv2.imshow("Camera", cv2.resize(visao, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
        cv2.imshow("Linha detectada", cv2.resize(linha, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
        cv2.waitKey(1)

    # chegou na faixa vermelha, anda mais um pouco e para em cima
    if chegada_em is None and chegada > FRACAO_CHEGADA:
        chegada_em = robot.getTime()

    if chegada_em is not None:
        if robot.getTime() - chegada_em > TEMPO_CHEGADA:
            motorE.setVelocity(0.0)
            motorD.setVelocity(0.0)
            print(f"Chegada! Tempo: {robot.getTime():.1f} s")
            break
        motorE.setVelocity(VEL_MIN)
        motorD.setVelocity(VEL_MIN)
        continue

    if centro is None and viu_chegada:
        # acabou a linha mas já dá para ver a chegada, segue reto
        motorE.setVelocity(VEL_MIN)
        motorD.setVelocity(VEL_MIN)
        continue

    if centro is None:
        # perdeu a linha, gira para o lado que viu por último
        if perdido_desde is None:
            perdido_desde = robot.getTime()
        if robot.getTime() - perdido_desde > TEMPO_PERDIDO:
            motorE.setVelocity(0.0)
            motorD.setVelocity(0.0)
            print("Linha perdida, carro parado")
            break
        motorE.setVelocity(VEL_MIN * ultimo_lado)
        motorD.setVelocity(-VEL_MIN * ultimo_lado)
        continue

    perdido_desde = None

    # distância da linha até o meio da imagem, de -1 (esquerda) até 1 (direita)
    erro = (centro[0] - largura / 2) / (largura / 2)
    derivada = (erro - erro_anterior) / (timestep / 1000)
    erro_anterior = erro
    if abs(erro) > 0.05:
        ultimo_lado = 1.0 if erro > 0 else -1.0

    # quanto maior o erro mais ele vira e mais devagar anda
    correcao = KP * erro + KD * derivada
    velocidade = max(VEL_MIN, VEL_MAX * (1 - abs(erro)))
    motorE.setVelocity(velocidade + correcao)
    motorD.setVelocity(velocidade - correcao)

# fica parado até fechar a simulação
while robot.step(timestep) != -1:
    if MOSTRAR_JANELAS:
        cv2.waitKey(1)

cv2.destroyAllWindows()
