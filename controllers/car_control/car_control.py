from controller import Robot
import cv2
import numpy as np

# --- Parâmetros ---
MOSTRAR_JANELAS = True

VEL_MAX = 9.0        # rad/s na reta
VEL_MIN = 4.0        # rad/s na curva mais fechada
KP = 5.0             # ganho proporcional
KD = 0.25            # ganho derivativo
LIMIAR_PRETO = 60    # abaixo disso, em tons de cinza, o pixel é linha (a sombra do carro fica perto de 90)
AREA_MINIMA = 60     # em pixels, descarta ruído
FRACAO_CHEGADA = 0.15  # fração vermelha da base da imagem que indica a chegada
TEMPO_CHEGADA = 0.6  # segundos andando reto para parar em cima da faixa
TEMPO_PERDIDO = 3.0  # segundos procurando a linha antes de desistir


def mascara_vermelha(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # O vermelho fica nas duas pontas da escala de matiz do OpenCV
    baixo = cv2.inRange(hsv, (0, 120, 60), (10, 255, 255))
    alto = cv2.inRange(hsv, (170, 120, 60), (180, 255, 255))
    return cv2.bitwise_or(baixo, alto)


def mascara_linha(frame, vermelho):
    cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cinza = cv2.GaussianBlur(cinza, (5, 5), 0)
    _, linha = cv2.threshold(cinza, LIMIAR_PRETO, 255, cv2.THRESH_BINARY_INV)
    linha[vermelho > 0] = 0
    return linha


def centro_da_linha(linha, topo, base):
    """Centro (x, y) do maior trecho de linha entre as linhas topo e base da imagem."""
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
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

camera = robot.getDevice('camera')
camera.enable(timestep)
largura = camera.getWidth()
altura = camera.getHeight()

# Faixa da imagem usada para medir o erro (a de baixo está colada no carro)
TOPO = int(altura * 0.50)
BASE = int(altura * 0.90)

erro_anterior = 0.0
ultimo_lado = 1.0    # lado em que a linha foi vista por último: -1 esquerda, 1 direita
perdido_desde = None
chegada_em = None

print("Seguidor de linha iniciado")

while robot.step(timestep) != -1:
    bruto = camera.getImage()
    if not bruto:
        continue

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
        # A linha acabou mas a faixa vermelha está à frente: segue reto até ela
        motorE.setVelocity(VEL_MIN)
        motorD.setVelocity(VEL_MIN)
        continue

    if centro is None:
        # Perdeu a linha: gira no lugar para o lado em que ela foi vista por último
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

    # Erro normalizado: -1 (linha na borda esquerda) a 1 (borda direita)
    erro = (centro[0] - largura / 2) / (largura / 2)
    derivada = (erro - erro_anterior) / (timestep / 1000)
    erro_anterior = erro
    if abs(erro) > 0.05:
        ultimo_lado = 1.0 if erro > 0 else -1.0

    correcao = KP * erro + KD * derivada
    velocidade = max(VEL_MIN, VEL_MAX * (1 - abs(erro)))
    motorE.setVelocity(velocidade + correcao)
    motorD.setVelocity(velocidade - correcao)

# Mantém o carro parado até o fim da simulação
while robot.step(timestep) != -1:
    if MOSTRAR_JANELAS:
        cv2.waitKey(1)

cv2.destroyAllWindows()
