# Seguidor de linha

Disciplina: VCIOT, Engenharia de Software, 2026

Equipe: Filipe Ribeiro Simões (RA 24007657)

## Escolha do controle

O carro é controlado dentro do próprio controller do Webots (`controllers/car_control/car_control.py`), sem controlador externo.

A escolha foi feita por três motivos. O primeiro é a reprodução: basta abrir o mundo e apertar play, sem configurar variáveis de ambiente nem iniciar outro processo em cada computador. O segundo é o sincronismo: o controller roda no mesmo passo da simulação, então cada imagem da câmera gera um comando para as rodas no passo seguinte, sem atraso de comunicação. O terceiro é que o processamento é leve (imagem de 126 x 128 pixels) e não precisa de outra máquina ou processo. A desvantagem é depurar com menos conforto do que em uma IDE, o que foi compensado com as janelas do OpenCV mostrando o que o carro enxerga.

## Como funciona

1. A imagem da câmera é convertida de BGRA para BGR, depois para tons de cinza, e suavizada com filtro gaussiano.
2. Um limiar fixo separa a linha preta do piso. O valor (60) fica abaixo do cinza da sombra do carro (cerca de 90), que senão seria confundida com a linha.
3. Na faixa de 50% a 90% da altura da imagem, o maior contorno é a linha, e o centro dele é calculado pelos momentos.
4. A distância desse centro ao meio da imagem é o erro, de -1 (linha à esquerda) a 1 (linha à direita).
5. Um controle PD transforma o erro em diferença de velocidade entre as rodas, e o carro desacelera quanto maior o erro.
6. Se a linha some, o carro gira para o último lado em que a viu e, se não achar em 3 segundos, para.
7. A faixa vermelha no fim da pista é detectada em HSV. Ao chegar nela, o carro para e mostra o tempo no console.

## O que mudou em relação ao mundo original

A pista é nova (`pistas/pista.png`): aberta, com início e fim, e curvas para os dois lados. A imagem está dentro do projeto em vez de ser baixada da internet, e parece achatada porque o Webots a estica para o piso de 1,2 x 1,8 m.

O carro ganhou caixa de colisão e dois rodízios sem atrito. No original, sem caixa de colisão, o Webots usava uma matriz de inércia padrão muito grande, e o carro derrapava e saía da pista nas curvas.

## Como executar

Requisitos: Webots R2025a e Python 3 com as bibliotecas abaixo.

```
pip install opencv-python numpy
```

Abra `worlds/carro.wbt` no Webots e aperte play. Para rodar sem as janelas do OpenCV, mude `MOSTRAR_JANELAS` para `False` no controller.
