from djitellopy import Tello
tello = Tello()

altura_operacional = 250

coordenadas_home = [75,100,25]

posicao_base1 = [100, 700, altura_operacional]

posicao_base2 = [350, 100, altura_operacional]

coordenadas_atuais = coordenadas_home

def diferenca(coordenadas_atuais, coordenadas_desejadas):
    dif = len(coordenadas_desejadas) * [0]

    if len(coordenadas_desejadas) > len(coordenadas_atuais):
        dif[2] = coordenadas_desejadas[2]

    for i in range(len(coordenadas_desejadas)):
        dif[i] = coordenadas_desejadas[i] - coordenadas_atuais[i]

    return(dif)

def mover_passo(distancia, passo, direcao):


def vai_para(origem, destino, coordenada_passo = 20):

    direcao = diferenca(origem , destino)
    print(direcao)

    if direcao[2] > 0:
        # tello.move_up(direcao[2])
        print("cima: ", direcao[2])

    if direcao[0] > 0:
        mover_passo(direcao[0], coordenada_passo, "left")
        print("esquerda: ", direcao[0])

    if direcao[0] < 0:
        # tello.move_right((direcao[0]*-1))
        print("direita: ", direcao[0]*-1)

    if direcao[1] > 0:
        # tello.move_forward(direcao[1])
        print("frente: ", direcao[1])

    if direcao[1] < 0:
        # tello.move_back((direcao[1]*-1))
        print("trás: ", direcao[1]*-1)

    if direcao[2] < 0:
        # tello.move_down((direcao[2]*-1))
        print("baixo: ", direcao[2]*-1)

    # if direcao[2] == 0:
    #     # tello.land()
    #     print("pousar")

print(vai_para(coordenadas_home,posicao_base1))
print(vai_para(posicao_base1,posicao_base2))
print(vai_para(posicao_base2,coordenadas_home))
