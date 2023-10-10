from djitellopy import Tello
tello = Tello()

altura_operacional = 250 #CONFERIR ALTURA OPERACIONAL SEMPRE

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

    distancia_passo = int(distancia/passo)

    if(direcao == "esquerda"):
        for i in range(distancia_passo):
            # tello.move_left(passo)
            print("esquerda: ", passo)

    if (direcao == "direita"):
        for i in range(distancia_passo):
            # tello.move_right(passo)
            print("direita: ", passo)

    if (direcao == "frente"):
        for i in range(distancia_passo):
            # tello.move_forward(passo)
            print("frente: ", passo)

    if (direcao == "tras"):
        for i in range(distancia_passo):
            # tello.move_back(passo)
            print("trás: ", passo)

def vai_para(origem, destino, coordenada_passo = 20):

    direcao = diferenca(origem , destino)
    print(direcao)

    if direcao[2] > 0:
        # tello.move_up(direcao[2])
        print("cima: ", direcao[2])

    if direcao[0] > 0:
        print("esquerda total: ", direcao[0])
        mover_passo(direcao[0], 25, "esquerda")

    if direcao[0] < 0:
        print("direita total: ", direcao[0] * -1)
        mover_passo(direcao[0]*-1, coordenada_passo, "direita")

    if direcao[1] > 0:
        print("frente total: ", direcao[1])
        mover_passo(direcao[1], 100, "frente")

    if direcao[1] < 0:
        print("trás total: ", direcao[1] * -1)
        mover_passo(direcao[1]*-1, 100, "tras")

    if direcao[2] < 0:
        # tello.move_down((direcao[2]*-1))
        print("baixo: ", direcao[2]*-1)


print(vai_para(coordenadas_home,posicao_base1))
print(vai_para(posicao_base1,posicao_base2))
print(vai_para(posicao_base2,coordenadas_home,25))
