import numpy as np

table= [['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '0'],
        ['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '1'],
        ['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '2'],
        ['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '3'],
        ['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '4'],
        ['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '5'],
        ['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '6'],
        ['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '7'],
        ['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '8'],
        ['_', '_', '_', '_', '_', '_', '_', '_', '_', '_', '9']
]

def showTable(table):
    print('  0    1    2    3    4    5    6    7    8    9  ')
    for i in range(len(table)):
        print(table[i])

def victoryCheck (table, jogador):
    # vitória na vertical
    verticalCounter = 0
    for i in range(len(table)):
        for j in range(len(table[0])-2):
            if table[i][j] == jogador:
                verticalCounter += 1
    if verticalCounter == 5:
        return True

    # vitória na horizontal
    horizontalCounter = 0
    for n in range(len(table)):
        for m in range(len(table[0])-2):
            if table[n][m] == jogador:
                horizontalCounter += 1
    if horizontalCounter == 5:
        return True

    # vitória na diagonal
    diagonalCounter = 0
    for d in range(len(table)):
        for e in range(len(table[0])-2):
            if table[d][e] == jogador:
                diagonalCounter += 1
            elif table[d][e] == jogador:
                diagonalCounter += 1
    if diagonalCounter == 5:
        return True

def minMax (table):
    
    
    return null

def play(table, jogador):
    showTable(table)

    move = input("Em qual posição você gostaria de jogar?\n")
    coordenate = move.split(',')
    coordenate[0] = int(coordenate[0])
    coordenate[1] = int(coordenate[1])
    while table[coordenate[0]][coordenate[1]] == 'X' or table[coordenate[0]][coordenate[1]] == 'O':
        print("Jogada inválida, tente novamente com outra posição.\n")
        move = input("Em qual posição você gostaria de jogar?\n")
        coordenate = move.split(',')
        coordenate[0] = int(coordenate[0])
        coordenate[1] = int(coordenate[1])
    table[coordenate[0]][coordenate[1]] = jogador
    if victoryCheck(table, jogador) == True:
        print("\nParabéns, você ganhou!!!")
        showTable(table)
        return True

jogador1 = 'X'
jogador2 = 'O'
while play(table, jogador1) != True and play(table, jogador2) != True:
    play(table, jogador1)
    play(table, jogador2)
