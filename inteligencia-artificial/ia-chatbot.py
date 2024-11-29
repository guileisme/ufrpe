def iniciar ():
  tabela_respostas = ['Olá usuário, como posso ajudar?\n',
                      '\nTudo bem, vou trabalhar nisso agora mesmo!',
                      '\nAté mais, usuário!',
                      '\nVocê não pode acessar o banco de dados do banco e transferir todo esse dinheiro para a sua conta!',
                      '\nO batman não existe, então é impossível chamar ele no momento.',
                      '\nEssa música é muito boa! Para tocá-la, abra o seu aplicativo de músicas e pesquise o nome da música.']
  nome = input('Qual é o seu nome? ')
  tabela_respostas[0] = '\nOlá {}, como posso ajudar?\n'.format(nome)
  tabela_respostas[2] = '\nAté mais, {}!'.format(nome)
  print(tabela_respostas[0])
  resposta1 = str(input())
  resposta2 = resposta1.lower()
  resposta = resposta2.split()
  terminar = 0
  verificar1 = False
  verificar2 = False
  verificar3 = False
  verificar4 = False
  verificar5 = False
  verificar6 = False
  while resposta[0] != 'encerrar' and terminar != 1:
    if resposta[0] == 'quero':
      print(tabela_respostas[1])
      terminar = 1
    if resposta[0] == 'encerrar':
      print(tabela_respostas[2])
      terminar = 1
    for i in range(len(resposta)):
      if resposta[i] == 'roubar':
        verificar1 = True
      if resposta[i] == 'banco':
        verificar2 = True
      if resposta[i] == 'tocar':
        verificar3 = True
      if resposta[i] == 'música':
        verificar4 = True
      if resposta[i] == 'chamar':
        verificar5 = True
      if resposta[i] == 'batman':
        verificar6 = True
      if verificar1 == True and verificar2 == True:
        print(tabela_respostas[3])
        terminar = 1
      if verificar3 == True and verificar4 == True:
        print(tabela_respostas[5])
        terminar = 1
      if verificar5 == True and verificar6 == True:
        print(tabela_respostas[4])
        terminar = 1

iniciar()