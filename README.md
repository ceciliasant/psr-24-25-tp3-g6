# Trabalho 3 - Robutler

## Sumário

Pretende-se desenvolver um sistema robótico que funcione como um mordomo.
O Robô deve ser capaz de realizar variadas missões de apoio habitualmente realizadas por trabalhadores humanos.

O robô deve ter desenvolvidas várias funcionalidades que suportem a operacionalização destas missões, tais como a perceção de objetos e a movimentação no cenário.

## Executando o Sistema

### Configuração Inicial do Workspace
1. Crie o workspace do Catkin:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```
2. Clone o repositório:
    ```bash
    git clone <url-do-repositorio>
    ```
3. Compile o workspace:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

### Iniciar o Sistema
Para iniciar o robô no cenário Gazebo, use:  
```bash
roslaunch robutler_bringup bringup.launch
```

## Requisitos
```bash
SpeechRecognition
python3-audio
alsa-utils 
libasound2 
libasound2-dev 
libjack-jackd2-dev
python3-opencv
cv_bridge

```

## Controlo do robô

### Navegação Semântica
```bash
rostopic pub /semantic_goal std_msgs/String "data: 'kitchen'"
```

### Navegação por Coordenadas
```bash
rostopic pub /move_to_coord std_msgs/String "data: '1.0,2.0'"
```

### Controle por Voz
Ative o controle por voz ao inicializar o robô:
```bash
roslaunch robutler_bringup bringup.launch enable_voice:=true
```

## Cenário de testes

O cenário de testes é chamado **AWS small house**, e é um apartamento T2 simulado em Gazebo.


## Objetivos

### Configuração do robô

Plataforma Turtlebot Waffle Pi.

### Mapeamento do Cenário

O cenário foi mapeado com SLAM, e utilizado como suporte à localização e navegação autónoma.

### Movimentação do robô pelo apartamento

É possível movimentar o robô pelo apartamento de várias formas distintas:

    - Manual: Controle via teleop (ex.: controle aprimorado com uma interface personalizada).
    - Coordenadas: Navegação para um alvo definido por X e Y.
    - Semântica: Navegação para locais pré-definidos, como "cozinha" ou "sala".
    - Voz: Navegação para alvos a partir de comandos de voz.

### Perceção

O robô deve identificar:

    - Objetos (ex.: esferas violeta, laptops, garrafas).
    - Pessoas.
    - Locais específicos, utilizando dados dos sensores.
    Reconhecimentos básicos podem ser baseados em cores ou sensores avançados para objetos mais complexos.

### Spawn de objetos na casa

Implementamos um script para adicionar objetos no cenário Gazebo:

    - Argumentos devem definir o objeto e divisão.
    - Ou gere aleatoriamente o objeto e a posição a partir de locais predefinidos.

### Missões Passivas

Se as funcionalidades acima descritas estiverem implementadas o robô conseguirá desempenhar várias missões.
Deverá ser possível iniciar uma missão utilizando um menu com marcadores interativos RViz:

http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started#menu

Deverá também acrescentar texto próximo do robô a indicar a missão que este está atualmente a desempenhar.

Exemplos de missões:

   - Mover para quarto, mover para sala;
   - Procurar uma esfera vermelha no quarto pequeno;
   - Verificar se o computador portátil está no escritório;
   - Verificar se está alguém no quarto grande;
   - Verificar se a mesa da sala está levantada (livre de objetos);
   - Fotografar a divisão X;
   - Verificar se alguém está em casa;
   - Contar o número de cubos azuis existentes em casa;
   - Outras que considerem interessantes.

### Missões Ativas (Muito Avançado)

Como funcionalidades avançadas, sugere-se que o robô seja capaz não só de observar mas também de interagir com o meio ambiente. A interação pode ser básica ou mais evoluída. Alguns exemplos:

   - Tocar no objeto X;
   - Deitar o objeto X abaixo da mesa na divisão Y;
   - Agarrar a lata de coca-cola do quarto e colocá-la no balde do lixo do escritório;
   - Outras que considerem relevantes.

