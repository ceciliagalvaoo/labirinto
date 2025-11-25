# Desafio de Navegação Autônoma em Labirinto (ROS2 + Pygame)

Este projeto implementa o **Desafio de Navegação Autônoma** proposto na disciplina de Computação, utilizando **algoritmos de grafos (BFS/DFS)** integrados a um simulador de labirinto em ROS2.

O trabalho está dividido em **duas partes**, conforme o enunciado:

- **Parte 1:** Navegação com mapa conhecido (planejamento global com BFS).  
- **Parte 2:** Exploração com sensores (DFS), construção de mapa próprio e navegação com BFS usando esse mapa aprendido.


## Contexto do Problema

O simulador representa um labirinto como uma **grade (grid)** de células com:

- **AZUL** → robô (posição inicial)  
- **VERMELHO** → alvo (target)  
- **BRANCO** → células livres (o robô pode passar)  
- **PRETO** → paredes (o robô não pode passar)  

O robô se move em passos unitários (cima/baixo/esquerda/direita) através de serviços ROS e tem acesso a **sensores locais** que informam o que existe nas células ao redor (livre, parede, alvo).


## Tecnologias utilizadas

- **ROS2** (Humble)  
- **C++**  
- **Simulador em Pygame** (pacote `cg`)  
- Pacotes ROS principais:
  - `cg` → simulador (janela do labirinto)  
  - `cg_teleop` → controle por teclado  
  - `cg_interfaces` → mensagens/serviços (`MoveCmd`, `Reset`, `RobotSensors`)  
  - `maze_navigator` → meu pacote com os nós de navegação (Parte 1 e Parte 2)


## Como rodar o projeto

### 1. Compilar o workspace

Na raiz do workspace (onde estão `src/`, `build/`, `install/`):

```bash
colcon build --symlink-install
source install/setup.bash
````

### 2. Rodar o simulador

Em um terminal:

```bash
source install/setup.bash
ros2 run cg maze
```

Isso abre a janela do Pygame com o labirinto, o robô azul e o alvo vermelho.

### 3. Parte 1 – Navegação com mapa conhecido

Em outro terminal:

```bash
source install/setup.bash
ros2 run maze_navigator maze_navigator_node
```

O robô irá:

1. Obter o **mapa completo** do labirinto via `/get_map`.
2. Obter sua posição inicial e a do alvo via `/move_command`.
3. Rodar **BFS** no grid do labirinto para encontrar o **caminho mínimo** até o alvo.
4. Enviar, passo a passo, comandos de movimento para o serviço `/move_command`
   (`"up"`, `"down"`, `"left"`, `"right"`).

Visualmente, você verá o robô **indo direto até o alvo pelo caminho mais curto**.

### 4. Parte 2 – Exploração + Mapa próprio + BFS do robô

Em outro terminal (com o simulador (`cg maze`) ainda rodando):

```bash
source install/setup.bash
ros2 run maze_navigator maze_mapper_node
```

O nó da Parte 2 será responsável por:

1. Explorar o labirinto usando **DFS com sensores**.
2. Construir um **mapa interno** só com o que o robô enxergou.
3. Fazer o robô **voltar para o início** (via reset).
4. Usar o **mesmo BFS da Parte 1**, mas rodando em cima do **mapa interno do robô**,
   e navegar do início até o alvo.

## Parte 1 – Navegação com mapa (BFS)

### Nó: `maze_navigator`

Na Parte 1, assumimos que o robô tem acesso ao **mapa completo** do labirinto.
O fluxo é:

1. **Obter mapa:**

   * Chama o serviço `/get_map` (`cg_interfaces/srv/GetMap`), que retorna:

     * `occupancy_grid_flattened`: vetor 1D de strings (`"f"` = livre, `"b"` = parede)
     * `occupancy_grid_shape`: `[height, width]`
   * Reconstruo o grid como uma matriz `height x width`.

2. **Obter posições:**

   * Chama `/move_command` com um comando “dummy” para receber:

     * `robot_pos` → `(start_row, start_col)`
     * `target_pos` → `(goal_row, goal_col)`

3. **Modelagem em grafo:**

   * Cada célula livre (`"f"`) é um **nó**.
   * Há uma **aresta** entre duas células vizinhas (cima, baixo, esquerda, direita) livres.

4. **BFS (Breadth-First Search):**

   * Aplico BFS no grid, a partir da célula do robô até a célula do alvo.
   * BFS garante o **menor caminho em número de passos** (cada passo vale 1).
   * Durante a BFS, guardo em `parent[row][col]`:

     * de onde vim (`parent_row`, `parent_col`)
     * qual direção usei (`UP`, `DOWN`, `LEFT`, `RIGHT`)

5. **Reconstrução da rota otimizada:**

   * Ando de trás pra frente (do alvo até o início), seguindo os `parent`.
   * Inverto a sequência para obter o caminho do início até o alvo.
   * Isso gera um vetor de `Direction` com o comando de cada passo.

6. **Execução no simulador:**

   * Para cada direção na rota:

     * Traduzo para `"up"`, `"down"`, `"left"`, `"right"`.
     * Chamo o serviço `/move_command` (`cg_interfaces/srv/MoveCmd`).
     * Coloco um `sleep` pequeno para visualizar o robô andando na tela.

> Em resumo, na Parte 1 o robô **planeja globalmente** usando BFS num mapa conhecido.


## Parte 2 – Exploração (DFS) + Mapa próprio + BFS

### Nó: `maze_mapper` 

Na Parte 2, o robô já **não pode usar o mapa pronto** para planejar desde o início.
Ele precisa:

1. **Explorar o labirinto com base nos sensores**
2. **Montar seu próprio mapa interno `map_`**
3. **Voltar ao início**
4. Só então aplicar o **BFS da Parte 1** em cima do mapa que ele mesmo construiu.

### 2.1 – Exploração com DFS (Depth-First Search)

* O nó cria uma estrutura `map_` (por exemplo `std::map<Cell, std::string>`) com:

  * `"f"` → livre
  * `"b"` → parede
  * `"t"` → target

* O robô mantém:

  * `current_pos_` → posição atual no grid
  * `visited_` → conjunto de células já exploradas
  * `dfs_stack_` → pilha de células para DFS

A cada iteração da exploração:

1. Chama `update_sensors()` e `map_surroundings()`:

   * Lê `cg_interfaces/msg/RobotSensors`:

     * `up`, `down`, `left`, `right`, `up_left`, etc.
   * Marca no `map_`:

     * células adjacentes que são parede (`"b"`)
     * células adjacentes que são livres (`"f"`)
     * se detectar alvo em uma direção cardinal, marca `"t"`

2. Tenta expandir a exploração (DFS) em 4 direções:

   ```cpp
   vizinhos = {UP, DOWN, LEFT, RIGHT}
   ```

   * Se vizinho não foi visitado e não é parede conhecida:

     * chama `try_move_exploration(direction)`
     * se o movimento for bem-sucedido (serviço `/move_command` responde `success`):

       * atualiza `current_pos_`
       * marca `map_[nova_pos] = "f"` (ou `"t"` se for o alvo)
       * marca `visited_.insert(current_pos_)`
       * empilha na `dfs_stack_`

3. Se nenhum vizinho novo puder ser visitado:

   * Faz **backtracking**:

     * tira o topo da pilha `dfs_stack_`
     * olha o novo topo (pai)
     * move o robô de volta pra ele usando `try_move_exploration()` com a direção inversa.

4. Critério de parada:

   * Quando a pilha volta a ter apenas a célula inicial
     → não há mais vizinhos livres não visitados
     → **DFS terminou** e estamos de volta ao início.

Durante toda a Fase 2.1:

* O robô **anda fisicamente no simulador**, explorando corredores e becos.
* O mapa `map_` vai sendo preenchido só com base nos **sensores + sucesso/falha dos movimentos**.

### 2.2 – Reset para o início

Depois de finalizar a exploração (DFS completa):

1. O nó chama o serviço `/reset` (`cg_interfaces/srv/Reset`) com:

   * `is_random = false`
   * `map_name = loaded_map_name` (o mesmo mapa)

2. Aguardamos o reset e, em seguida, chamamos `/move_command` com:

   * `direction = ""` (comando vazio só pra ler posição)

3. Assim, atualizamos `current_pos_` com a **posição inicial real** do robô pós-reset.

> Isso implementa o requisito:
> “Quando o robô termine a exploração, ele volta para o início...”

### 2.3 – BFS usando o mapa do robô (reutilizando o algoritmo da Parte 1)

Agora:

* Temos um mapa `map_` construído apenas com sensores.
* Estamos de volta no início (`initial_pos_` = `current_pos_`).
* Sabemos onde está o alvo (`target_pos_`).

O nó então:

1. Constrói uma matriz (`grid`) a partir de `map_`:

   * qualquer célula não mapeada vira `"b"` (desconhecida = tratada como parede)
   * `"f"` → célula livre
   * `"b"` → parede
   * `"t"` → alvo

2. Chama `bfs_find_path(height, width, grid, bfs_start, bfs_goal)`:

   * `bfs_start = current_pos_` (início após reset, corrigido para o sistema local do grid)
   * `bfs_goal = target_pos_` (posição do alvo no mesmo sistema)

3. O BFS encontra um **caminho ótimo** no mapa do robô.

4. O robô executa esse caminho com `try_move_navigation(direction)`:

   * para cada direção, envia `/move_command`
   * anda passo a passo até o alvo

Se tudo der certo, ao final teremos:

> O robô saiu do início e chegou ao alvo **usando somente o mapa que ele mesmo construiu na fase de exploração**.

Isso demonstra exatamente a frase:

> “...e percorra até o alvo com o algoritmo da parte 1, utilizando o mapa que ele mesmo criou.”


## Algoritmos de Grafos utilizados

### BFS – Breadth-First Search (Busca em Largura)

Usado em:

* **Parte 1**: para achar o caminho ótimo no mapa oficial (`/get_map`).
* **Parte 2 (Fase 2.2)**: para achar o caminho ótimo no mapa interno (`map_`) construído pelo robô.

Propriedades:

* Explora o grafo em “camadas” (nível 0, 1, 2...).
* Garante o **menor número de passos** entre início e alvo em grafos não ponderados.
* Complexidade: O(V + E), onde V = número de células, E = número de vizinhos.

### DFS – Depth-First Search (Busca em Profundidade)

Usado em:

* **Parte 2 (Fase 2.1)**: como estratégia de exploração.

Funcionamento:

* A partir de uma célula:

  * tenta sempre entrar em um vizinho livre **ainda não visitado**
  * se não há vizinho novo, **volta** (backtracking) para a célula anterior (pilha)
* Garante que todas as células alcançáveis a partir da origem serão visitadas.


## Considerações finais

* A **Parte 1** mostra que, com o mapa completo, um BFS simples é capaz de encontrar a rota ótima.
* A **Parte 2** mostra um cenário mais “robótico”:

  * o robô não tem o mapa pronto,
  * explora com sensores (DFS),
  * constrói seu próprio mapa,
  * volta ao início,
  * e só então aplica o BFS em cima do mapa aprendido.

Isso conecta:

* **Grafo + BFS** com navegação planejada,
* **DFS + sensores** com mapeamento autônomo,
* e o ciclo completo de **exploração → mapeamento → planejamento → navegação**.



