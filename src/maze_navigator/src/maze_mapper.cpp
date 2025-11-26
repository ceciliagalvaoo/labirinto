// Inclusão das bibliotecas necessárias do ROS2 e interfaces customizadas
#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/reset.hpp>

// Bibliotecas padrão do C++
#include <chrono>
#include <queue>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <set>
#include <map>
#include <stack>

// Permite usar literais de tempo como 100ms, 5s, etc
using namespace std::chrono_literals;

// DEFINIÇÕES DE GRID E DIREÇÕES

// Enumeração para representar as quatro direções cardinais possíveis
enum class Direction { UP, DOWN, LEFT, RIGHT };

// Estrutura que representa uma célula no grid do labirinto
struct Cell {
    int row;  // Linha (muda com UP/DOWN)
    int col;  // Coluna (muda com LEFT/RIGHT)
    
    // Operador de comparação menor que - usado para ordenar células em containers (set, map)
    bool operator<(const Cell& other) const {
        if (row != other.row) return row < other.row;
        return col < other.col;
    }
    
    // Operador de igualdade - verifica se duas células são a mesma posição
    bool operator==(const Cell& other) const {
        return row == other.row && col == other.col;
    }
    
    // Operador de desigualdade - verifica se duas células são diferentes
    bool operator!=(const Cell& other) const {
        return !(*this == other);
    }
};

// Estrutura que armazena informações sobre o pai de uma célula no caminho BFS
struct ParentInfo {
    int parent_row, parent_col;  // Posição da célula pai
    Direction dir;               // Direção tomada do pai até esta célula
    bool has_parent = false;     // Flag indicando se esta célula tem um pai válido
};

// Funções auxiliares para verificar o tipo de célula baseado na string
bool is_wall(const std::string &cell)   { return cell == "b"; }  // "b" = blocked (parede)
bool is_free(const std::string &cell)   { return cell == "f"; }  // "f" = free (livre)
bool is_target(const std::string &cell) { return cell == "t"; }  // "t" = target (alvo)

// BFS PARA ENCONTRAR O CAMINHO OTIMIZADO

// Função que reconstrói o caminho do início ao objetivo usando a estrutura de pais
std::vector<Direction> reconstruct_path(
    Cell start,                                           // Célula inicial
    Cell goal,                                            // Célula objetivo
    const std::vector<std::vector<ParentInfo>> &parent   // Matriz com informações dos pais
) {
    std::vector<Direction> reversed;  // Vetor para armazenar o caminho em ordem reversa
    Cell cur = goal;                  // Começar do objetivo e ir até o início

    // Percorrer de trás para frente usando os pais
    while (cur != start) {
        auto p = parent[cur.row][cur.col];  // Obter informações do pai da célula atual
        
        // Verificar se a célula tem um pai válido
        if (!p.has_parent) {
            std::cerr << "Reconstrucao falhou: sem pai em (" 
                      << cur.row << "," << cur.col << ")\n";
            break;
        }
        
        reversed.push_back(p.dir);  // Adicionar direção ao caminho reverso
        cur = {p.parent_row, p.parent_col};  // Mover para a célula pai
    }

    // Inverter o vetor para obter o caminho na ordem correta (início → fim)
    return std::vector<Direction>(reversed.rbegin(), reversed.rend());
}

// Função BFS (Busca em Largura) para encontrar o caminho mais curto
std::vector<Direction> bfs_find_path(
    int height,                            // Altura do grid
    int width,                             // Largura do grid
    const std::vector<std::string> &cells, // Vetor com todas as células do grid
    Cell start,                            // Posição inicial
    Cell goal                              // Posição objetivo
) {
    // Matriz para marcar células já visitadas
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    // Matriz para armazenar informações de pai de cada célula
    std::vector<std::vector<ParentInfo>> parent(height, std::vector<ParentInfo>(width));

    // Fila para o algoritmo BFS
    std::queue<Cell> q;
    q.push(start);
    visited[start.row][start.col] = true;  // Marcar início como visitado

    // Arrays para representar as 4 direções cardinais
    // Deslocamentos em linha: UP (-1), DOWN (+1), LEFT (0), RIGHT (0)
    const int drow[4] = {-1, 1, 0, 0};
    // Deslocamentos em coluna: UP (0), DOWN (0), LEFT (-1), RIGHT (+1)
    const int dcol[4] = {0, 0, -1, 1};
    // Direções correspondentes aos deslocamentos
    const Direction dirs[4] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

    bool found = false;  // Flag para indicar se o objetivo foi encontrado

    // Loop principal do BFS
    while (!q.empty() && !found) {
        Cell cur = q.front();  // Pegar próxima célula da fila
        q.pop();

        // Explorar todas as 4 direções cardinais
        for (int k = 0; k < 4; ++k) {
            // Calcular posição do vizinho
            int nrow = cur.row + drow[k];
            int ncol = cur.col + dcol[k];

            // Verificar se vizinho está dentro dos limites do grid
            if (nrow < 0 || nrow >= height || ncol < 0 || ncol >= width) continue;
            
            // Pular se já foi visitado
            if (visited[nrow][ncol]) continue;

            // Obter tipo da célula vizinha
            auto &cell = cells[nrow * width + ncol];
            // Pular se for parede
            if (is_wall(cell)) continue;

            // Marcar vizinho como visitado
            visited[nrow][ncol] = true;
            // Registrar informações do pai
            parent[nrow][ncol] = {cur.row, cur.col, dirs[k], true};

            // Verificar se chegamos ao objetivo
            if (nrow == goal.row && ncol == goal.col) {
                found = true;
                break;
            }

            // Adicionar vizinho à fila para exploração futura
            q.push({nrow, ncol});
        }
    }

    // Se não encontrou caminho, retornar vetor vazio
    if (!found) {
        std::cerr << "Nenhum caminho encontrado de ("
                  << start.row << "," << start.col << ") ate ("
                  << goal.row  << "," << goal.col  << ").\n";
        return {};
    }

    // Reconstruir e retornar o caminho encontrado
    return reconstruct_path(start, goal, parent);
}

// NÓ ROS2 - MAPEAMENTO E NAVEGAÇÃO

// Classe principal que herda de Node do ROS2
class MazeMapper : public rclcpp::Node {
public:
    // Construtor da classe
    MazeMapper()
    : rclcpp::Node("maze_mapper"),      // Inicializar nó com nome "maze_mapper"
      current_pos_{0, 0},               // Posição atual inicializada em (0,0)
      initial_pos_{0, 0},               // Posição inicial em (0,0)
      target_pos_{-1, -1},              // Target com posição inválida inicialmente
      found_target_(false)              // Flag de target encontrado como falso
    {
        // Criar cliente para o serviço de movimento
        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        // Criar cliente para o serviço de reset
        reset_client_ = this->create_client<cg_interfaces::srv::Reset>("/reset");
        
        // Criar subscrição ao tópico de sensores do robô com QoS SensorData
        // O QoS SensorData descarta valores antigos com mais frequência e mantém latência baixa
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
        sensor_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors",
            qos_profile,  // Usar QoS SensorData (otimizado para 100Hz)
            std::bind(&MazeMapper::sensor_callback, this, std::placeholders::_1)  // Callback
        );
        
        RCLCPP_INFO(this->get_logger(), "MazeMapper inicializado!");
    }

    // Função principal que executa todo o fluxo do programa
    void run() {
        // Tentar inicializar o sistema
        if (!initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Falha na inicializacao");
            return;
        }
        
        // Imprimir cabeçalho da Fase 1
        RCLCPP_INFO(this->get_logger(), "\n╔════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║  FASE 1: EXPLORAÇÃO COMPLETA (DFS)            ║");
        RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════╝");
        // Executar exploração completa do labirinto usando DFS
        explore_maze_complete();
        
        // Verificar se o target foi encontrado durante a exploração
        if (!found_target_) {
            RCLCPP_ERROR(this->get_logger(), 
                        "❌ Target não foi encontrado durante exploração!");
            return;
        }
        
        // Imprimir cabeçalho da Fase 2
        RCLCPP_INFO(this->get_logger(), "\n╔════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║  FASE 2: RESET E NAVEGAÇÃO ÓTIMA             ║");
        RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════╝");
        
        // Resetar robô para posição inicial
        reset_to_initial();
        // Navegar até o target usando o caminho ótimo (BFS)
        navigate_to_target();
    }

private:
    // Clientes ROS2 para serviços
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client_;
    // Subscrição para sensores
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    
    // Mapa do labirinto: mapeia posição (Cell) para tipo de célula ("f", "b", "t")
    std::map<Cell, std::string> map_;
    
    // Variáveis de estado do robô
    Cell current_pos_;   // Posição atual do robô
    Cell initial_pos_;   // Posição inicial do robô
    Cell target_pos_;    // Posição do target
    bool found_target_;  // Flag indicando se o target foi encontrado
    std::string loaded_map_name_;  // Nome do mapa carregado
    
    // Estruturas para o algoritmo DFS
    std::set<Cell> visited_;      // Conjunto de células já visitadas
    std::stack<Cell> dfs_stack_;  // Pilha para o algoritmo DFS
    
    // Ponteiro para os dados mais recentes dos sensores
    cg_interfaces::msg::RobotSensors::SharedPtr last_sensor_data_;
    
    // Callback chamado sempre que uma nova mensagem de sensores é recebida
    void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        last_sensor_data_ = msg;  // Armazenar os dados mais recentes dos sensores
    }
    
    // Função de inicialização do sistema
    bool initialize() {
        // Aguardar até que o serviço de movimento esteja disponível (timeout de 5 segundos)
        if (!move_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /move_command nao disponivel");
            return false;
        }
        
        // Aguardar até que o serviço de reset esteja disponível (timeout de 5 segundos)
        if (!reset_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /reset nao disponivel");
            return false;
        }
        
        // Aguardar dados dos sensores
        RCLCPP_INFO(this->get_logger(), "Aguardando dados dos sensores...");
        // Tentar 50 vezes com intervalos de 100ms (total de 5 segundos)
        for (int i = 0; i < 50 && !last_sensor_data_; ++i) {
            rclcpp::sleep_for(100ms);  // Esperar 100ms
            rclcpp::spin_some(shared_from_this());  // Processar callbacks pendentes
        }
        
        // Verificar se conseguiu receber dados dos sensores
        if (!last_sensor_data_) {
            RCLCPP_ERROR(this->get_logger(), "Timeout aguardando sensores");
            return false;
        }
        
        // Primeiro, fazer um reset para garantir estado inicial limpo
        RCLCPP_INFO(this->get_logger(), "Fazendo reset inicial...");
        auto reset_req = std::make_shared<cg_interfaces::srv::Reset::Request>();
        reset_req->is_random = false;  // Não usar mapa aleatório
        reset_req->map_name = "";  // Usar mapa atual (string vazia)
        
        // Enviar requisição de reset de forma assíncrona
        auto reset_future = reset_client_->async_send_request(reset_req);
        // Aguardar resposta do serviço (timeout de 5 segundos)
        if (rclcpp::spin_until_future_complete(shared_from_this(), reset_future, 5s)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto reset_resp = reset_future.get();  // Obter resposta
            if (reset_resp->success) {
                loaded_map_name_ = reset_resp->loaded_map_name;  // Armazenar nome do mapa
                RCLCPP_INFO(this->get_logger(), "Reset inicial OK, mapa: %s", loaded_map_name_.c_str());
                rclcpp::sleep_for(500ms);  // Esperar 500ms para estabilizar
                // Limpar e atualizar sensores
                last_sensor_data_.reset();  // Resetar dados antigos
                // Processar novos dados de sensores
                for (int i = 0; i < 5; ++i) {
                    rclcpp::spin_some(shared_from_this());
                    rclcpp::sleep_for(100ms);
                }
            }
        }
        
        // Obter posição inicial fazendo uma requisição "vazia" (sem movimento)
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = "";  // String vazia = apenas consultar estado
        
        // Enviar requisição de forma assíncrona
        auto future = move_client_->async_send_request(req);
        // Aguardar resposta
        if (rclcpp::spin_until_future_complete(shared_from_this(), future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            return false;  // Falha na comunicação
        }
        
        auto resp = future.get();  // Obter resposta do serviço
        // Verificar se a resposta contém as posições válidas
        if (resp->robot_pos.size() < 2 || resp->target_pos.size() < 2) {
            return false;  // Dados inválidos
        }
        
        // Simulador retorna posições no formato [row, col]
        current_pos_ = {(int)resp->robot_pos[0], (int)resp->robot_pos[1]};
        initial_pos_ = current_pos_;  // Salvar posição inicial
        target_pos_ = {(int)resp->target_pos[0], (int)resp->target_pos[1]};
        
        // Imprimir informações de posição
        RCLCPP_INFO(this->get_logger(), 
                    "Inicio: (%d,%d), Alvo: (%d,%d)",
                    current_pos_.row, current_pos_.col,
                    target_pos_.row, target_pos_.col);
        
        return true;  // Inicialização bem-sucedida
    }
    
    // Função para atualizar dados dos sensores
    void update_sensors() {
        // Com 100Hz (10ms por mensagem), aguardar tempo suficiente para dados atualizados
        rclcpp::sleep_for(300ms);  // Esperar 300ms para sensores atualizarem
        rclcpp::spin_some(shared_from_this());  // Processar callbacks (inclui sensor_callback)
        rclcpp::spin_some(shared_from_this());  // Processar novamente para garantir
    }
    
    // Função que mapeia as células ao redor da posição atual usando sensores
    void map_surroundings() {
        if (!last_sensor_data_) return;  // Verificar se há dados de sensores disponíveis
        
        // Mapear célula atual como livre (confirmado por estar nela)
        map_[current_pos_] = "f";
        
        // Estrutura auxiliar para organizar informações dos vizinhos
        struct Neighbor {
            int drow, dcol;           // Deslocamento em linha e coluna
            std::string sensor_value; // Valor lido pelo sensor para esta posição
            bool is_cardinal;         // Se é direção cardinal (não diagonal)
        };
        
        // IMPORTANTE: Marcar quais sensores são cardinais
        // Apenas direções cardinais permitem movimento real
        std::vector<Neighbor> neighbors = {
            {-1, 0, last_sensor_data_->up, true},           // UP - cardinal
            {1, 0, last_sensor_data_->down, true},          // DOWN - cardinal
            {0, -1, last_sensor_data_->left, true},         // LEFT - cardinal
            {0, 1, last_sensor_data_->right, true},         // RIGHT - cardinal
            {-1, -1, last_sensor_data_->up_left, false},    // UP_LEFT - diagonal
            {-1, 1, last_sensor_data_->up_right, false},    // UP_RIGHT - diagonal
            {1, -1, last_sensor_data_->down_left, false},   // DOWN_LEFT - diagonal
            {1, 1, last_sensor_data_->down_right, false}    // DOWN_RIGHT - diagonal
        };
        
        // Processar cada vizinho
        for (const auto& n : neighbors) {
            // Calcular posição do vizinho
            Cell neighbor = {current_pos_.row + n.drow, current_pos_.col + n.dcol};
            
            // Se já conhecemos essa célula de movimento real, não sobrescrever
            auto it = map_.find(neighbor);
            if (it != map_.end()) {
                // Já foi validada por movimento real, manter o valor
                continue;
            }
            
            // SÓ CONSIDERAR TARGET SE ESTIVER NAS 4 DIREÇÕES CARDINAIS
            // (porque só podemos nos mover nessas direções)
            if (n.sensor_value == "t" && n.is_cardinal) {
                map_[neighbor] = "t";  // Mapear como target
                
                // ESTRATÉGIA: Aceitar target em até 2 células de distância
                // (tolerância para pequenos erros de sensores)
                if (!found_target_) {
                    // Calcular distância de Manhattan até o target esperado
                    int dist = std::abs(neighbor.row - target_pos_.row) + 
                               std::abs(neighbor.col - target_pos_.col);
                    
                    if (dist == 0) {
                        // Posição exata! Perfeito
                        found_target_ = true;
                        RCLCPP_INFO(this->get_logger(), 
                                   "TARGET CONFIRMADO em (%d,%d)!", 
                                   neighbor.row, neighbor.col);
                    } else if (dist <= 2) {
                        // Próximo ao target esperado - aceitar mas avisar
                        found_target_ = true;
                        RCLCPP_WARN(this->get_logger(),
                                   "TARGET detectado em (%d,%d) (esperado: %d,%d, dist=%d) - aceitando",
                                   neighbor.row, neighbor.col,
                                   target_pos_.row, target_pos_.col, dist);
                    } else {
                        // Muito longe - provável erro do sensor
                        RCLCPP_WARN(this->get_logger(),
                                   "Sensor detectou 't' em (%d,%d) mas target real é (%d,%d) (dist=%d) - ignorando!",
                                   neighbor.row, neighbor.col,
                                   target_pos_.row, target_pos_.col, dist);
                    }
                }
            }
            // NÃO mapear mais nada dos sensores - só movimento confirma!
        }
    }
    
    // Função para EXPLORAÇÃO (DFS) - pode corrigir o mapa baseado em movimento real
    bool try_move_exploration(Direction dir) {
        // Criar requisição de movimento
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        
        // Converter direção enum para string
        switch (dir) {
            case Direction::UP:    req->direction = "up"; break;
            case Direction::DOWN:  req->direction = "down"; break;
            case Direction::LEFT:  req->direction = "left"; break;
            case Direction::RIGHT: req->direction = "right"; break;
        }
        
        // Calcular célula de destino baseado na direção
        Cell target_cell = current_pos_;
        if (dir == Direction::UP) target_cell.row--;
        else if (dir == Direction::DOWN) target_cell.row++;
        else if (dir == Direction::LEFT) target_cell.col--;
        else if (dir == Direction::RIGHT) target_cell.col++;
        
        // Enviar requisição de movimento de forma assíncrona
        auto future = move_client_->async_send_request(req);
        // Aguardar resposta
        if (rclcpp::spin_until_future_complete(shared_from_this(), future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            return false;  // Falha na comunicação
        }
        
        auto resp = future.get();  // Obter resposta
        
        // Verificar se movimento foi bem-sucedido
        if (resp->success && resp->robot_pos.size() >= 2) {
            Cell old_pos = current_pos_;  // Salvar posição antiga
            current_pos_ = {(int)resp->robot_pos[0], (int)resp->robot_pos[1]};  // Atualizar posição
            
            // CONFIRMAR que célula destino é livre (EXCETO se for o target!)
            if (!(target_cell.row == target_pos_.row && target_cell.col == target_pos_.col)) {
                map_[target_cell] = "f";  // Marcar como livre
            }
            
            // CONFIRMAR que célula atual também é livre (EXCETO se for o target!)
            if (!(current_pos_.row == target_pos_.row && current_pos_.col == target_pos_.col)) {
                map_[current_pos_] = "f";  // Marcar como livre
            } else {
                // Chegamos no target! Marcar como encontrado
                if (!found_target_) {
                    found_target_ = true;
                    map_[current_pos_] = "t";  // Marcar como target
                    RCLCPP_INFO(this->get_logger(),
                               "🎯 TARGET ALCANÇADO por movimento! Posição: (%d,%d)",
                               current_pos_.row, current_pos_.col);
                }
            }
            
            // Log do movimento bem-sucedido
            RCLCPP_INFO(this->get_logger(), 
                       "✅ Moveu: (%d,%d) → (%d,%d) [%s]",
                       old_pos.row, old_pos.col,
                       current_pos_.row, current_pos_.col,
                       req->direction.c_str());
            
            update_sensors();  // Atualizar sensores após movimento
            return true;
        }
        
        // Movimento falhou - CONFIRMAR que é parede
        RCLCPP_DEBUG(this->get_logger(), 
                    "❌ Bloqueado tentando ir para (%d,%d)", 
                    target_cell.row, target_cell.col);
        
        // Corrigir mapa se estava errado
        auto it = map_.find(target_cell);
        if (it != map_.end() && it->second != "b") {
            // Célula estava mapeada incorretamente, corrigir
            RCLCPP_WARN(this->get_logger(),
                       "CORREÇÃO: Célula (%d,%d) estava como '%s', agora é 'b'",
                       target_cell.row, target_cell.col, it->second.c_str());
        }
        
        map_[target_cell] = "b";  // Marcar como parede (blocked)
        
        return false;  // Movimento falhou
    }
    
    // Função para NAVEGAÇÃO (BFS) - NÃO modifica o mapa, apenas move
    bool try_move_navigation(Direction dir) {
        // IMPORTANTE: Atualizar sensores ANTES de mover
        update_sensors();
        
        // Criar requisição de movimento
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        
        // Converter direção enum para string
        switch (dir) {
            case Direction::UP:    req->direction = "up"; break;
            case Direction::DOWN:  req->direction = "down"; break;
            case Direction::LEFT:  req->direction = "left"; break;
            case Direction::RIGHT: req->direction = "right"; break;
        }
        
        // Log de debug antes de enviar comando
        RCLCPP_DEBUG(this->get_logger(),
                    "Enviando comando: '%s' | Posição atual antes do comando: (%d,%d)",
                    req->direction.c_str(), current_pos_.row, current_pos_.col);
        
        // Enviar requisição de movimento de forma assíncrona
        auto future = move_client_->async_send_request(req);
        // Aguardar resposta
        if (rclcpp::spin_until_future_complete(shared_from_this(), future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Timeout ao enviar comando de movimento!");
            return false;
        }
        
        auto resp = future.get();  // Obter resposta
        
        // Log de debug da resposta recebida
        RCLCPP_DEBUG(this->get_logger(),
                    "Resposta recebida: success=%d, robot_pos.size()=%zu",
                    resp->success, resp->robot_pos.size());
        
        // Verificar se movimento foi bem-sucedido
        if (resp->success && resp->robot_pos.size() >= 2) {
            Cell old_pos = current_pos_;  // Salvar posição antiga
            Cell new_pos = {(int)resp->robot_pos[0], (int)resp->robot_pos[1]};  // Nova posição
            
            // Log de debug da nova posição
            RCLCPP_DEBUG(this->get_logger(),
                        "📍 Resposta do serviço: robot_pos=[%d,%d]",
                        new_pos.row, new_pos.col);
            
            current_pos_ = new_pos;  // Atualizar posição atual
            
            // Log do movimento bem-sucedido
            RCLCPP_INFO(this->get_logger(), 
                       "✅ Moveu: (%d,%d) → (%d,%d) [%s]",
                       old_pos.row, old_pos.col,
                       current_pos_.row, current_pos_.col,
                       req->direction.c_str());
            
            return true;
        }
        
        // Movimento falhou durante navegação - NÃO deve acontecer!
        // (porque o caminho BFS deve estar correto baseado no mapa explorado)
        RCLCPP_ERROR(this->get_logger(), 
                    "❌ Falha crítica: movimento bloqueado durante navegação BFS!");
        RCLCPP_ERROR(this->get_logger(),
                    "   Resposta: success=%d, robot_pos.size()=%zu",
                    resp->success, resp->robot_pos.size());
        
        return false;
    }

    // Função que explora o labirinto completamente usando algoritmo DFS (Depth-First Search)
    void explore_maze_complete() {
        RCLCPP_INFO(this->get_logger(), "Iniciando exploração DFS até encontrar target...");
        
        // Inicializar DFS
        visited_.insert(current_pos_);  // Marcar posição inicial como visitada
        dfs_stack_.push(current_pos_);  // Adicionar posição inicial à pilha
        map_surroundings();  // Mapear células ao redor da posição inicial
        
        // VERIFICAR SE JÁ DETECTOU TARGET NA POSIÇÃO INICIAL
        if (found_target_) {
            RCLCPP_INFO(this->get_logger(), 
                       "TARGET DETECTADO NA POSIÇÃO INICIAL! Parando exploração.");
            save_map_to_file();  // Salvar mapa em arquivo
            print_map();  // Imprimir mapa no console
            return;
        }
        
        int steps = 0;  // Contador de passos
        const int MAX_STEPS = 10000;  // Limite máximo de passos para evitar loop infinito
        
        // Loop principal do DFS
        while (!dfs_stack_.empty() && steps < MAX_STEPS) {
            // VERIFICAR SE ENCONTROU TARGET ANTES DE CONTINUAR
            if (found_target_) {
                RCLCPP_INFO(this->get_logger(), 
                           "\nTARGET DETECTADO! Interrompendo exploração...");
                RCLCPP_INFO(this->get_logger(), 
                           "   Target está em: (%d,%d)", target_pos_.row, target_pos_.col);
                RCLCPP_INFO(this->get_logger(), 
                           "   Robô está em: (%d,%d)", current_pos_.row, current_pos_.col);
                break;  // Sair do loop
            }
            
            map_surroundings();  // Mapear células ao redor da posição atual
            
            // VERIFICAR NOVAMENTE APÓS MAPEAR (pode ter detectado agora)
            if (found_target_) {
                RCLCPP_INFO(this->get_logger(), 
                           "\nTARGET DETECTADO PELOS SENSORES! Parando exploração.");
                break;  // Sair do loop
            }
            
            // Tentar encontrar vizinho não visitado (4 direções principais)
            // Lista de vizinhos possíveis com suas direções e posições
            std::vector<std::pair<Direction, Cell>> neighbors = {
                {Direction::UP, {current_pos_.row - 1, current_pos_.col}},    // Vizinho acima
                {Direction::DOWN, {current_pos_.row + 1, current_pos_.col}},  // Vizinho abaixo
                {Direction::LEFT, {current_pos_.row, current_pos_.col - 1}},  // Vizinho à esquerda
                {Direction::RIGHT, {current_pos_.row, current_pos_.col + 1}}  // Vizinho à direita
            };
            
            bool moved = false;  // Flag indicando se conseguiu mover
            
            // Tentar mover para cada vizinho
            for (const auto& [dir, neighbor] : neighbors) {
                // NÃO MOVER PARA O TARGET!
                // (queremos apenas detectá-lo, não entrar nele durante exploração)
                if (neighbor == target_pos_) {
                    RCLCPP_INFO(this->get_logger(), 
                               "DETECTADO: Target está adjacente em (%d,%d)! Marcando como encontrado.", 
                               neighbor.row, neighbor.col);
                    visited_.insert(neighbor);  // Marcar como visitado
                    map_[neighbor] = "t";  // Garantir que está mapeado como target
                    found_target_ = true;  // Marcar como encontrado!
                    continue;  // Não tentar mover para o target
                }
                
                // Verificar se já visitamos este vizinho
                if (visited_.find(neighbor) != visited_.end()) continue;
                
                // Verificar se já sabemos que é parede
                auto it = map_.find(neighbor);
                if (it != map_.end() && it->second == "b") {
                    visited_.insert(neighbor);  // Marcar como visitado para não tentar de novo
                    continue;
                }
                
                // Tentar mover para o vizinho
                if (try_move_exploration(dir)) {
                    visited_.insert(current_pos_);  // Marcar nova posição como visitada
                    dfs_stack_.push(current_pos_);  // Adicionar à pilha do DFS
                    moved = true;  // Indicar que moveu com sucesso
                    steps++;  // Incrementar contador de passos
                    
                    // A cada 50 passos, imprimir estatísticas
                    if (steps % 50 == 0) {
                        RCLCPP_INFO(this->get_logger(), 
                                   "Exploração: %d passos, %zu visitadas, %zu mapeadas",
                                   steps, visited_.size(), map_.size());
                    }
                    break;  // Sair do loop de vizinhos após movimento bem-sucedido
                }
            }
            
            // Se não conseguiu mover para nenhum vizinho, fazer backtrack
            if (!moved) {
                // Backtrack - remover posição atual da pilha
                dfs_stack_.pop();
                
                // Se ainda há células na pilha, voltar para a célula pai
                if (!dfs_stack_.empty()) {
                    Cell target = dfs_stack_.top();  // Obter célula pai (topo da pilha)
                    
                    RCLCPP_DEBUG(this->get_logger(), 
                                "⬅Backtrack para (%d,%d)", target.row, target.col);
                    
                    // Mover para o pai baseado na diferença de posição
                    if (target.row < current_pos_.row) try_move_exploration(Direction::UP);
                    else if (target.row > current_pos_.row) try_move_exploration(Direction::DOWN);
                    else if (target.col < current_pos_.col) try_move_exploration(Direction::LEFT);
                    else if (target.col > current_pos_.col) try_move_exploration(Direction::RIGHT);
                    
                    // IMPORTANTE: Mapear após backtrack também!
                    map_surroundings();
                    
                    steps++;  // Incrementar contador (backtrack também conta)
                }
            }
        }
        
        // Imprimir resumo da exploração
        RCLCPP_INFO(this->get_logger(), 
                    "\nEXPLORAÇÃO CONCLUÍDA:");
        RCLCPP_INFO(this->get_logger(), 
                    "   • Passos: %d", steps);
        RCLCPP_INFO(this->get_logger(), 
                    "   • Células visitadas: %zu", visited_.size());
        RCLCPP_INFO(this->get_logger(), 
                    "   • Células mapeadas: %zu", map_.size());
        RCLCPP_INFO(this->get_logger(), 
                    "   • Target encontrado: %s", found_target_ ? "SIM ✓" : "NÃO ✗");
        
        // Mapear posição final uma última vez
        map_surroundings();
        
        // VERIFICAÇÃO FINAL: Se o target está no mapa mas não foi marcado como encontrado
        if (!found_target_) {
            // Verificar se a célula do target está mapeada
            auto it = map_.find(target_pos_);
            if (it != map_.end()) {
                RCLCPP_WARN(this->get_logger(),
                           "\nTarget estava no mapa mas não foi marcado como encontrado!");
                RCLCPP_INFO(this->get_logger(),
                           "   Célula (%d,%d) está mapeada como: '%s'",
                           target_pos_.row, target_pos_.col, it->second.c_str());
                
                // Se está mapeado (independente do valor), considerar encontrado
                found_target_ = true;
                map_[target_pos_] = "t";  // Garantir que está como target
                RCLCPP_INFO(this->get_logger(),
                           "✅ Marcando como encontrado baseado no mapa!");
            } else {
                // Target não está no mapa - aviso crítico
                RCLCPP_WARN(this->get_logger(), 
                           "\nAVISO: Target não foi detectado durante exploração!");
                RCLCPP_WARN(this->get_logger(), 
                           "   Pode estar em área não explorada ou sensores falharam.");
            }
        }
        
        // Salvar mapa em arquivo e imprimir no console
        save_map_to_file();
        print_map();
    }

    // Função que reseta o robô para a posição inicial
    bool reset_to_initial() {
        // Imprimir cabeçalho decorativo
        RCLCPP_INFO(this->get_logger(), 
                   "\n╔════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), 
                   "║  RESETANDO PARA POSIÇÃO INICIAL               ║");
        RCLCPP_INFO(this->get_logger(), 
                   "╚════════════════════════════════════════════════╝");
        
        RCLCPP_INFO(this->get_logger(), 
                   "Chamando serviço /reset...");
        
        // ESTRATÉGIA 1: Tentar resetar usando o nome do mapa explicitamente
        auto req = std::make_shared<cg_interfaces::srv::Reset::Request>();
        req->is_random = false;  // Não usar mapa aleatório
        req->map_name = loaded_map_name_;  // Usar nome do mapa salvo anteriormente
        
        RCLCPP_INFO(this->get_logger(),
                   "   Resetando mapa: '%s'", loaded_map_name_.c_str());
        
        // Enviar requisição de reset de forma assíncrona
        auto future = reset_client_->async_send_request(req);
        // Aguardar resposta com timeout de 5 segundos
        if (rclcpp::spin_until_future_complete(shared_from_this(), future, 5s)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Falha ao chamar serviço /reset");
            return false;
        }
        
        auto resp = future.get();  // Obter resposta do serviço
        
        // Verificar se o reset foi bem-sucedido
        if (!resp->success) {
            RCLCPP_ERROR(this->get_logger(), "❌ Reset falhou!");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ Reset bem-sucedido!");
        RCLCPP_INFO(this->get_logger(), 
                   "   Mapa: %s", resp->loaded_map_name.c_str());
        
        // IMPORTANTE: Limpar dados antigos dos sensores
        last_sensor_data_.reset();
        
        // Aguardar e atualizar sensores MÚLTIPLAS vezes para garantir dados frescos
        RCLCPP_INFO(this->get_logger(), "Atualizando sensores após reset...");
        rclcpp::sleep_for(300ms);  // Esperar 300ms para estabilizar
        
        // Processar callbacks múltiplas vezes
        for (int i = 0; i < 5; ++i) {
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(100ms);
        }
        
        // Verificar se conseguiu receber dados dos sensores
        if (!last_sensor_data_) {
            RCLCPP_WARN(this->get_logger(), "Sensores ainda não recebidos após reset!");
        }
        
        // OBTER POSIÇÃO ATUAL DO ROBÔ APÓS RESET
        // Fazer uma chamada ao /move_command com direção vazia para confirmar posição
        RCLCPP_INFO(this->get_logger(), "Verificando posição real do robô...");
        
        auto move_req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        move_req->direction = "";  // Direção vazia = apenas consultar estado
        
        // Enviar requisição de forma assíncrona
        auto move_future = move_client_->async_send_request(move_req);
        // Aguardar resposta
        if (rclcpp::spin_until_future_complete(shared_from_this(), move_future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Falha ao verificar posição após reset");
            return false;
        }
        
        auto move_resp = move_future.get();  // Obter resposta
        
        // Verificar se a resposta contém dados válidos
        if (move_resp->robot_pos.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "❌ Resposta inválida ao verificar posição");
            return false;
        }
        
        // Imprimir informações da resposta
        RCLCPP_INFO(this->get_logger(),
                   "Resposta do /move_command (vazio):");
        RCLCPP_INFO(this->get_logger(),
                   "   robot_pos = [%d, %d]",
                   (int)move_resp->robot_pos[0], (int)move_resp->robot_pos[1]);
        
        // Imprimir posição do target se disponível
        if (move_resp->target_pos.size() >= 2) {
            RCLCPP_INFO(this->get_logger(),
                       "   target_pos = [%d, %d]",
                       (int)move_resp->target_pos[0], (int)move_resp->target_pos[1]);
        }
        
        // ATUALIZAR POSIÇÃO COM BASE NA RESPOSTA REAL
        current_pos_ = {(int)move_resp->robot_pos[0], (int)move_resp->robot_pos[1]};
        
        RCLCPP_INFO(this->get_logger(), 
                   "   ✅ Posição confirmada: (%d,%d)", 
                   current_pos_.row, current_pos_.col);
        
        // VERIFICAÇÃO CRÍTICA: Se não está na posição inicial, tentar reset novamente!
        int reset_attempts = 0;  // Contador de tentativas
        const int MAX_RESET_ATTEMPTS = 3;  // Máximo de 3 tentativas
        
        // Loop para tentar resetar até conseguir voltar à posição inicial
        while ((current_pos_.row != initial_pos_.row || current_pos_.col != initial_pos_.col) 
               && reset_attempts < MAX_RESET_ATTEMPTS) 
        {
            RCLCPP_WARN(this->get_logger(),
                       "AVISO: Posição após reset (%d,%d) é diferente da inicial esperada (%d,%d)",
                       current_pos_.row, current_pos_.col,
                       initial_pos_.row, initial_pos_.col);
            
            reset_attempts++;  // Incrementar contador de tentativas
            RCLCPP_WARN(this->get_logger(),
                       "Tentativa %d/%d: Tentando reset novamente...",
                       reset_attempts, MAX_RESET_ATTEMPTS);
            
            // Tentar reset novamente
            auto retry_req = std::make_shared<cg_interfaces::srv::Reset::Request>();
            retry_req->is_random = false;
            retry_req->map_name = loaded_map_name_;
            
            // Enviar requisição de reset
            auto retry_future = reset_client_->async_send_request(retry_req);
            // Aguardar resposta
            if (rclcpp::spin_until_future_complete(shared_from_this(), retry_future, 5s)
                == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto retry_resp = retry_future.get();
                if (retry_resp->success) {
                    RCLCPP_INFO(this->get_logger(), "   Reset repetido OK!");
                    
                    // Limpar e atualizar sensores
                    last_sensor_data_.reset();
                    rclcpp::sleep_for(300ms);
                    for (int i = 0; i < 5; ++i) {
                        rclcpp::spin_some(shared_from_this());
                        rclcpp::sleep_for(100ms);
                    }
                    
                    // Verificar posição novamente
                    auto check_req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
                    check_req->direction = "";  // Apenas consultar
                    
                    auto check_future = move_client_->async_send_request(check_req);
                    if (rclcpp::spin_until_future_complete(shared_from_this(), check_future)
                        == rclcpp::FutureReturnCode::SUCCESS)
                    {
                        auto check_resp = check_future.get();
                        if (check_resp->robot_pos.size() >= 2) {
                            // Atualizar posição atual
                            current_pos_ = {(int)check_resp->robot_pos[0], (int)check_resp->robot_pos[1]};
                            RCLCPP_INFO(this->get_logger(),
                                       "   Nova posição: (%d,%d)", current_pos_.row, current_pos_.col);
                        }
                    }
                }
            }
        }
        
        // Verificar se conseguiu resetar corretamente após todas as tentativas
        if (current_pos_.row != initial_pos_.row || current_pos_.col != initial_pos_.col) {
            RCLCPP_ERROR(this->get_logger(),
                        "ERRO CRÍTICO: Não foi possível resetar para a posição inicial!");
            RCLCPP_ERROR(this->get_logger(),
                        "   Esperado: (%d,%d) | Atual: (%d,%d)",
                        initial_pos_.row, initial_pos_.col,
                        current_pos_.row, current_pos_.col);
            RCLCPP_ERROR(this->get_logger(),
                        "   O serviço /reset pode estar com bug!");
            return false;
        }
        
        // Imprimir estado final após reset bem-sucedido
        RCLCPP_INFO(this->get_logger(),
                   "Estado final após reset:");
        RCLCPP_INFO(this->get_logger(),
                   "   current_pos_ = (%d,%d)",
                   current_pos_.row, current_pos_.col);
        RCLCPP_INFO(this->get_logger(),
                   "   initial_pos_ = (%d,%d)",
                   initial_pos_.row, initial_pos_.col);
        RCLCPP_INFO(this->get_logger(),
                   "   target_pos_ = (%d,%d)",
                   target_pos_.row, target_pos_.col);
        
        return true;  // Reset bem-sucedido
    }
    
    // Função que navega do ponto atual até o target usando o caminho ótimo (BFS)
    void navigate_to_target() {
        // IMPORTANTE: Atualizar sensores ANTES de começar a navegação
        RCLCPP_INFO(this->get_logger(), "Atualizando sensores antes da navegação...");
        update_sensors();  // Primeira atualização
        rclcpp::sleep_for(200ms);  // Esperar 200ms
        update_sensors();  // Segunda atualização para garantir
        
        // Imprimir cabeçalho decorativo
        RCLCPP_INFO(this->get_logger(), 
                   "\n╔════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), 
                   "║  NAVEGANDO PARA O TARGET (BFS)                ║");
        RCLCPP_INFO(this->get_logger(), 
                   "╚════════════════════════════════════════════════╝");
        
        RCLCPP_INFO(this->get_logger(),
                   "Posição ANTES da navegação BFS: (%d,%d)",
                   current_pos_.row, current_pos_.col);
        
        // Calcular dimensões do grid baseado no mapa explorado
        int min_row = 9999, max_row = -9999;
        int min_col = 9999, max_col = -9999;
        
        // Encontrar limites mínimos e máximos do mapa
        for (const auto& [cell, _] : map_) {
            min_row = std::min(min_row, cell.row);
            max_row = std::max(max_row, cell.row);
            min_col = std::min(min_col, cell.col);
            max_col = std::max(max_col, cell.col);
        }
        
        // Calcular altura e largura do grid
        int height = max_row - min_row + 1;
        int width = max_col - min_col + 1;
        
        // Criar grid para o BFS
        // Células desconhecidas permanecem como paredes (padrão "b")
        std::vector<std::string> grid(height * width, "b");
        
        // Preencher grid com dados do mapa explorado
        for (const auto& [cell, value] : map_) {
            // Converter coordenadas absolutas para coordenadas relativas do grid
            int r = cell.row - min_row;
            int c = cell.col - min_col;
            grid[r * width + c] = value;  // Copiar valor do mapa
        }
        
        // Converter posições inicial e final para coordenadas do grid
        Cell bfs_start = {current_pos_.row - min_row, current_pos_.col - min_col};
        Cell bfs_goal = {target_pos_.row - min_row, target_pos_.col - min_col};
        
        // Imprimir informações do grid
        RCLCPP_INFO(this->get_logger(), 
                    "Grid: %dx%d", height, width);
        RCLCPP_INFO(this->get_logger(), 
                    "Start: (%d,%d) | Goal: (%d,%d)",
                    current_pos_.row, current_pos_.col,
                    target_pos_.row, target_pos_.col);
        
        // Executar algoritmo BFS para encontrar caminho ótimo
        auto path = bfs_find_path(height, width, grid, bfs_start, bfs_goal);
        
        // Verificar se encontrou um caminho
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ BFS não encontrou caminho!");
            save_debug_grid(height, width, grid, bfs_start, bfs_goal);  // Salvar grid para debug
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), 
                    "✓ Caminho ótimo: %zu passos", path.size());
        
        // Executar o caminho encontrado
        execute_path(path);
        
        // Verificar se chegou no target
        if (current_pos_ == target_pos_) {
            // Imprimir mensagem de sucesso com estatísticas
            RCLCPP_INFO(this->get_logger(), 
                       "\n╔════════════════════════════════════════════════╗");
            RCLCPP_INFO(this->get_logger(), 
                       "║     🎯 TARGET ALCANÇADO COM SUCESSO! 🎯       ║");
            RCLCPP_INFO(this->get_logger(), 
                       "║                                               ║");
            RCLCPP_INFO(this->get_logger(), 
                       "║  Células visitadas na exploração: %-3zu        ║", 
                       visited_.size());
            RCLCPP_INFO(this->get_logger(), 
                       "║  Células mapeadas: %-3zu                       ║", 
                       map_.size());
            RCLCPP_INFO(this->get_logger(), 
                       "║  Passos no caminho ótimo: %-3zu                ║", 
                       path.size());
            RCLCPP_INFO(this->get_logger(), 
                       "╚════════════════════════════════════════════════╝");
        } else {
            // Aviso se não chegou exatamente no target
            RCLCPP_WARN(this->get_logger(),
                       "Parou em (%d,%d) mas target está em (%d,%d)",
                       current_pos_.row, current_pos_.col,
                       target_pos_.row, target_pos_.col);
        }
    }
    
    // Função que salva o grid em arquivo para debug quando BFS falha
    void save_debug_grid(int height, int width, 
                         const std::vector<std::string>& grid,
                         Cell start, Cell goal) {
        std::ofstream file("debug_grid.txt");  // Criar arquivo
        // Escrever cabeçalho com informações do grid
        file << "DEBUG GRID BFS - " << height << "x" << width << "\n";
        file << "Start: (" << start.row << "," << start.col << ")\n";
        file << "Goal: (" << goal.row << "," << goal.col << ")\n\n";
        
        // Percorrer todo o grid e escrever representação visual
        for (int r = 0; r < height; ++r) {
            for (int c = 0; c < width; ++c) {
                // Marcar posição inicial com 'S'
                if (r == start.row && c == start.col) {
                    file << "S";
                } 
                // Marcar posição final com 'G'
                else if (r == goal.row && c == goal.col) {
                    file << "G";
                } 
                // Para outras células, usar símbolo baseado no tipo
                else {
                    std::string cell = grid[r * width + c];
                    if (cell == "b") file << "#";       // Parede
                    else if (cell == "f") file << ".";  // Livre
                    else if (cell == "t") file << "T";  // Target
                    else file << "?";                   // Desconhecido
                }
            }
            file << "\n";  // Nova linha ao fim de cada linha do grid
        }
        file.close();  // Fechar arquivo
        RCLCPP_ERROR(this->get_logger(), "Grid debug salvo: debug_grid.txt");
    }
    
    // Função que executa uma sequência de movimentos (caminho)
    void execute_path(const std::vector<Direction>& path) {
        RCLCPP_INFO(this->get_logger(), 
                   "\nIniciando execução do caminho (%zu passos)...", path.size());
        
        // Executar cada movimento do caminho
        for (size_t i = 0; i < path.size(); ++i) {
            Direction d = path[i];  // Obter direção atual
            
            // Converter direção para string legível
            const char* dir_name = 
                d == Direction::UP ? "UP" :
                d == Direction::DOWN ? "DOWN" :
                d == Direction::LEFT ? "LEFT" : "RIGHT";
            
            // Calcular próxima posição esperada baseada na direção
            Cell expected = current_pos_;
            if (d == Direction::UP) expected.row--;
            else if (d == Direction::DOWN) expected.row++;
            else if (d == Direction::LEFT) expected.col--;
            else if (d == Direction::RIGHT) expected.col++;
            
            // Imprimir informações do movimento
            RCLCPP_INFO(this->get_logger(), 
                       "   [%zu/%zu] %s: (%d,%d) → (%d,%d)", 
                       i+1, path.size(), dir_name,
                       current_pos_.row, current_pos_.col,
                       expected.row, expected.col);
            
            // Tentar executar o movimento
            if (!try_move_navigation(d)) {
                // Movimento falhou - erro crítico!
                RCLCPP_ERROR(this->get_logger(), 
                           "❌ Falha no passo %zu/%zu!", i+1, path.size());
                RCLCPP_ERROR(this->get_logger(),
                           "   Tentou: %s para (%d,%d)", 
                           dir_name, expected.row, expected.col);
                RCLCPP_ERROR(this->get_logger(),
                           "   Isso NÃO deveria acontecer - o caminho BFS está incorreto!");
                return;  // Abortar execução
            }
            
            // Verificar se chegou onde esperava
            if (current_pos_.row != expected.row || current_pos_.col != expected.col) {
                RCLCPP_WARN(this->get_logger(),
                           "Posição diferente! Esperava (%d,%d) mas está em (%d,%d)",
                           expected.row, expected.col,
                           current_pos_.row, current_pos_.col);
            }
            
            rclcpp::sleep_for(100ms);  // Pequena pausa entre movimentos
        }
        
        // Imprimir mensagem de conclusão
        RCLCPP_INFO(this->get_logger(), 
                   "✅ Caminho completo executado! Posição final: (%d,%d)",
                   current_pos_.row, current_pos_.col);
    }
    
    // Função que salva o mapa construído em arquivo de texto
    void save_map_to_file() {
        std::ofstream file("mapa_construido.txt");  // Criar arquivo
        
        // Calcular limites do mapa
        int min_row = 9999, max_row = -9999;
        int min_col = 9999, max_col = -9999;
        
        // Encontrar limites mínimos e máximos
        for (const auto& [cell, _] : map_) {
            min_row = std::min(min_row, cell.row);
            max_row = std::max(max_row, cell.row);
            min_col = std::min(min_col, cell.col);
            max_col = std::max(max_col, cell.col);
        }
        
        // Escrever cabeçalho com dimensões do mapa
        file << "MAPA CONSTRUÍDO: " << (max_row - min_row + 1) << "x" 
             << (max_col - min_col + 1) << "\n\n";
        
        // Percorrer todas as linhas do mapa
        for (int r = min_row; r <= max_row; ++r) {
            // Percorrer todas as colunas
            for (int c = min_col; c <= max_col; ++c) {
                Cell cell = {r, c};
                
                // Marcar posição inicial com 'S' (Start)
                if (cell == initial_pos_) file << "S";
                // Marcar posição do target com 'G' (Goal)
                else if (cell == target_pos_) file << "G";
                // Para outras células
                else {
                    auto it = map_.find(cell);
                    if (it == map_.end()) file << " ";          // Não explorada
                    else if (it->second == "b") file << "#";    // Parede
                    else if (it->second == "f") file << ".";    // Livre
                    else if (it->second == "t") file << "T";    // Target
                    else file << "?";                           // Desconhecido
                }
            }
            file << "\n";  // Nova linha ao fim de cada linha do mapa
        }
        
        file.close();  // Fechar arquivo
        RCLCPP_INFO(this->get_logger(), "Mapa salvo: mapa_construido.txt");
    }
    
    // Função que imprime o mapa no console (terminal)
    void print_map() {
        // Calcular limites do mapa
        int min_row = 9999, max_row = -9999;
        int min_col = 9999, max_col = -9999;
        
        // Encontrar limites mínimos e máximos
        for (const auto& [cell, _] : map_) {
            min_row = std::min(min_row, cell.row);
            max_row = std::max(max_row, cell.row);
            min_col = std::min(min_col, cell.col);
            max_col = std::max(max_col, cell.col);
        }
        
        RCLCPP_INFO(this->get_logger(), "\n=== MAPA CONSTRUÍDO ===");
        
        // Percorrer todas as linhas do mapa
        for (int r = min_row; r <= max_row; ++r) {
            std::string line;  // String para armazenar a linha
            // Percorrer todas as colunas
            for (int c = min_col; c <= max_col; ++c) {
                Cell cell = {r, c};
                
                // Marcar posição atual do robô com 'R'
                if (cell == current_pos_) line += "R";
                // Marcar posição do target com 'T'
                else if (cell == target_pos_) line += "T";
                // Para outras células
                else {
                    auto it = map_.find(cell);
                    if (it == map_.end()) line += " ";          // Não explorada
                    else if (it->second == "b") line += "#";    // Parede
                    else if (it->second == "f") line += ".";    // Livre
                    else if (it->second == "t") line += "T";    // Target
                    else line += "?";                           // Desconhecido
                }
            }
            RCLCPP_INFO(this->get_logger(), "%s", line.c_str());  // Imprimir linha
        }
        
        RCLCPP_INFO(this->get_logger(), "=======================\n");
    }
};

// Função main - ponto de entrada do programa
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  // Inicializar ROS2
    auto node = std::make_shared<MazeMapper>();  // Criar nó MazeMapper
    node->run();  // Executar a lógica principal do nó
    rclcpp::shutdown();  // Finalizar ROS2
    return 0;  // Retornar 0 indicando execução bem-sucedida
}