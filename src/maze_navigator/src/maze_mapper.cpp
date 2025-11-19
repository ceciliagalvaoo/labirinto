#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/srv/reset.hpp>

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

using namespace std::chrono_literals;

// DEFINIÇÕES DE GRID E DIREÇÕES
enum class Direction { UP, DOWN, LEFT, RIGHT };

struct Cell {
    int row;  // Linha (muda com UP/DOWN)
    int col;  // Coluna (muda com LEFT/RIGHT)
    
    bool operator<(const Cell& other) const {
        if (row != other.row) return row < other.row;
        return col < other.col;
    }
    
    bool operator==(const Cell& other) const {
        return row == other.row && col == other.col;
    }
    
    bool operator!=(const Cell& other) const {
        return !(*this == other);
    }
};

struct ParentInfo {
    int parent_row, parent_col;
    Direction dir;
    bool has_parent = false;
};

bool is_wall(const std::string &cell)   { return cell == "b"; }
bool is_free(const std::string &cell)   { return cell == "f"; }
bool is_target(const std::string &cell) { return cell == "t"; }

// BFS PARA ENCONTRAR O CAMINHO OTIMIZADO

std::vector<Direction> reconstruct_path(
    Cell start,
    Cell goal,
    const std::vector<std::vector<ParentInfo>> &parent
) {
    std::vector<Direction> reversed;
    Cell cur = goal;

    while (cur != start) {
        auto p = parent[cur.row][cur.col];
        if (!p.has_parent) {
            std::cerr << "Reconstrucao falhou: sem pai em (" 
                      << cur.row << "," << cur.col << ")\n";
            break;
        }
        reversed.push_back(p.dir);
        cur = {p.parent_row, p.parent_col};
    }

    return std::vector<Direction>(reversed.rbegin(), reversed.rend());
}

std::vector<Direction> bfs_find_path(
    int height,
    int width,
    const std::vector<std::string> &cells,
    Cell start,
    Cell goal
) {
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    std::vector<std::vector<ParentInfo>> parent(height, std::vector<ParentInfo>(width));

    std::queue<Cell> q;
    q.push(start);
    visited[start.row][start.col] = true;

    // Direções: UP, DOWN, LEFT, RIGHT
    const int drow[4] = {-1, 1, 0, 0};
    const int dcol[4] = {0, 0, -1, 1};
    const Direction dirs[4] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

    bool found = false;

    while (!q.empty() && !found) {
        Cell cur = q.front();
        q.pop();

        for (int k = 0; k < 4; ++k) {
            int nrow = cur.row + drow[k];
            int ncol = cur.col + dcol[k];

            if (nrow < 0 || nrow >= height || ncol < 0 || ncol >= width) continue;
            if (visited[nrow][ncol]) continue;

            auto &cell = cells[nrow * width + ncol];
            if (is_wall(cell)) continue;

            visited[nrow][ncol] = true;
            parent[nrow][ncol] = {cur.row, cur.col, dirs[k], true};

            if (nrow == goal.row && ncol == goal.col) {
                found = true;
                break;
            }

            q.push({nrow, ncol});
        }
    }

    if (!found) {
        std::cerr << "Nenhum caminho encontrado de ("
                  << start.row << "," << start.col << ") ate ("
                  << goal.row  << "," << goal.col  << ").\n";
        return {};
    }

    return reconstruct_path(start, goal, parent);
}

// NÓ ROS2 - MAPEAMENTO E NAVEGAÇÃO

class MazeMapper : public rclcpp::Node {
public:
    MazeMapper()
    : rclcpp::Node("maze_mapper"),
      current_pos_{0, 0},
      initial_pos_{0, 0},
      target_pos_{-1, -1},
      found_target_(false)
    {
        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        reset_client_ = this->create_client<cg_interfaces::srv::Reset>("/reset");
        
        sensor_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors",
            10,
            std::bind(&MazeMapper::sensor_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "MazeMapper inicializado!");
    }

    void run() {
        if (!initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Falha na inicializacao");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "\n╔════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║  FASE 1: EXPLORAÇÃO COMPLETA (DFS)            ║");
        RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════╝");
        explore_maze_complete();
        
        if (!found_target_) {
            RCLCPP_ERROR(this->get_logger(), 
                        "❌ Target não foi encontrado durante exploração!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "\n╔════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║  FASE 2: RESET E NAVEGAÇÃO ÓTIMA             ║");
        RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════╝");
        
        reset_to_initial();
        navigate_to_target();
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    
    // Mapa: (row, col) -> tipo ("f", "b", "t")
    std::map<Cell, std::string> map_;
    
    Cell current_pos_;
    Cell initial_pos_;
    Cell target_pos_;
    bool found_target_;
    std::string loaded_map_name_;  // Nome do mapa carregado
    
    // Para DFS
    std::set<Cell> visited_;
    std::stack<Cell> dfs_stack_;
    
    cg_interfaces::msg::RobotSensors::SharedPtr last_sensor_data_;
    
    void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        last_sensor_data_ = msg;
    }
    
    bool initialize() {
        if (!move_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /move_command nao disponivel");
            return false;
        }
        
        if (!reset_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /reset nao disponivel");
            return false;
        }
        
        // Aguardar sensores
        RCLCPP_INFO(this->get_logger(), "Aguardando dados dos sensores...");
        for (int i = 0; i < 50 && !last_sensor_data_; ++i) {
            rclcpp::sleep_for(100ms);
            rclcpp::spin_some(shared_from_this());
        }
        
        if (!last_sensor_data_) {
            RCLCPP_ERROR(this->get_logger(), "Timeout aguardando sensores");
            return false;
        }
        
        // Primeiro, fazer um reset para garantir estado inicial limpo
        RCLCPP_INFO(this->get_logger(), "Fazendo reset inicial...");
        auto reset_req = std::make_shared<cg_interfaces::srv::Reset::Request>();
        reset_req->is_random = false;
        reset_req->map_name = "";  // Usar mapa atual
        
        auto reset_future = reset_client_->async_send_request(reset_req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), reset_future, 5s)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto reset_resp = reset_future.get();
            if (reset_resp->success) {
                loaded_map_name_ = reset_resp->loaded_map_name;
                RCLCPP_INFO(this->get_logger(), "Reset inicial OK, mapa: %s", loaded_map_name_.c_str());
                rclcpp::sleep_for(500ms);
                // Limpar e atualizar sensores
                last_sensor_data_.reset();
                for (int i = 0; i < 5; ++i) {
                    rclcpp::spin_some(shared_from_this());
                    rclcpp::sleep_for(100ms);
                }
            }
        }
        
        // Obter posição inicial
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = "";
        
        auto future = move_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            return false;
        }
        
        auto resp = future.get();
        if (resp->robot_pos.size() < 2 || resp->target_pos.size() < 2) {
            return false;
        }
        
        // Simulador retorna [row, col]
        current_pos_ = {(int)resp->robot_pos[0], (int)resp->robot_pos[1]};
        initial_pos_ = current_pos_;
        target_pos_ = {(int)resp->target_pos[0], (int)resp->target_pos[1]};
        
        RCLCPP_INFO(this->get_logger(), 
                    "Inicio: (%d,%d), Alvo: (%d,%d)",
                    current_pos_.row, current_pos_.col,
                    target_pos_.row, target_pos_.col);
        
        return true;
    }
    
    void update_sensors() {
        rclcpp::sleep_for(150ms);
        rclcpp::spin_some(shared_from_this());
        rclcpp::spin_some(shared_from_this());
    }
    
    void map_surroundings() {
        if (!last_sensor_data_) return;
        
        // Mapear célula atual como livre (confirmado por estar nela)
        map_[current_pos_] = "f";
        
        struct Neighbor {
            int drow, dcol;
            std::string sensor_value;
            bool is_cardinal;  // Se é direção cardinal (não diagonal)
        };
        
        // IMPORTANTE: Marcar quais sensores são cardinais
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
        
        for (const auto& n : neighbors) {
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
                map_[neighbor] = "t";
                
                // ESTRATÉGIA: Aceitar target em até 2 células de distância
                // (tolerância para pequenos erros de sensores)
                if (!found_target_) {
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
    
    // Função para EXPLORAÇÃO (DFS) - pode corrigir o mapa
    bool try_move_exploration(Direction dir) {
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        
        switch (dir) {
            case Direction::UP:    req->direction = "up"; break;
            case Direction::DOWN:  req->direction = "down"; break;
            case Direction::LEFT:  req->direction = "left"; break;
            case Direction::RIGHT: req->direction = "right"; break;
        }
        
        // Calcular célula de destino
        Cell target_cell = current_pos_;
        if (dir == Direction::UP) target_cell.row--;
        else if (dir == Direction::DOWN) target_cell.row++;
        else if (dir == Direction::LEFT) target_cell.col--;
        else if (dir == Direction::RIGHT) target_cell.col++;
        
        auto future = move_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            return false;
        }
        
        auto resp = future.get();
        
        if (resp->success && resp->robot_pos.size() >= 2) {
            Cell old_pos = current_pos_;
            current_pos_ = {(int)resp->robot_pos[0], (int)resp->robot_pos[1]};
            
            // CONFIRMAR que célula destino é livre (EXCETO se for o target!)
            if (!(target_cell.row == target_pos_.row && target_cell.col == target_pos_.col)) {
                map_[target_cell] = "f";
            }
            
            // CONFIRMAR que célula atual também é livre (EXCETO se for o target!)
            if (!(current_pos_.row == target_pos_.row && current_pos_.col == target_pos_.col)) {
                map_[current_pos_] = "f";
            } else {
                // Chegamos no target! Marcar como encontrado
                if (!found_target_) {
                    found_target_ = true;
                    map_[current_pos_] = "t";
                    RCLCPP_INFO(this->get_logger(),
                               "🎯 TARGET ALCANÇADO por movimento! Posição: (%d,%d)",
                               current_pos_.row, current_pos_.col);
                }
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "✅ Moveu: (%d,%d) → (%d,%d) [%s]",
                       old_pos.row, old_pos.col,
                       current_pos_.row, current_pos_.col,
                       req->direction.c_str());
            
            update_sensors();
            return true;
        }
        
        // Movimento falhou - CONFIRMAR que é parede
        RCLCPP_DEBUG(this->get_logger(), 
                    "❌ Bloqueado tentando ir para (%d,%d)", 
                    target_cell.row, target_cell.col);
        
        // Corrigir mapa se estava errado
        auto it = map_.find(target_cell);
        if (it != map_.end() && it->second != "b") {
            RCLCPP_WARN(this->get_logger(),
                       "CORREÇÃO: Célula (%d,%d) estava como '%s', agora é 'b'",
                       target_cell.row, target_cell.col, it->second.c_str());
        }
        
        map_[target_cell] = "b";
        
        return false;
    }
    
    // Função para NAVEGAÇÃO (BFS) - NÃO modifica o mapa, apenas move
    bool try_move_navigation(Direction dir) {
        // IMPORTANTE: Atualizar sensores ANTES de mover
        update_sensors();
        
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        
        switch (dir) {
            case Direction::UP:    req->direction = "up"; break;
            case Direction::DOWN:  req->direction = "down"; break;
            case Direction::LEFT:  req->direction = "left"; break;
            case Direction::RIGHT: req->direction = "right"; break;
        }
        
        RCLCPP_DEBUG(this->get_logger(),
                    "Enviando comando: '%s' | Posição atual antes do comando: (%d,%d)",
                    req->direction.c_str(), current_pos_.row, current_pos_.col);
        
        auto future = move_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Timeout ao enviar comando de movimento!");
            return false;
        }
        
        auto resp = future.get();
        
        RCLCPP_DEBUG(this->get_logger(),
                    "Resposta recebida: success=%d, robot_pos.size()=%zu",
                    resp->success, resp->robot_pos.size());
        
        if (resp->success && resp->robot_pos.size() >= 2) {
            Cell old_pos = current_pos_;
            Cell new_pos = {(int)resp->robot_pos[0], (int)resp->robot_pos[1]};
            
            RCLCPP_DEBUG(this->get_logger(),
                        "📍 Resposta do serviço: robot_pos=[%d,%d]",
                        new_pos.row, new_pos.col);
            
            current_pos_ = new_pos;
            
            RCLCPP_INFO(this->get_logger(), 
                       "✅ Moveu: (%d,%d) → (%d,%d) [%s]",
                       old_pos.row, old_pos.col,
                       current_pos_.row, current_pos_.col,
                       req->direction.c_str());
            
            return true;
        }
        
        // Movimento falhou durante navegação - NÃO deve acontecer!
        RCLCPP_ERROR(this->get_logger(), 
                    "❌ Falha crítica: movimento bloqueado durante navegação BFS!");
        RCLCPP_ERROR(this->get_logger(),
                    "   Resposta: success=%d, robot_pos.size()=%zu",
                    resp->success, resp->robot_pos.size());
        
        return false;
    }
    
    void explore_maze_complete() {
        RCLCPP_INFO(this->get_logger(), "Iniciando exploração DFS até encontrar target...");
        
        // Inicializar DFS
        visited_.insert(current_pos_);
        dfs_stack_.push(current_pos_);
        map_surroundings();
        
        // VERIFICAR SE JÁ DETECTOU TARGET NA POSIÇÃO INICIAL
        if (found_target_) {
            RCLCPP_INFO(this->get_logger(), 
                       "TARGET DETECTADO NA POSIÇÃO INICIAL! Parando exploração.");
            save_map_to_file();
            print_map();
            return;
        }
        
        int steps = 0;
        const int MAX_STEPS = 10000;
        
        while (!dfs_stack_.empty() && steps < MAX_STEPS) {
            // VERIFICAR SE ENCONTROU TARGET ANTES DE CONTINUAR
            if (found_target_) {
                RCLCPP_INFO(this->get_logger(), 
                           "\nTARGET DETECTADO! Interrompendo exploração...");
                RCLCPP_INFO(this->get_logger(), 
                           "   Target está em: (%d,%d)", target_pos_.row, target_pos_.col);
                RCLCPP_INFO(this->get_logger(), 
                           "   Robô está em: (%d,%d)", current_pos_.row, current_pos_.col);
                break;
            }
            
            map_surroundings();
            
            // VERIFICAR NOVAMENTE APÓS MAPEAR (pode ter detectado agora)
            if (found_target_) {
                RCLCPP_INFO(this->get_logger(), 
                           "\nTARGET DETECTADO PELOS SENSORES! Parando exploração.");
                break;
            }
            
            // Tentar encontrar vizinho não visitado (4 direções principais)
            std::vector<std::pair<Direction, Cell>> neighbors = {
                {Direction::UP, {current_pos_.row - 1, current_pos_.col}},
                {Direction::DOWN, {current_pos_.row + 1, current_pos_.col}},
                {Direction::LEFT, {current_pos_.row, current_pos_.col - 1}},
                {Direction::RIGHT, {current_pos_.row, current_pos_.col + 1}}
            };
            
            bool moved = false;
            
            for (const auto& [dir, neighbor] : neighbors) {
                // NÃO MOVER PARA O TARGET!
                if (neighbor == target_pos_) {
                    RCLCPP_INFO(this->get_logger(), 
                               "DETECTADO: Target está adjacente em (%d,%d)! Marcando como encontrado.", 
                               neighbor.row, neighbor.col);
                    visited_.insert(neighbor); // Marcar como visitado
                    map_[neighbor] = "t";  // Garantir que está mapeado como target
                    found_target_ = true;  // Marcar como encontrado!
                    continue;
                }
                
                // Verificar se já visitamos
                if (visited_.find(neighbor) != visited_.end()) continue;
                
                // Verificar se já sabemos que é parede
                auto it = map_.find(neighbor);
                if (it != map_.end() && it->second == "b") {
                    visited_.insert(neighbor); // Marcar como visitado para não tentar de novo
                    continue;
                }
                
                // Tentar mover
                if (try_move_exploration(dir)) {
                    visited_.insert(current_pos_);
                    dfs_stack_.push(current_pos_);
                    moved = true;
                    steps++;
                    
                    if (steps % 50 == 0) {
                        RCLCPP_INFO(this->get_logger(), 
                                   "Exploração: %d passos, %zu visitadas, %zu mapeadas",
                                   steps, visited_.size(), map_.size());
                    }
                    break;
                }
            }
            
            if (!moved) {
                // Backtrack
                dfs_stack_.pop();
                
                if (!dfs_stack_.empty()) {
                    Cell target = dfs_stack_.top();
                    
                    RCLCPP_DEBUG(this->get_logger(), 
                                "⬅Backtrack para (%d,%d)", target.row, target.col);
                    
                    // Mover para o pai
                    if (target.row < current_pos_.row) try_move_exploration(Direction::UP);
                    else if (target.row > current_pos_.row) try_move_exploration(Direction::DOWN);
                    else if (target.col < current_pos_.col) try_move_exploration(Direction::LEFT);
                    else if (target.col > current_pos_.col) try_move_exploration(Direction::RIGHT);
                    
                    // IMPORTANTE: Mapear após backtrack também!
                    map_surroundings();
                    
                    steps++;
                }
            }
        }
        
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
                RCLCPP_WARN(this->get_logger(), 
                           "\nAVISO: Target não foi detectado durante exploração!");
                RCLCPP_WARN(this->get_logger(), 
                           "   Pode estar em área não explorada ou sensores falharam.");
            }
        }
        
        save_map_to_file();
        print_map();
    }
    
    bool reset_to_initial() {
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
        req->is_random = false;
        req->map_name = loaded_map_name_;  // Usar nome do mapa salvo
        
        RCLCPP_INFO(this->get_logger(),
                   "   Resetando mapa: '%s'", loaded_map_name_.c_str());
        
        auto future = reset_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future, 5s)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Falha ao chamar serviço /reset");
            return false;
        }
        
        auto resp = future.get();
        
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
        rclcpp::sleep_for(300ms);
        
        for (int i = 0; i < 5; ++i) {
            rclcpp::spin_some(shared_from_this());
            rclcpp::sleep_for(100ms);
        }
        
        if (!last_sensor_data_) {
            RCLCPP_WARN(this->get_logger(), "Sensores ainda não recebidos após reset!");
        }
        
        // OBTER POSIÇÃO ATUAL DO ROBÔ APÓS RESET
        // Fazer uma chamada ao /move_command com direção vazia para confirmar posição
        RCLCPP_INFO(this->get_logger(), "Verificando posição real do robô...");
        
        auto move_req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        move_req->direction = "";
        
        auto move_future = move_client_->async_send_request(move_req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), move_future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Falha ao verificar posição após reset");
            return false;
        }
        
        auto move_resp = move_future.get();
        
        if (move_resp->robot_pos.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "❌ Resposta inválida ao verificar posição");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(),
                   "Resposta do /move_command (vazio):");
        RCLCPP_INFO(this->get_logger(),
                   "   robot_pos = [%d, %d]",
                   (int)move_resp->robot_pos[0], (int)move_resp->robot_pos[1]);
        
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
        int reset_attempts = 0;
        const int MAX_RESET_ATTEMPTS = 3;
        
        while ((current_pos_.row != initial_pos_.row || current_pos_.col != initial_pos_.col) 
               && reset_attempts < MAX_RESET_ATTEMPTS) 
        {
            RCLCPP_WARN(this->get_logger(),
                       "AVISO: Posição após reset (%d,%d) é diferente da inicial esperada (%d,%d)",
                       current_pos_.row, current_pos_.col,
                       initial_pos_.row, initial_pos_.col);
            
            reset_attempts++;
            RCLCPP_WARN(this->get_logger(),
                       "Tentativa %d/%d: Tentando reset novamente...",
                       reset_attempts, MAX_RESET_ATTEMPTS);
            
            // Tentar reset novamente
            auto retry_req = std::make_shared<cg_interfaces::srv::Reset::Request>();
            retry_req->is_random = false;
            retry_req->map_name = loaded_map_name_;
            
            auto retry_future = reset_client_->async_send_request(retry_req);
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
                    check_req->direction = "";
                    
                    auto check_future = move_client_->async_send_request(check_req);
                    if (rclcpp::spin_until_future_complete(shared_from_this(), check_future)
                        == rclcpp::FutureReturnCode::SUCCESS)
                    {
                        auto check_resp = check_future.get();
                        if (check_resp->robot_pos.size() >= 2) {
                            current_pos_ = {(int)check_resp->robot_pos[0], (int)check_resp->robot_pos[1]};
                            RCLCPP_INFO(this->get_logger(),
                                       "   Nova posição: (%d,%d)", current_pos_.row, current_pos_.col);
                        }
                    }
                }
            }
        }
        
        // Verificar se conseguiu resetar corretamente
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
        
        return true;
    }
    
    void navigate_to_target() {
        // IMPORTANTE: Atualizar sensores ANTES de começar a navegação
        RCLCPP_INFO(this->get_logger(), "Atualizando sensores antes da navegação...");
        update_sensors();
        rclcpp::sleep_for(200ms);
        update_sensors();
        
        RCLCPP_INFO(this->get_logger(), 
                   "\n╔════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), 
                   "║  NAVEGANDO PARA O TARGET (BFS)                ║");
        RCLCPP_INFO(this->get_logger(), 
                   "╚════════════════════════════════════════════════╝");
        
        RCLCPP_INFO(this->get_logger(),
                   "Posição ANTES da navegação BFS: (%d,%d)",
                   current_pos_.row, current_pos_.col);
        
        // Calcular grid para BFS
        int min_row = 9999, max_row = -9999;
        int min_col = 9999, max_col = -9999;
        
        for (const auto& [cell, _] : map_) {
            min_row = std::min(min_row, cell.row);
            max_row = std::max(max_row, cell.row);
            min_col = std::min(min_col, cell.col);
            max_col = std::max(max_col, cell.col);
        }
        
        int height = max_row - min_row + 1;
        int width = max_col - min_col + 1;
        
        // Células desconhecidas permanecem como paredes (padrão)
        std::vector<std::string> grid(height * width, "b");
        
        for (const auto& [cell, value] : map_) {
            int r = cell.row - min_row;
            int c = cell.col - min_col;
            grid[r * width + c] = value;
        }
        
        Cell bfs_start = {current_pos_.row - min_row, current_pos_.col - min_col};
        Cell bfs_goal = {target_pos_.row - min_row, target_pos_.col - min_col};
        
        RCLCPP_INFO(this->get_logger(), 
                    "Grid: %dx%d", height, width);
        RCLCPP_INFO(this->get_logger(), 
                    "Start: (%d,%d) | Goal: (%d,%d)",
                    current_pos_.row, current_pos_.col,
                    target_pos_.row, target_pos_.col);
        
        auto path = bfs_find_path(height, width, grid, bfs_start, bfs_goal);
        
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ BFS não encontrou caminho!");
            save_debug_grid(height, width, grid, bfs_start, bfs_goal);
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), 
                    "✓ Caminho ótimo: %zu passos", path.size());
        
        execute_path(path);
        
        // Verificar se chegou no target
        if (current_pos_ == target_pos_) {
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
            RCLCPP_WARN(this->get_logger(),
                       "Parou em (%d,%d) mas target está em (%d,%d)",
                       current_pos_.row, current_pos_.col,
                       target_pos_.row, target_pos_.col);
        }
    }
    
    void save_debug_grid(int height, int width, 
                         const std::vector<std::string>& grid,
                         Cell start, Cell goal) {
        std::ofstream file("debug_grid.txt");
        file << "DEBUG GRID BFS - " << height << "x" << width << "\n";
        file << "Start: (" << start.row << "," << start.col << ")\n";
        file << "Goal: (" << goal.row << "," << goal.col << ")\n\n";
        
        for (int r = 0; r < height; ++r) {
            for (int c = 0; c < width; ++c) {
                if (r == start.row && c == start.col) {
                    file << "S";
                } else if (r == goal.row && c == goal.col) {
                    file << "G";
                } else {
                    std::string cell = grid[r * width + c];
                    if (cell == "b") file << "#";
                    else if (cell == "f") file << ".";
                    else if (cell == "t") file << "T";
                    else file << "?";
                }
            }
            file << "\n";
        }
        file.close();
        RCLCPP_ERROR(this->get_logger(), "Grid debug salvo: debug_grid.txt");
    }
    
    void execute_path(const std::vector<Direction>& path) {
        RCLCPP_INFO(this->get_logger(), 
                   "\nIniciando execução do caminho (%zu passos)...", path.size());
        
        for (size_t i = 0; i < path.size(); ++i) {
            Direction d = path[i];
            
            const char* dir_name = 
                d == Direction::UP ? "UP" :
                d == Direction::DOWN ? "DOWN" :
                d == Direction::LEFT ? "LEFT" : "RIGHT";
            
            // Calcular próxima posição esperada
            Cell expected = current_pos_;
            if (d == Direction::UP) expected.row--;
            else if (d == Direction::DOWN) expected.row++;
            else if (d == Direction::LEFT) expected.col--;
            else if (d == Direction::RIGHT) expected.col++;
            
            RCLCPP_INFO(this->get_logger(), 
                       "   [%zu/%zu] %s: (%d,%d) → (%d,%d)", 
                       i+1, path.size(), dir_name,
                       current_pos_.row, current_pos_.col,
                       expected.row, expected.col);
            
            if (!try_move_navigation(d)) {
                RCLCPP_ERROR(this->get_logger(), 
                           "❌ Falha no passo %zu/%zu!", i+1, path.size());
                RCLCPP_ERROR(this->get_logger(),
                           "   Tentou: %s para (%d,%d)", 
                           dir_name, expected.row, expected.col);
                RCLCPP_ERROR(this->get_logger(),
                           "   Isso NÃO deveria acontecer - o caminho BFS está incorreto!");
                return;
            }
            
            // Verificar se chegou onde esperava
            if (current_pos_.row != expected.row || current_pos_.col != expected.col) {
                RCLCPP_WARN(this->get_logger(),
                           "Posição diferente! Esperava (%d,%d) mas está em (%d,%d)",
                           expected.row, expected.col,
                           current_pos_.row, current_pos_.col);
            }
            
            rclcpp::sleep_for(100ms);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "✅ Caminho completo executado! Posição final: (%d,%d)",
                   current_pos_.row, current_pos_.col);
    }
    
    void save_map_to_file() {
        std::ofstream file("mapa_construido.txt");
        
        int min_row = 9999, max_row = -9999;
        int min_col = 9999, max_col = -9999;
        
        for (const auto& [cell, _] : map_) {
            min_row = std::min(min_row, cell.row);
            max_row = std::max(max_row, cell.row);
            min_col = std::min(min_col, cell.col);
            max_col = std::max(max_col, cell.col);
        }
        
        file << "MAPA CONSTRUÍDO: " << (max_row - min_row + 1) << "x" 
             << (max_col - min_col + 1) << "\n\n";
        
        for (int r = min_row; r <= max_row; ++r) {
            for (int c = min_col; c <= max_col; ++c) {
                Cell cell = {r, c};
                
                if (cell == initial_pos_) file << "S";
                else if (cell == target_pos_) file << "G";
                else {
                    auto it = map_.find(cell);
                    if (it == map_.end()) file << " ";
                    else if (it->second == "b") file << "#";
                    else if (it->second == "f") file << ".";
                    else if (it->second == "t") file << "T";
                    else file << "?";
                }
            }
            file << "\n";
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Mapa salvo: mapa_construido.txt");
    }
    
    void print_map() {
        int min_row = 9999, max_row = -9999;
        int min_col = 9999, max_col = -9999;
        
        for (const auto& [cell, _] : map_) {
            min_row = std::min(min_row, cell.row);
            max_row = std::max(max_row, cell.row);
            min_col = std::min(min_col, cell.col);
            max_col = std::max(max_col, cell.col);
        }
        
        RCLCPP_INFO(this->get_logger(), "\n=== MAPA CONSTRUÍDO ===");
        
        for (int r = min_row; r <= max_row; ++r) {
            std::string line;
            for (int c = min_col; c <= max_col; ++c) {
                Cell cell = {r, c};
                
                if (cell == current_pos_) line += "R";
                else if (cell == target_pos_) line += "T";
                else {
                    auto it = map_.find(cell);
                    if (it == map_.end()) line += " ";
                    else if (it->second == "b") line += "#";
                    else if (it->second == "f") line += ".";
                    else if (it->second == "t") line += "T";
                    else line += "?";
                }
            }
            RCLCPP_INFO(this->get_logger(), "%s", line.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "=======================\n");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeMapper>();
    node->run();
    rclcpp::shutdown();
    return 0;
}