// Inclusão das bibliotecas necessárias do ROS2 e interfaces customizadas
#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>

// Bibliotecas padrão do C++
#include <chrono>
#include <queue>
#include <string>
#include <vector>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

// ============================================================================
// DEFINIÇÕES DE GRID E DIREÇÕES
// ============================================================================

// Enumeração para representar as 4 direções possíveis de movimento
enum class Direction { UP, DOWN, LEFT, RIGHT };

// Estrutura para representar uma célula no grid (posição x, y)
struct Cell {
    int x;
    int y;
};

// Estrutura para armazenar informações sobre o pai de uma célula durante o BFS
struct ParentInfo {
    int px, py;              // Coordenadas do pai
    Direction dir;           // Direção tomada para chegar nesta célula
    bool has_parent = false; // Indica se esta célula tem um pai válido
};

// Função auxiliar: verifica se uma célula é uma parede
bool is_wall(const std::string &cell)   { return cell == "b"; }

// Função auxiliar: verifica se uma célula é livre para movimento
bool is_free(const std::string &cell)   { return cell == "f"; }


// ============================================================================
// BFS PARA ENCONTRAR O CAMINHO OTIMIZADO 
// ============================================================================

// Reconstrói o caminho do início ao objetivo usando a matriz de pais
std::vector<Direction> reconstruct_path(
    Cell start,
    Cell goal,
    const std::vector<std::vector<ParentInfo>> &parent
) {
    std::vector<Direction> reversed;
    Cell cur = goal;

    // Percorre de trás para frente, do objetivo até o início
    while (!(cur.x == start.x && cur.y == start.y)) {
        auto p = parent[cur.y][cur.x];
        
        // Verifica se a célula atual tem um pai válido
        if (!p.has_parent) {
            std::cerr << "Reconstrucao falhou: sem pai em (" 
                      << cur.x << "," << cur.y << ")\n";
            break;
        }
        
        // Adiciona a direção ao caminho reverso
        reversed.push_back(p.dir);
        
        // Move para o pai
        cur = {p.px, p.py};
    }

    // Inverte o vetor para obter o caminho correto (do início ao fim)
    return std::vector<Direction>(reversed.rbegin(), reversed.rend());
}

// Algoritmo BFS (Busca em Largura) para encontrar o caminho mais curto
// Recebe as dimensões do mapa, as células, posição inicial e objetivo
std::vector<Direction> bfs_find_path(
    int width,
    int height,
    const std::vector<std::string> &cells,
    Cell start,
    Cell goal
) {
    // Valida se as posições inicial e final são válidas
    if (start.x < 0 || start.y < 0 || goal.x < 0 || goal.y < 0) {
        std::cerr << "Start ou goal invalidos!\n";
        return {};
    }

    // Matriz para controlar quais células já foram visitadas
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    
    // Matriz para armazenar informações sobre o pai de cada célula
    std::vector<std::vector<ParentInfo>> parent(height, std::vector<ParentInfo>(width));

    // Fila para o algoritmo BFS
    std::queue<Cell> q;
    q.push(start);
    visited[start.y][start.x] = true;

    // Arrays para representar os movimentos nas 4 direções (cima, baixo, esquerda, direita)
    const int dx[4] = {0, 0, -1, 1};   // Deslocamento em X
    const int dy[4] = {-1, 1, 0, 0};   // Deslocamento em Y
    const Direction dirs[4] = {
        Direction::UP,
        Direction::DOWN,
        Direction::LEFT,
        Direction::RIGHT
    };

    bool found = false;

    // Loop principal do BFS
    while (!q.empty() && !found) {
        Cell cur = q.front();
        q.pop();

        // Explora todas as 4 direções possíveis
        for (int k = 0; k < 4; ++k) {
            int nx = cur.x + dx[k];  // Nova posição X
            int ny = cur.y + dy[k];  // Nova posição Y

            // Verifica se a nova posição está dentro dos limites do grid
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            
            // Verifica se já visitamos esta célula
            if (visited[ny][nx]) continue;

            // Obtém o tipo da célula
            auto &cell = cells[ny * width + nx];
            
            // Ignora paredes
            if (is_wall(cell)) continue;

            // Marca como visitada
            visited[ny][nx] = true;
            
            // Armazena informações sobre o pai (célula atual e direção)
            parent[ny][nx] = {cur.x, cur.y, dirs[k], true};

            // Verifica se chegamos ao objetivo
            if (nx == goal.x && ny == goal.y) {
                found = true;
                break;
            }

            // Adiciona a nova célula à fila para exploração
            q.push({nx, ny});
        }
    }

    // Se não encontrou caminho, retorna vetor vazio
    if (!found) {
        std::cerr << "Nenhum caminho encontrado de ("
                  << start.x << "," << start.y << ") ate ("
                  << goal.x  << "," << goal.y  << ").\n";
        return {};
    }

    // Reconstrói e retorna o caminho encontrado
    return reconstruct_path(start, goal, parent);
}

// ============================================================================
// NÓ ROS2 (chama /get_map, /move_command e executa a rota)
// ============================================================================

class MazeNavigator : public rclcpp::Node {
public:
    // Construtor: inicializa o nó e cria os clientes de serviço
    MazeNavigator()
    : rclcpp::Node("maze_navigator")
    {
        // Cliente para obter o mapa do labirinto
        get_map_client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        
        // Cliente para enviar comandos de movimento
        move_client_    = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    }

    // Método principal que executa toda a lógica de navegação
    void run() {
        RCLCPP_INFO(this->get_logger(), "Solicitando mapa...");

        // Aguarda o serviço /get_map ficar disponível (timeout de 5 segundos)
        if (!get_map_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /get_map nao disponivel.");
            return;
        }

        // Cria requisição para obter o mapa
        auto map_req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto map_future = get_map_client_->async_send_request(map_req);

        // Aguarda a resposta do serviço
        if (rclcpp::spin_until_future_complete(shared_from_this(), map_future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Falha ao chamar /get_map");
            return;
        }

        // Obtém a resposta contendo o mapa
        auto map_resp = map_future.get();

        // Valida a forma do grid (deve ter pelo menos 2 dimensões)
        if (map_resp->occupancy_grid_shape.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Forma do grid invalida.");
            return;
        }

        // Extrai dimensões e células do mapa
        int height = map_resp->occupancy_grid_shape[0];  // Altura do grid
        int width  = map_resp->occupancy_grid_shape[1];  // Largura do grid
        auto cells = map_resp->occupancy_grid_flattened; // Células do grid (vetor 1D)

        RCLCPP_INFO(this->get_logger(),
                    "Mapa recebido: %dx%d (cells=%zu)", width, height, cells.size());

        // Verifica se o tamanho do vetor de células corresponde às dimensões
        if ((int)cells.size() != width * height) {
            RCLCPP_WARN(this->get_logger(),
                "Tamanho do vetor de celulas (%zu) difere de width*height (%d).",
                cells.size(), width * height);
        }

        // Aguarda o serviço /move_command ficar disponível
        if (!move_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /move_command nao disponivel.");
            return;
        }

        // Cria requisição dummy para obter as posições do robô e do alvo
        auto move_req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        move_req->direction = "";  // Comando vazio apenas para consultar posições

        RCLCPP_INFO(this->get_logger(),
                    "Chamando /move_command uma vez para obter robot_pos e target_pos...");

        // Envia requisição e aguarda resposta
        auto move_future = move_client_->async_send_request(move_req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), move_future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Falha ao chamar /move_command");
            return;
        }

        // Obtém a resposta com as posições
        auto move_resp = move_future.get();

        // Valida se as posições do robô e do alvo foram recebidas
        if (move_resp->robot_pos.size() < 2 || move_resp->target_pos.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Resposta de /move_command invalida (posicoes).");
            return;
        }

        // Extrai as posições inicial e final
        Cell start{ move_resp->robot_pos[0], move_resp->robot_pos[1] };   // Posição do robô
        Cell goal { move_resp->target_pos[0], move_resp->target_pos[1] }; // Posição do alvo

        RCLCPP_INFO(this->get_logger(),
                    "Posicao do robo: (%d, %d), alvo: (%d, %d)",
                    start.x, start.y, goal.x, goal.y);

        // Calcula o caminho mais curto usando BFS
        auto path = bfs_find_path(width, height, cells, start, goal);
        
        // Verifica se foi encontrado algum caminho
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Nenhuma rota encontrada.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Executando caminho com %zu passos...", path.size());
        
        // Executa o caminho encontrado
        execute_path(path);
    }

private:
    // Cliente para o serviço de obtenção do mapa
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    
    // Cliente para o serviço de comando de movimento
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;

    // Executa uma sequência de movimentos
    void execute_path(const std::vector<Direction> &path) {
        // Verifica se o serviço está disponível
        if (!move_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /move_command nao disponivel.");
            return;
        }

        // Itera sobre cada direção no caminho
        for (Direction d : path) {
            // Cria requisição de movimento
            auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();

            // Converte a direção enum para string
            switch (d) {
                case Direction::UP:    req->direction = "up";    break;
                case Direction::DOWN:  req->direction = "down";  break;
                case Direction::LEFT:  req->direction = "left";  break;
                case Direction::RIGHT: req->direction = "right"; break;
            }

            RCLCPP_INFO(this->get_logger(), "Movendo: %s", req->direction.c_str());

            // Envia o comando de movimento
            auto future = move_client_->async_send_request(req);
            rclcpp::spin_until_future_complete(shared_from_this(), future);

            // Aguarda 150ms entre movimentos (para evitar comandos muito rápidos)
            rclcpp::sleep_for(150ms);
        }

        RCLCPP_INFO(this->get_logger(), "Caminho concluido!");
    }
};

// ============================================================================
// FUNÇÃO MAIN
// ============================================================================

int main(int argc, char * argv[])
{
    // Inicializa o ROS2
    rclcpp::init(argc, argv);
    
    // Cria uma instância do nó MazeNavigator
    auto node = std::make_shared<MazeNavigator>();
    
    // Executa toda a lógica de navegação
    node->run();
    
    // Finaliza o ROS2
    rclcpp::shutdown();
    
    return 0;
}