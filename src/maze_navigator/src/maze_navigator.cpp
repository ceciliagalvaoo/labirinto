#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>

#include <chrono>
#include <queue>
#include <string>
#include <vector>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

// DEFINIÇÕES DE GRID E DIREÇÕES

enum class Direction { UP, DOWN, LEFT, RIGHT };

struct Cell {
    int x;
    int y;
};

struct ParentInfo {
    int px, py;
    Direction dir;
    bool has_parent = false;
};

bool is_wall(const std::string &cell)   { return cell == "b"; }
bool is_free(const std::string &cell)   { return cell == "f"; }


// BFS PARA ENCONTRAR O CAMINHO OTIMIZADO 

std::vector<Direction> reconstruct_path(
    Cell start,
    Cell goal,
    const std::vector<std::vector<ParentInfo>> &parent
) {
    std::vector<Direction> reversed;
    Cell cur = goal;

    while (!(cur.x == start.x && cur.y == start.y)) {
        auto p = parent[cur.y][cur.x];
        if (!p.has_parent) {
            std::cerr << "Reconstrucao falhou: sem pai em (" 
                      << cur.x << "," << cur.y << ")\n";
            break;
        }
        reversed.push_back(p.dir);
        cur = {p.px, p.py};
    }

    return std::vector<Direction>(reversed.rbegin(), reversed.rend());
}

// Agora recebemos start e goal explicitamente
std::vector<Direction> bfs_find_path(
    int width,
    int height,
    const std::vector<std::string> &cells,
    Cell start,
    Cell goal
) {
    if (start.x < 0 || start.y < 0 || goal.x < 0 || goal.y < 0) {
        std::cerr << "Start ou goal invalidos!\n";
        return {};
    }

    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    std::vector<std::vector<ParentInfo>> parent(height, std::vector<ParentInfo>(width));

    std::queue<Cell> q;
    q.push(start);
    visited[start.y][start.x] = true;

    const int dx[4] = {0, 0, -1, 1};
    const int dy[4] = {-1, 1, 0, 0};
    const Direction dirs[4] = {
        Direction::UP,
        Direction::DOWN,
        Direction::LEFT,
        Direction::RIGHT
    };

    bool found = false;

    while (!q.empty() && !found) {
        Cell cur = q.front();
        q.pop();

        for (int k = 0; k < 4; ++k) {
            int nx = cur.x + dx[k];
            int ny = cur.y + dy[k];

            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            if (visited[ny][nx]) continue;

            auto &cell = cells[ny * width + nx];
            if (is_wall(cell)) continue;  // ignora paredes

            visited[ny][nx] = true;
            parent[ny][nx] = {cur.x, cur.y, dirs[k], true};

            if (nx == goal.x && ny == goal.y) {
                found = true;
                break;
            }

            q.push({nx, ny});
        }
    }

    if (!found) {
        std::cerr << "Nenhum caminho encontrado de ("
                  << start.x << "," << start.y << ") ate ("
                  << goal.x  << "," << goal.y  << ").\n";
        return {};
    }

    return reconstruct_path(start, goal, parent);
}

// NÓ ROS2 (chama /get_map, /move_command e executa a rota)

class MazeNavigator : public rclcpp::Node {
public:
    MazeNavigator()
    : rclcpp::Node("maze_navigator")
    {
        get_map_client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        move_client_    = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    }

    void run() {
        RCLCPP_INFO(this->get_logger(), "Solicitando mapa...");

        if (!get_map_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /get_map nao disponivel.");
            return;
        }

        auto map_req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto map_future = get_map_client_->async_send_request(map_req);

        if (rclcpp::spin_until_future_complete(shared_from_this(), map_future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Falha ao chamar /get_map");
            return;
        }

        auto map_resp = map_future.get();

        if (map_resp->occupancy_grid_shape.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Forma do grid invalida.");
            return;
        }

        int height = map_resp->occupancy_grid_shape[0];
        int width  = map_resp->occupancy_grid_shape[1];
        auto cells = map_resp->occupancy_grid_flattened;

        RCLCPP_INFO(this->get_logger(),
                    "Mapa recebido: %dx%d (cells=%zu)", width, height, cells.size());

        if ((int)cells.size() != width * height) {
            RCLCPP_WARN(this->get_logger(),
                "Tamanho do vetor de celulas (%zu) difere de width*height (%d).",
                cells.size(), width * height);
        }

        // Agora precisamos descobrir as posicoes do robo e do alvo pelo MoveCmd
        if (!move_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /move_command nao disponivel.");
            return;
        }

        auto move_req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        move_req->direction = "";  // comando dummy so pra obter posicoes

        RCLCPP_INFO(this->get_logger(),
                    "Chamando /move_command uma vez para obter robot_pos e target_pos...");

        auto move_future = move_client_->async_send_request(move_req);
        if (rclcpp::spin_until_future_complete(shared_from_this(), move_future)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Falha ao chamar /move_command");
            return;
        }

        auto move_resp = move_future.get();

        if (move_resp->robot_pos.size() < 2 || move_resp->target_pos.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Resposta de /move_command invalida (posicoes).");
            return;
        }

        Cell start{ move_resp->robot_pos[0], move_resp->robot_pos[1] };
        Cell goal { move_resp->target_pos[0], move_resp->target_pos[1] };

        RCLCPP_INFO(this->get_logger(),
                    "Posicao do robo: (%d, %d), alvo: (%d, %d)",
                    start.x, start.y, goal.x, goal.y);

        // Calcula caminho a partir da posicao atual do robo (depois desse movimento)
        auto path = bfs_find_path(width, height, cells, start, goal);
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Nenhuma rota encontrada.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Executando caminho com %zu passos...", path.size());
        execute_path(path);
    }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;

    void execute_path(const std::vector<Direction> &path) {
        if (!move_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Servico /move_command nao disponivel.");
            return;
        }

        for (Direction d : path) {
            auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();

            switch (d) {
                case Direction::UP:    req->direction = "up";    break;
                case Direction::DOWN:  req->direction = "down";  break;
                case Direction::LEFT:  req->direction = "left";  break;
                case Direction::RIGHT: req->direction = "right"; break;
            }

            RCLCPP_INFO(this->get_logger(), "Movendo: %s", req->direction.c_str());

            auto future = move_client_->async_send_request(req);
            rclcpp::spin_until_future_complete(shared_from_this(), future);

            rclcpp::sleep_for(150ms);
        }

        RCLCPP_INFO(this->get_logger(), "Caminho concluido!");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeNavigator>();
    node->run();   // faz tudo de uma vez
    rclcpp::shutdown();
    return 0;
}
