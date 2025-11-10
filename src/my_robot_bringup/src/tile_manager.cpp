#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <gazebo_msgs/msg/entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <cmath>
#include <memory>
#include <string>

struct TileData {
    double x;
    double y;
    bool active;
    bool triggered;
};

class TileManager : public rclcpp::Node {
public:
    TileManager() : Node("tile_manager"),
                    current_spawn_index_(0),
                    robot_x_(0.0),
                    robot_y_(0.0) {
        // Declare parameters
        this->declare_parameter<int>("grid_size", 5);
        this->declare_parameter<double>("tile_spacing", 0.5);
        this->declare_parameter<std::string>("mode", "disappear");
        this->declare_parameter<double>("detection_radius", 0.3);
        this->declare_parameter<std::string>("scenario_file", "");

        // Get parameters
        grid_size_ = this->get_parameter("grid_size").as_int();
        tile_spacing_ = this->get_parameter("tile_spacing").as_double();
        mode_ = this->get_parameter("mode").as_string();
        detection_radius_ = this->get_parameter("detection_radius").as_double();
        scenario_file_ = this->get_parameter("scenario_file").as_string();

        // Create service clients
        spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
        set_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");

        // Subscribe to robot odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diff_drive_controller/odom",
            10,
            std::bind(&TileManager::odomCallback, this, std::placeholders::_1)
        );

        // Wait for services
        RCLCPP_INFO(this->get_logger(), "Waiting for Gazebo services...");

        if (!spawn_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Spawn service not available");
        }
        if (!delete_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Delete service not available");
        }
        if (!set_state_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Set entity state service not available");
        }

        // Load tile SDF template
        loadTileSDF();

        // Prepare tile spawn queue
        prepareTileQueue();

        // Start spawning the first tile (chain reaction will spawn the rest)
        if (!spawn_queue_.empty()) {
            spawnNextTile();
        }

        RCLCPP_INFO(this->get_logger(), "Tile Manager initialized: %dx%d grid, mode=%s",
                    grid_size_, grid_size_, mode_.c_str());
    }

private:
    struct TileSpawnInfo {
        std::string name;
        std::string type;  // "tile" or "wall"
        double x;
        double y;
    };

    void loadTileSDF() {
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("my_robot_description");

            // Load tile model
            std::string tile_model_path = package_share_dir + "/models/tile/model.sdf";
            std::ifstream tile_file(tile_model_path);
            if (!tile_file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Tile model not found at %s", tile_model_path.c_str());
            } else {
                std::stringstream buffer;
                buffer << tile_file.rdbuf();
                tile_sdf_template_ = buffer.str();
                tile_file.close();
                RCLCPP_INFO(this->get_logger(), "Loaded tile SDF template");
            }

            // Load wall model
            std::string wall_model_path = package_share_dir + "/models/brick_wall/model.sdf";
            std::ifstream wall_file(wall_model_path);
            if (!wall_file.is_open()) {
                RCLCPP_WARN(this->get_logger(), "Wall model not found at %s", wall_model_path.c_str());
            } else {
                std::stringstream buffer;
                buffer << wall_file.rdbuf();
                wall_sdf_template_ = buffer.str();
                wall_file.close();
                RCLCPP_INFO(this->get_logger(), "Loaded wall SDF template");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load SDF templates: %s", e.what());
        }
    }

    void parseScenarioFile() {
        std::ifstream file(scenario_file_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open scenario file: %s", scenario_file_.c_str());
            return;
        }

        std::vector<std::string> grid;
        std::string line;

        // Read file and filter out comment lines
        // Comments start with // or ; (not # which is used for walls)
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            // Skip comment lines (// or ;)
            if (line.length() >= 2 && line[0] == '/' && line[1] == '/') continue;
            if (line[0] == ';') continue;
            grid.push_back(line);
        }
        file.close();

        if (grid.empty()) {
            RCLCPP_WARN(this->get_logger(), "Scenario file is empty or contains only comments");
            return;
        }

        // Calculate grid dimensions
        int rows = grid.size();
        int cols = grid[0].length();

        // Calculate offset to center the grid
        double offset_x = (cols - 1) * tile_spacing_ / 2.0;
        double offset_y = (rows - 1) * tile_spacing_ / 2.0;

        int tile_count = 0;
        int wall_count = 0;

        // Parse the grid
        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < static_cast<int>(grid[row].length()); col++) {
                char cell = grid[row][col];

                if (cell == '.' || cell == ' ' || cell == 'R') {
                    // Empty cell or robot marker - skip
                    continue;
                }

                TileSpawnInfo info;
                double x = col * tile_spacing_ - offset_x;
                double y = -(row * tile_spacing_ - offset_y);  // Invert Y for correct orientation

                if (cell == 'T') {
                    // Interactive tile
                    info.type = "tile";
                    info.name = "tile_" + std::to_string(tile_count++);
                    info.x = x;
                    info.y = y;

                    spawn_queue_.push_back(info);

                    // Pre-create tile data for tracking
                    TileData tile_data;
                    tile_data.x = x;
                    tile_data.y = y;
                    tile_data.active = true;
                    tile_data.triggered = false;
                    tiles_[info.name] = tile_data;

                } else if (cell == '#') {
                    // Wall
                    info.type = "wall";
                    info.name = "wall_" + std::to_string(wall_count++);
                    info.x = x;
                    info.y = y;

                    spawn_queue_.push_back(info);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loaded scenario: %d tiles, %d walls from file: %s",
                    tile_count, wall_count, scenario_file_.c_str());
    }

    void prepareTileQueue() {
        // Check if scenario file is specified
        if (!scenario_file_.empty()) {
            parseScenarioFile();
        } else {
            // Use old grid-based approach
            double offset = (grid_size_ - 1) * tile_spacing_ / 2.0;

            for (int i = 0; i < grid_size_; i++) {
                for (int j = 0; j < grid_size_; j++) {
                    TileSpawnInfo info;
                    info.type = "tile";
                    info.name = "tile_" + std::to_string(i) + "_" + std::to_string(j);
                    info.x = i * tile_spacing_ - offset;
                    info.y = j * tile_spacing_ - offset;

                    spawn_queue_.push_back(info);

                    // Pre-create tile data
                    TileData tile_data;
                    tile_data.x = info.x;
                    tile_data.y = info.y;
                    tile_data.active = true;
                    tile_data.triggered = false;
                    tiles_[info.name] = tile_data;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Prepared %zu tiles for spawning", spawn_queue_.size());
        }
    }

    void spawnNextTile() {
        if (current_spawn_index_ >= spawn_queue_.size()) {
            // All tiles spawned
            RCLCPP_INFO(this->get_logger(), "Finished spawning all %zu objects", spawn_queue_.size());
            return;
        }

        const auto& info = spawn_queue_[current_spawn_index_];

        // Select the correct SDF template based on type
        std::string sdf_template;
        std::string model_name;

        if (info.type == "tile") {
            sdf_template = tile_sdf_template_;
            model_name = "floor_tile";
        } else if (info.type == "wall") {
            sdf_template = wall_sdf_template_;
            model_name = "brick_wall";
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown object type: %s", info.type.c_str());
            current_spawn_index_++;
            spawnNextTile();  // Continue with next object
            return;
        }

        // Create unique SDF by replacing the model name
        std::string unique_sdf = sdf_template;
        size_t pos = unique_sdf.find(model_name);
        if (pos != std::string::npos) {
            unique_sdf.replace(pos, model_name.length(), info.name);
        }

        // Create spawn request
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = info.name;
        request->xml = unique_sdf;
        request->initial_pose.position.x = info.x;
        request->initial_pose.position.y = info.y;
        request->initial_pose.position.z = 0.0;

        // Increment index before async call
        current_spawn_index_++;

        // Spawn object with callback that triggers next spawn
        spawn_client_->async_send_request(request,
            [this, name = info.name, type = info.type, x = info.x, y = info.y]
            (rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture future) {
                try {
                    auto result = future.get();
                    if (result->success) {
                        RCLCPP_INFO(this->get_logger(), "Successfully spawned %s (%s) at (%.2f, %.2f)",
                                    name.c_str(), type.c_str(), x, y);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to spawn %s: %s",
                                    name.c_str(), result->status_message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception spawning %s: %s",
                                name.c_str(), e.what());
                }

                // Spawn next object after this one completes (success or failure)
                spawnNextTile();
            });
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        // Check which tiles are under the robot
        checkTileTriggers();
    }

    void checkTileTriggers() {
        for (auto& [tile_name, tile_data] : tiles_) {
            if (!tile_data.active) {
                continue;
            }

            // Calculate distance from robot to tile center
            double dx = robot_x_ - tile_data.x;
            double dy = robot_y_ - tile_data.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            // Check if robot is on tile
            if (distance < detection_radius_ && !tile_data.triggered) {
                triggerTile(tile_name, tile_data);
                tile_data.triggered = true;
            }
        }
    }

    void triggerTile(const std::string& tile_name, TileData& tile_data) {
        RCLCPP_INFO(this->get_logger(), "Robot stepped on %s!", tile_name.c_str());

        if (mode_ == "disappear") {
            deleteTile(tile_name);
            tile_data.active = false;
        } else if (mode_ == "color_change") {
            changeTileColor(tile_name, tile_data);
        }
    }

    void deleteTile(const std::string& tile_name) {
        auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        request->name = tile_name;

        delete_client_->async_send_request(request,
            [this, tile_name](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture future) {
                auto result = future.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Deleted %s", tile_name.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to delete %s", tile_name.c_str());
                }
            });
    }

    void changeTileColor(const std::string& tile_name, const TileData&) {
        // Note: Gazebo doesn't support runtime material changes via services
        // Alternative: delete and respawn with different color, or use Gazebo topics
        RCLCPP_INFO(this->get_logger(), "Tile %s color changed (placeholder)", tile_name.c_str());
        // For now, just mark as triggered
        // Future implementation could delete and respawn with red color
    }

    // Member variables
    int grid_size_;
    double tile_spacing_;
    std::string mode_;
    double detection_radius_;
    std::string scenario_file_;

    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_client_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::string tile_sdf_template_;
    std::string wall_sdf_template_;
    std::vector<TileSpawnInfo> spawn_queue_;
    size_t current_spawn_index_;

    std::map<std::string, TileData> tiles_;
    double robot_x_;
    double robot_y_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TileManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
