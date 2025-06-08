//
// Created by Yinghao Qin on 08/06/2025.
//

#include "case.hpp"

Case::Case(const string& kDataPath, const string& file_name) {
    this->file_name_ = file_name;
    this->instance_name_ = file_name.substr(0, file_name.find('.'));

    std::string full_path = kDataPath;
    if (!full_path.empty() && full_path.back() != '/')
        full_path += '/';
    full_path += file_name;

    this->read_problem(full_path);
}


Case::~Case() {
    for (int i = 0; i < problem_size_; i++) {
        delete[] this->distances_[i];
    }
    delete[] this->distances_;
}

void Case::read_problem(const std::string &file_path) {
    this->num_depot_ = 1;

    std::ifstream infile(file_path);
    if (!infile.is_open()) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string line;
    std::stringstream ss;

    // === 1. read head parameters ===
    while (std::getline(infile, line)) {
        if (line.find("VEHICLES") != std::string::npos) {
            ss.clear(); ss.str(line.substr(line.find(':') + 1));
            ss >> this->num_vehicle_;
        } else if (line.find("CAPACITY") != std::string::npos && line.find("ENERGY") == std::string::npos) {
            ss.clear(); ss.str(line.substr(line.find(':') + 1));
            ss >> this->max_vehicle_capa_;
        } else if (line.find("ENERGY_CAPACITY") != std::string::npos) {
            ss.clear(); ss.str(line.substr(line.find(':') + 1));
            ss >> this->max_battery_capa_;
        } else if (line.find("ENERGY_CONSUMPTION") != std::string::npos) {
            ss.clear(); ss.str(line.substr(line.find(':') + 1));
            ss >> this->energy_consumption_rate_;
        } else if (line.find("OPTIMAL_VALUE") != std::string::npos) {
            std::string val = line.substr(line.find(':') + 1);
            val.erase(0, val.find_first_not_of(" \t")); // 去除前导空格
            ss.clear(); ss.str(val);
            ss >> this->optimum_;
        } else if (line.find("DEMAND_SECTION") != std::string::npos) {
            break;
        }
    }

    // === 2. DEMAND_SECTION: depot + customers ===
    demand_.clear();
    while (std::getline(infile, line)) {
        if (line.empty() || !isdigit(line[0])) break;

        ss.clear(); ss.str(line);
        int id, d;
        ss >> id >> d;

        if (demand_.size() < id)
            demand_.resize(id, 0);
        demand_[id - 1] = d;

        if (d == 0)
            depot_ = id - 1;  // depot index in file
    }

    this->num_customer_ = static_cast<int>(demand_.size()) - this->num_depot_; // exclude depot

    // === 3. STATIONS_COORD_SECTION: station coordinates ===
    while (std::getline(infile, line)) {
        if (line.find("STATIONS_COORD_SECTION") != std::string::npos)
            continue;
        if (line.find("NODE_COORD_SECTION") != std::string::npos)
            break;
        if (line.empty() || !isdigit(line[0])) break;

        ++this->num_station_;
    }

    // === 4. POSITIONS_: depot + customer + stations ===
    this->problem_size_ = static_cast<int>(demand_.size()) + num_station_;
    positions_.resize(problem_size_, {0, 0});

    // Go back to read NODE_COORD_SECTION
    infile.clear();
    infile.seekg(0);
    while (std::getline(infile, line)) {
        if (line.find("NODE_COORD_SECTION") != std::string::npos) break;
    }

    for (int i = 0; i < problem_size_; ++i) {
        std::getline(infile, line);
        ss.clear(); ss.str(line);
        int id;
        double x, y;
        ss >> id >> x >> y;
        positions_[id - 1] = {x, y};
    }

    this->dimension_ = this->problem_size_;

    infile.close();

    // === 5. initialise distance matrix  ===
    this->max_service_time_ = std::numeric_limits<double>::max();
    this->distances_ = generate_2D_matrix_double(problem_size_, problem_size_);
    for (int i = 0; i < problem_size_; ++i) {
        for (int j = 0; j < problem_size_; ++j) {
            distances_[i][j] = euclidean_distance(i, j);
        }
    }
}

double **Case::generate_2D_matrix_double(const int n, const int m) {
    auto **matrix = new double *[n];
    for (int i = 0; i < n; i++) {
        matrix[i] = new double[m];
    }
    //initialise the 2-d array
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            matrix[i][j] = 0.0;
        }
    }
    return matrix;
}

double Case::euclidean_distance(const int i, const int j) const {
    return sqrt(pow(positions_[i].first - positions_[j].first, 2) +
                pow(positions_[i].second - positions_[j].second, 2));
}

int Case::get_customer_demand_(const int customer) const {
    return demand_[customer];
}

double Case::get_distance(const int from, const int to) {
    //adds partial evaluation to the overall fitness evaluation count
    //It can be used when local search is used and a whole evaluation is not necessary
    evals_ += 1.0 / problem_size_;

    return distances_[from][to];
}

double Case::get_evals() const {
    return evals_;
}

double Case::calculate_total_dist(const vector<vector<int>>& chromR) const {
    double tour_length = 0.0;

    for (const auto& route : chromR) {
        if (route.empty()) continue;

        tour_length += distances_[depot_][route[0]];
        for (int j = 0; j < route.size() - 1; ++j) {
            tour_length += distances_[route[j]][route[j + 1]];
        }
        tour_length += distances_[route.back()][depot_];
    }

    return tour_length;
}

double Case::calculate_total_dist_follower(int **routes, const int num_routes, const int *num_nodes_per_route) const {
    double tour_length = 0.0;
    for (int i = 0; i < num_routes; ++i) {
        for (int j = 0; j < num_nodes_per_route[i] - 1; ++j) {
            tour_length += distances_[routes[i][j]][routes[i][j + 1]];
        }
    }

    return tour_length;
}

int Case::calculate_demand_sum(const vector<int> &route) const {
    int demand_sum = 0;
    for(const auto node : route) {
        demand_sum += demand_[node];
    }

    return demand_sum;
}

double Case::compute_total_distance(const vector<vector<int>> &routes) {
    double tour_length = 0.0;
    for (auto& route : routes) {
        for (int j = 0; j < route.size() - 1; ++j) {
            tour_length += distances_[route[j]][route[j + 1]];
        }
    }

    evals_++;

    return tour_length;
}

double Case::compute_total_distance(const vector<int> &route) const {
    double tour_length = 0.0;
    for (int j = 0; j < route.size() - 1; ++j) {
        tour_length += distances_[route[j]][route[j + 1]];
    }

    return tour_length;
}

vector<int> Case::compute_demand_sum_per_route(const vector<vector<int>> &routes) const {
    vector<int> demand_sum_per_route;
    for (auto & route : routes) {
        int temp = 0;
        for (const int node : route) {
            temp += get_customer_demand_(node);
        }
        demand_sum_per_route.push_back(temp);
    }

    return std::move(demand_sum_per_route);
}

bool Case::is_charging_station(const int node) const {
    bool flag;
    if (node == depot_ || ( node >= num_depot_ + num_customer_ && node < problem_size_))
        flag = true;
    else
        flag = false;
    return flag;
}

std::ostream& operator<<(std::ostream& os, const Case& c) {
    os << "Case: " << c.instance_name_ << "\n"
       << "  Dimension   : " << c.dimension_ << "\n"
       << "  Depot index: " << c.depot_ << "\n"
       << "  # Customers : " << c.num_customer_ << "\n"
       << "  # Stations  : " << c.num_station_ << "\n"
       << "  # Vehicles  : " << c.num_vehicle_ << "\n"
       << "  Max Capacity: " << c.max_vehicle_capa_ << "\n"
       << "  Battery Cap : " << c.max_battery_capa_ << "\n"
       << "  Energy Rate : " << c.energy_consumption_rate_ << "\n"
       << "  Known Optimum: " << c.optimum_ << "\n"
       << "  Total Nodes: " << c.problem_size_ << "\n"
       << "  Demand (depot + customers): [";

    for (size_t i = 0; i < c.demand_.size(); ++i) {
        os << c.demand_[i];
        if (i < c.demand_.size() - 1) os << ", ";
    }
    os << "]\n";

    os << "  Node Positions (x, y): \n";
    for (size_t i = 0; i < c.positions_.size(); ++i) {
        os << "    Node " << i << ": (" << c.positions_[i].first << ", " << c.positions_[i].second << ")";
        if (c.is_charging_station(static_cast<int>(i))) os << " [CS]";
        if (i == c.depot_) os << " [Depot]";
        os << "\n";
    }

    return os;
}
