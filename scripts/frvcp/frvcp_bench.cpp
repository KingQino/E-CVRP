// Route-level timing: SE (first feasible count) + RE fallback, SE (both counts) + RE fallback, exact DP.
// Usage: frvcp_bench <data_dir> <route_sample.csv> <out.csv>
#include "follower.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <filesystem>
namespace fs = std::filesystem;
using clk = std::chrono::steady_clock;

static std::string find_file(const fs::path& root, const std::string& name) {
    for (auto& p : fs::recursive_directory_iterator(root))
        if (p.path().filename() == name + ".evrp") return p.path().parent_path().string();
    return "";
}

int main(int argc, char** argv) {
    fs::path data = argv[1];
    std::ifstream in(argv[2]); std::ofstream out(argv[3]);
    out << "instance,m,se1_cost,se2_cost,dp_cost,se1_us,se2_us,dp_us,se1_reads,se2_reads,dp_reads\n";
    std::string line; std::getline(in, line);
    std::map<std::string, std::vector<std::vector<int>>> routes;
    std::vector<std::string> order;
    while (std::getline(in, line)) {
        std::stringstream ss(line); std::string inst, seed, stage, hc, ec, r;
        std::getline(ss, inst, ','); std::getline(ss, seed, ','); std::getline(ss, stage, ',');
        std::getline(ss, hc, ','); std::getline(ss, ec, ','); std::getline(ss, r, ',');
        std::vector<int> v; std::stringstream rs(r); std::string t;
        while (std::getline(rs, t, '-')) v.push_back(std::stoi(t));
        if (!routes.count(inst)) order.push_back(inst);
        routes[inst].push_back(v);
    }
    for (auto& inst : order) {
        Parameters params;
        Case c(find_file(data, inst), inst + ".evrp");
        Preprocessor pre(c, params);
        Follower f(&c, &pre);
        for (auto& v : routes[inst]) {
            const int n = (int)v.size();
            auto heur = [&](bool both) {
                f.se_compare_station_counts = both;
                double x = f.eval_simple_enum_cost(v.data(), n);
                if (x < 0) x = f.eval_remove_enum_cost(v.data(), n);
                return x;
            };
            double res[3]; double us[3]; std::uint64_t reads[3];
            for (int k = 0; k < 3; ++k) {
                // reads
                std::uint64_t r0 = c.get_distance_calls();
                res[k] = k == 2 ? f.eval_optimal_cost(v.data(), n) : heur(k == 1);
                reads[k] = c.get_distance_calls() - r0;
                // time: repeat until >= 2 ms
                int reps = 0; auto t0 = clk::now(); double el = 0;
                volatile double sink = 0;
                do {
                    sink += k == 2 ? f.eval_optimal_cost(v.data(), n) : heur(k == 1);
                    ++reps; el = std::chrono::duration<double, std::micro>(clk::now() - t0).count();
                } while (el < 2000 && reps < 100000);
                us[k] = el / reps;
            }
            out << inst << ',' << n - 1 << ',' << res[0] << ',' << res[1] << ',' << res[2] << ',' << us[0] << ','
                << us[1] << ',' << us[2] << ',' << reads[0] << ',' << reads[1] << ',' << reads[2] << '\n';
        }
        std::cerr << inst << ' ' << routes[inst].size() << '\n';
    }
}
