//
// Tests for the STN trajectory logger (stn-logging branch).
//

#include "gtest/gtest.h"
#include "stn_logger.hpp"
#include "lahc.hpp"
#include "ma.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

fs::path data_path() { return fs::path(__FILE__).parent_path().parent_path() / "data"; }

std::vector<std::string> read_lines(const fs::path& path) {
    std::ifstream in(path);
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) lines.push_back(line);
    return lines;
}

class StnLoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        instance = std::make_unique<Case>(data_path().string(), "E-n22-k4.evrp");  // customers 1-21, stations 22-29
        preprocessor = std::make_unique<Preprocessor>(*instance, params);
    }

    std::unique_ptr<Individual> make(const std::vector<std::vector<int>>& routes) const {
        return std::make_unique<Individual>(instance.get(), preprocessor.get(), routes, 0.0,
                                            instance->compute_demand_sum_per_route(routes));
    }

    Parameters params;
    std::unique_ptr<Case> instance;
    std::unique_ptr<Preprocessor> preprocessor;
};

}  // namespace

TEST(StnSignatureTest, UsesFnv1a64) {
    EXPECT_EQ(StnLogger::signature(""), "cbf29ce484222325");
    EXPECT_EQ(StnLogger::signature("a"), "af63dc4c8601ec8c");
}

TEST_F(StnLoggerTest, CanonicalFormIgnoresDepotStationsDirectionAndRouteOrder) {
    const auto a = make({{0, 1, 2, 22, 3, 0}, {0, 4, 5, 0}});
    const auto b = make({{0, 5, 4, 0}, {0, 3, 23, 2, 1, 0}});
    EXPECT_EQ(StnLogger::canonical_routes(*a, *instance), "1-2-3|4-5");
    EXPECT_EQ(StnLogger::canonical_routes(*b, *instance), "1-2-3|4-5");

    const auto c = make({{0, 1, 3, 2, 0}, {0, 4, 5, 0}});  // a different visiting order
    EXPECT_NE(StnLogger::signature(StnLogger::canonical_routes(*a, *instance)),
              StnLogger::signature(StnLogger::canonical_routes(*c, *instance)));
}

TEST_F(StnLoggerTest, ObservationsFollowTheGridAndDoNotUseTheBudget) {
    const fs::path directory = fs::temp_directory_path() / "ecvrp_stn_logger_test";
    fs::remove_all(directory);
    const auto individual = make({{0, 1, 2, 0}});
    {
        StnLogger logger(*instance, directory, 1000, 0.1, true);  // grid every 100 distance calls
        const std::uint64_t calls_before = instance->get_distance_calls();
        for (std::uint64_t calls : {5ULL, 50ULL, 120ULL, 150ULL, 330ULL}) {
            logger.observe(*individual, static_cast<long>(calls), calls);
        }
        logger.observe(*individual, 340, 340, true);
        EXPECT_EQ(instance->get_distance_calls(), calls_before);
    }
    const auto raw = read_lines(directory / "stn_raw.E-n22-k4.csv");
    ASSERT_EQ(raw.size(), 5U);  // header + 5, 120, 330, 340 (forced)
    EXPECT_EQ(raw[1].rfind("5,5,", 0), 0U);
    EXPECT_EQ(raw[2].rfind("120,120,", 0), 0U);
    EXPECT_EQ(raw[3].rfind("330,330,", 0), 0U);
    EXPECT_EQ(raw[4].rfind("340,340,", 0), 0U);
    EXPECT_EQ(read_lines(directory / "stn_solutions.E-n22-k4.csv").size(), 2U);  // one signature
    fs::remove_all(directory);
}

namespace {

struct RunResult {
    double lower_cost;
    std::uint64_t distance_calls;
};

template <typename Algorithm>
RunResult run_once(const bool stn) {
    Parameters params;
    params.k_stats_path = (fs::temp_directory_path() / "ecvrp_stn_run_test").string();
    params.enable_stn_logging = stn;
    params.ma_ls_policy = LocalSearchPolicy::Static;
    params.ma_static_unrestricted = true;
    Case instance(data_path().string(), "E-n22-k4.evrp");
    Preprocessor preprocessor(instance, params);
    Algorithm algorithm(3, &instance, &preprocessor);
    const double cost = algorithm.run();
    return {cost, instance.get_distance_calls()};
}

}  // namespace

TEST(StnLoggingRunTest, LoggingDoesNotChangeLahc) {
    const RunResult off = run_once<Lahc>(false);
    const RunResult on = run_once<Lahc>(true);
    EXPECT_EQ(off.lower_cost, on.lower_cost);
    EXPECT_EQ(off.distance_calls, on.distance_calls);
    fs::remove_all(fs::temp_directory_path() / "ecvrp_stn_run_test");
}

TEST(StnLoggingRunTest, LoggingDoesNotChangeMa) {
    const RunResult off = run_once<Ma>(false);
    const RunResult on = run_once<Ma>(true);
    EXPECT_EQ(off.lower_cost, on.lower_cost);
    EXPECT_EQ(off.distance_calls, on.distance_calls);
    fs::remove_all(fs::temp_directory_path() / "ecvrp_stn_run_test");
}
