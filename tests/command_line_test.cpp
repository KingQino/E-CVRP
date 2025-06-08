//
// Created by Yinghao Qin on 08/06/2025.
//

#include "gtest/gtest.h"
#include "command_line.hpp"

using namespace ::testing;


TEST(CommandLineTest, ParseParametersPathOutput) {
    char actual_path[PATH_MAX];
    if (realpath("/Users/yhq/CLionProjects/E-CVRP/cmake-build-debug-llvm/Tests", actual_path) == nullptr) {
        FAIL() << "Failed to resolve real path for /proc/self/exe";
    }

    const char* argv[] = {
        actual_path,
        "-alg", "Lahc",
         "-ins", "large.evrp",
         "-log", "1",
         "-stp", "0",
         "-mth", "0",
    };
    constexpr int argc = std::size(argv);

    CommandLine cmd(argc, const_cast<char**>(argv));
    Parameters params;

    internal::CaptureStdout();
    cmd.parse_parameters(params);
    std::string output = internal::GetCapturedStdout();

    EXPECT_EQ(params.algorithm, Algorithm::LAHC);
    EXPECT_EQ(params.instance, "large.evrp");
    EXPECT_EQ(params.enable_logging, true);
    EXPECT_EQ(params.stop_criteria, 0);
    EXPECT_EQ(params.enable_multithreading, false);

    EXPECT_TRUE(output.find("[INFO] Data path:") != std::string::npos);
    EXPECT_TRUE(output.find("[INFO] Stats path:") != std::string::npos);
}