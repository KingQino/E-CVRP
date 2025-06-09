//
// Created by Yinghao Qin on 08/06/2025.
//

#include "gtest/gtest.h"
#include "command_line.hpp"

#include <filesystem>

namespace fs = std::filesystem;

TEST(CommandLineTest, ParseParametersPathOutput) {
    // 模拟可执行文件路径，必须是有效路径，否则 CommandLine 会报错
    fs::path fake_executable = fs::current_path() / "Run";

    const std::string fake_exe_str = fake_executable.string();

    const char* argv[] = {
        fake_exe_str.c_str(),  // argv[0]
        "-alg", "Lahc",
        "-ins", "large.evrp",
        "-log", "1",
        "-stp", "0",
        "-mth", "0",
    };
    constexpr int argc = std::size(argv);

    CommandLine cmd(argc, const_cast<char**>(argv));
    Parameters params;

    ::testing::internal::CaptureStdout();
    cmd.parse_parameters(params);
    std::string output = ::testing::internal::GetCapturedStdout();

    EXPECT_EQ(params.algorithm, Algorithm::LAHC);
    EXPECT_EQ(params.instance, "large.evrp");
    EXPECT_TRUE(params.enable_logging);
    EXPECT_EQ(params.stop_criteria, 0);
    EXPECT_FALSE(params.enable_multithreading);

    EXPECT_TRUE(output.find("[INFO] Default data path:") != std::string::npos);
    EXPECT_TRUE(output.find("[INFO] Default stats path:") != std::string::npos);
}
