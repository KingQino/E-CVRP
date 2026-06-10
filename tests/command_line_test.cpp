//
// Created by Yinghao Qin on 08/06/2025.
//

#include "gtest/gtest.h"
#include "command_line.hpp"

#include <filesystem>

namespace fs = std::filesystem;

namespace {

fs::path find_fake_executable() {
    fs::path fake_executable = fs::current_path() / "Tests";
    if (!fs::exists(fake_executable)) {
        fake_executable = fs::current_path() / "Run";
    }
    if (!fs::exists(fake_executable)) {
        fake_executable = fs::path(__FILE__).parent_path().parent_path() / "build-debug" / "Tests";
    }
    return fake_executable;
}

} // namespace

TEST(CommandLineTest, ParseParametersPathOutput) {
    const fs::path fake_executable = find_fake_executable();
    ASSERT_TRUE(fs::exists(fake_executable));

    const std::string fake_exe_str = fake_executable.string();

    const char* argv[] = {
        fake_exe_str.c_str(),  // argv[0]
        "-alg", "TwoStageAlns",
        "-ins", "large.evrp",
        "-log", "1",
        "-stp", "0",
        "-mth", "0",
        "-alns_start_dev", "0.05",
        "-alns_decay", "0.97",
        "-alns_min_destroy_abs", "7",
        "-alns_repair_bias", "1.25",
    };
    constexpr int argc = std::size(argv);

    CommandLine cmd(argc, const_cast<char**>(argv));
    Parameters params;

    ::testing::internal::CaptureStdout();
    cmd.parse_parameters(params);
    std::string output = ::testing::internal::GetCapturedStdout();

    EXPECT_EQ(params.algorithm, Algorithm::TWOSTAGEALNS);
    EXPECT_EQ(params.instance, "large.evrp");
    EXPECT_TRUE(params.enable_logging);
    EXPECT_EQ(params.stop_criteria, 0);
    EXPECT_FALSE(params.enable_multithreading);
    EXPECT_DOUBLE_EQ(params.alns_start_deviation, 0.05);
    EXPECT_DOUBLE_EQ(params.alns_decay, 0.97);
    EXPECT_EQ(params.alns_min_destroy_abs, 7);
    EXPECT_DOUBLE_EQ(params.alns_repair_rand_factor, 1.25);

    EXPECT_TRUE(output.find("[INFO] Default data path:") != std::string::npos);
    EXPECT_TRUE(output.find("[INFO] Default stats path:") != std::string::npos);
}

TEST(CommandLineTest, ParseTwoStageAlnsAlgorithmAlias) {
    const fs::path fake_executable = find_fake_executable();
    ASSERT_TRUE(fs::exists(fake_executable));

    const std::string fake_exe_str = fake_executable.string();

    const char* argv[] = {
        fake_exe_str.c_str(),
        "-alg", "two_stage_alns",
        "-ins", "E-n29-k4-s7.evrp",
    };
    constexpr int argc = std::size(argv);

    CommandLine cmd(argc, const_cast<char**>(argv));
    Parameters params;
    ::testing::internal::CaptureStdout();
    cmd.parse_parameters(params);
    ::testing::internal::GetCapturedStdout();

    EXPECT_EQ(params.algorithm, Algorithm::TWOSTAGEALNS);
}

TEST(CommandLineTest, ShortHelpFlagExitsSuccessfully) {
    const fs::path fake_executable = find_fake_executable();
    ASSERT_TRUE(fs::exists(fake_executable));

    const std::string fake_exe_str = fake_executable.string();

    const char* argv[] = {
        fake_exe_str.c_str(),
        "-h",
    };
    constexpr int argc = std::size(argv);

    EXPECT_EXIT(
        {
            CommandLine cmd(argc, const_cast<char**>(argv));
            (void)cmd;
        },
        ::testing::ExitedWithCode(0),
        "");
}

TEST(CommandLineTest, LongHelpFlagExitsSuccessfully) {
    const fs::path fake_executable = find_fake_executable();
    ASSERT_TRUE(fs::exists(fake_executable));

    const std::string fake_exe_str = fake_executable.string();

    const char* argv[] = {
        fake_exe_str.c_str(),
        "-help",
    };
    constexpr int argc = std::size(argv);

    EXPECT_EXIT(
        {
            CommandLine cmd(argc, const_cast<char**>(argv));
            (void)cmd;
        },
        ::testing::ExitedWithCode(0),
        "");
}

TEST(CommandLineTest, UnknownAlgorithmExitsWithError) {
    const fs::path fake_executable = find_fake_executable();
    ASSERT_TRUE(fs::exists(fake_executable));

    const std::string fake_exe_str = fake_executable.string();

    const char* argv[] = {
        fake_exe_str.c_str(),
        "-alg", "cbma",
    };
    constexpr int argc = std::size(argv);

    EXPECT_EXIT(
        {
            CommandLine cmd(argc, const_cast<char**>(argv));
            Parameters params;
            cmd.parse_parameters(params);
        },
        ::testing::ExitedWithCode(1),
        "Unknown algorithm 'cbma'");
}
