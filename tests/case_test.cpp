//
// Created by Yinghao Qin on 08/06/2025.
//

#include "gtest/gtest.h"
#include "case.hpp"

using namespace ::testing;

class CaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        const std::string file_name_ = "E-n29-k4-s7.evrp";
        const std::string kDataPath = "../data";  // 根据你实际路径调整
        instance = new Case(kDataPath, file_name_);
    }

    void TearDown() override {
        delete instance;
    }

    Case* instance{};
};

TEST_F(CaseTest, Constructor) {
    SCOPED_TRACE("Case constructing...");

    // 基本结构检查
    EXPECT_EQ(instance->depot_, 0);
    EXPECT_EQ(instance->num_customer_, 21);
    EXPECT_EQ(instance->num_station_, 7);
    EXPECT_EQ(instance->problem_size_, 29);
    EXPECT_EQ(instance->dimension_, 29);

    // 需求值验证：第3个节点（文件id=3，索引=2） → demand = 700
    EXPECT_EQ(instance->get_customer_demand_(2), 700);

    // 充电站位置（索引22~28），应该全部是 CS
    for (int i = 22; i <= 28; ++i) {
        EXPECT_TRUE(instance->is_charging_station(i)) << "Node " << i << " should be a charging station";
    }

    // 非充电站位置（如顾客）
    EXPECT_FALSE(instance->is_charging_station(1));  // 顾客
    EXPECT_FALSE(instance->is_charging_station(13)); // 顾客
    EXPECT_TRUE(instance->is_charging_station(0));   // 仓库（特例，也当作CS）

    // 安全性测试：不越界、不访问 Node 29（不存在）
    EXPECT_LT(28, instance->problem_size_);
    EXPECT_FALSE(instance->is_charging_station(29)); // 不应测试越界节点
}
