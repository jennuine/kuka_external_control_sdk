// Copyright 2026 KUKA Hungaria Kft.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#define private public
#include "kuka/external-control-sdk/kss/mxa/client.h"
#undef private

using kuka::external::control::ControlMode;
using kuka::external::control::EventHandler;
using kuka::external::control::kss::CycleTime;
using kuka::external::control::kss::mxa::Client;

namespace
{

class RecordingEventHandler : public EventHandler
{
public:
  void OnSampling() override { sampling_count++; }
  void OnError(const std::string & reason) override { errors.push_back(reason); }

  int sampling_count = 0;
  std::vector<std::string> errors;
};

// Drives the mxA function blocks through the shared axis group state, the way
// the controller would, without a KRC
class MxaRsiStart : public ::testing::Test
{
protected:
  static constexpr int kAxisGroup = 1;
  static constexpr int kTechFunctionCmdId = 35;
  static constexpr int kOrderStateDone = 6;
  static constexpr int kOrderStateAborted = 8;

  void SetUp() override
  {
    auto & group = KRC_AXISGROUPREFARR[kAxisGroup];
    group = AXIS_GROUP_REF{};
    group.INITIALIZED = true;
    group.ONLINE = true;
    group.READDONE = true;
    group.READAXISGROUPINIT = true;
    group.AUTEXTSTATE.EXT = true;
    group.AUTEXTSTATE.ALARM_STOP = true;
    group.AUTEXTSTATE.USER_SAFE = true;
    group.AUTEXTSTATE.PERI_RDY = true;
    group.AUTEXTSTATE.PRO_ACT = true;

    auto handler = std::make_unique<RecordingEventHandler>();
    handler_ = handler.get();
    client_.RegisterEventHandler(std::move(handler));
  }

  void TearDown() override
  {
    // The destructor would wait for the keep-alive thread to cancel
    client_.ResetRSI();
  }

  void StartRsi() { client_.StartRSI(ControlMode::JOINT_POSITION_CONTROL, CycleTime::RSI_4MS); }

  // One keep-alive cycle, with the KRC echoing every new command
  void Cycle()
  {
    client_.mxa_wrapper_.mxa_aut_ext_.OnCycle();
    client_.HandleRsiStart();

    auto & group = KRC_AXISGROUPREFARR[kAxisGroup];
    if (group.COMMAND.ORDERID != 0 && group.COMMAND.ORDERID != group.CMDSTATE.ORDERIDRET)
    {
      EXPECT_EQ(group.COMMAND.COMMANDID, kTechFunctionCmdId);
      issued_.push_back(group.COMMAND.ORDERID);
      group.CMDSTATE.ORDERIDRET = group.COMMAND.ORDERID;
      group.CMDSTATE.CMDIDRET = group.COMMAND.COMMANDID;
    }
  }

  void Cycles(int count)
  {
    for (int i = 0; i < count; i++)
    {
      Cycle();
    }
  }

  void ReportOrderState(int order_id, int state)
  {
    auto & state_return = KRC_AXISGROUPREFARR[kAxisGroup].CMDSTATE.STATERETURN[1];
    state_return.SR_ORDERID = order_id;
    state_return.SR_STATE = state;
  }

  Client client_{"127.0.0.1", 1};
  RecordingEventHandler * handler_ = nullptr;
  std::vector<int> issued_;
};

TEST_F(MxaRsiStart, RunningDispatcherGetsOneStartCommand)
{
  StartRsi();
  Cycles(50);

  EXPECT_EQ(issued_.size(), 1u);
  EXPECT_FALSE(client_.start_cmd_dispatcher_);
  EXPECT_TRUE(client_.start_rsi_);
  EXPECT_EQ(handler_->sampling_count, 1);
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, SecondStartAfterCancelGetsANewCommand)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);
  client_.ResetRSI();

  StartRsi();
  Cycles(5);

  ASSERT_EQ(issued_.size(), 2u);
  EXPECT_GT(issued_[1], issued_[0]);
}

TEST_F(MxaRsiStart, AbortedStartIsReissuedOnce)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  ReportOrderState(issued_[0], kOrderStateAborted);
  Cycles(50);

  ASSERT_EQ(issued_.size(), 2u);
  EXPECT_EQ(issued_[1], issued_[0] + 1);
  EXPECT_TRUE(client_.start_rsi_);
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, GivesUpAfterTheRetryCap)
{
  StartRsi();
  for (int attempt = 0; attempt <= Client::kMaxRsiStartRetries; attempt++)
  {
    Cycles(5);
    ASSERT_EQ(issued_.size(), static_cast<size_t>(attempt + 1));
    ReportOrderState(issued_.back(), kOrderStateAborted);
  }
  Cycles(50);

  EXPECT_EQ(issued_.size(), static_cast<size_t>(Client::kMaxRsiStartRetries + 1));
  EXPECT_FALSE(client_.start_rsi_);
  ASSERT_EQ(handler_->errors.size(), 1u);
  EXPECT_NE(handler_->errors[0].find("aborted"), std::string::npos);
}

TEST_F(MxaRsiStart, DoneEndsTheStart)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  ReportOrderState(issued_[0], kOrderStateDone);
  Cycles(50);

  EXPECT_EQ(issued_.size(), 1u);
  EXPECT_FALSE(client_.start_rsi_);
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, StoppedDispatcherIsStartedBeforeTheCommand)
{
  auto & group = KRC_AXISGROUPREFARR[kAxisGroup];
  group.AUTEXTSTATE.PRO_ACT = false;

  StartRsi();
  Cycles(10);
  EXPECT_TRUE(issued_.empty());
  EXPECT_TRUE(client_.start_cmd_dispatcher_);
  EXPECT_TRUE(group.KRCCONTROL.RESET);

  group.AUTEXTSTATE.PRO_ACT = true;
  Cycles(50);

  EXPECT_EQ(issued_.size(), 1u);
  EXPECT_FALSE(client_.start_cmd_dispatcher_);
}

}  // namespace
