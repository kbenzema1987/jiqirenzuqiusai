/**
 * @file PlayBall.cpp
 *
 * This file defines an implementation of a skill that plays the ball (under consideration of the skill request).
 *
 * @author Arne Hasselbring
 */

#include "WhiteFleetBackfieldOutlet.h"
#include "WhiteFleetSkillBehaviorControl.h"
#include "WhiteFleetSimPhysical.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "WhiteFleetAutonomousKickoff.h"

// ---- 补：以下头文件按工程实际路径确认 ----
#include "WhiteFleetK1V21MatchProfile.h"   // guardedPhysicalReceiver()
#include "Settings.h"                      // Settings::lowestValidPlayerNumber 等

// 平台检查：不再包含 <cstdlib>（系统头文件，禁止）
#include <cstdio>
#include <cmath>
#include <tuple>
#include <limits>
#include <algorithm>

option((WhiteFleetSkillBehaviorControl) PlayBall,
       load((float) goalPostHandlingAreaRadius,
            (float) goalPostHandlingAreaHysteresisMultiplier,
            (float) duelMinDistanceToClosest,
            (Angle) duelMinAngleToClosest))
{
  // 若以下 provider 未在框架中全局注册，请在此处补充 requires：
  // requires(theClearTarget, theKickAtGoalEvaluation, theIndirectKick, theBehaviorParameters);

  const auto activateDuel = [&]
  {
    if(theSkillRequest.mdfzRule || theSkillRequest.restartPassRequired ||
       (theSkillRequest.backfieldStrongOutlet && theSkillRequest.skill == SkillRequest::pass) ||
       ((theGameState.isKickOff() || theGameState.isFreeKick()) && theGameState.isForOwnTeam()))
      return false;

    const auto [distanceToClosestObstacle, angleToClosestObstacle, smallestAngleToCloseObstacle] =
      [&]() -> std::tuple<float, Angle, Angle>
    {
      float x = std::numeric_limits<float>::max();
      Angle y = 0_deg;
      Angle z = 180_deg;
      for(const auto& o : theObstacleModel.obstacles)
      {
        const float d = (o.center - theFieldBall.positionRelative).squaredNorm();
        if(d < x)
        {
          x = d;
          y = o.center.angle();
          if(x < Math::sqr(duelMinDistanceToClosest))          // 改：sqr -> Math::sqr
            z = std::min<Angle>(Angle(std::abs(y)), z);        // 改：显式模板参数
        }
      }
      return {std::sqrt(x), y, z};
    }();

    const bool obstacleClose = distanceToClosestObstacle <
      (duelMinDistanceToClosest * (1.f - (std::abs(angleToClosestObstacle) / duelMinAngleToClosest / 3.f)));
    const bool obstacleAngleClose = smallestAngleToCloseObstacle <= duelMinAngleToClosest;
    const bool ballSeen = theFieldBall.ballWasSeen(300);
    const bool ballPosXClose = Math::between<float>(theFieldBall.positionRelative.x(), 0.f, 2000.f); // 改：Math::between
    const bool ballPosYClose = std::abs(theFieldBall.positionRelative.y()) < 500.f;
    const bool rotationToBallOk = std::abs(theFieldBall.positionRelative.angle()) < 80_deg;

    return obstacleClose && obstacleAngleClose &&
           ballSeen && ballPosXClose && ballPosYClose && rotationToBallOk;
  };

  const auto stopDuel = [&]
  {
    if(theSkillRequest.restartPassRequired ||
       (theSkillRequest.backfieldStrongOutlet && theSkillRequest.skill == SkillRequest::pass) ||
       ((theGameState.isKickOff() || theGameState.isFreeKick()) && theGameState.isForOwnTeam()))
      return true;

    if(theFieldBall.positionRelative.x() > 2500.f)
      return true;

    for(const Obstacle& omo : theObstacleModel.obstacles)
    {
      const float distanceToObstacle = (omo.center - theFieldBall.positionRelative).norm();
      const float angleToObstacle = std::abs(omo.center.angle());

      if(distanceToObstacle < (duelMinDistanceToClosest * 1.2f *
            (1.f - (std::abs(angleToObstacle) / Math::pi / 2.f))))   // 改：pi -> Math::pi
        return false;
    }
    return true;
  };

  const bool isLeft = theFieldBall.positionOnField.y() > 0.f;
  const bool tactical = WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair);
  const Vector2f usedGoalPost(
    std::min(theFieldDimensions.xPosOwnGoalPost,
             theFieldBall.positionOnField.x() - theFieldDimensions.goalPostRadius),
    isLeft ? theFieldDimensions.yPosLeftGoal : theFieldDimensions.yPosRightGoal);

  const auto backfield = [&](Angle& relDirection, float& length) -> bool
  {
    using namespace WhiteFleetBackfieldOutlet;
    const int v = variant();
    if(v == off || theGameState.isKickOff() || theGameState.isFreeKick() || theGameState.isPenaltyKick() ||
       theGameState.isGoalkeeper() || !theFieldBall.ballWasSeen(300) || !theRobotPose.isFinite())
      return false;

    Input in;
    in.ball = theFieldBall.positionOnField;
    in.robot = theRobotPose.translation;
    in.halfWidth = theFieldDimensions.yPosLeftTouchline;
    for(const auto& o : theGlobalOpponentsModel.opponents)
      if(o.position.allFinite())
        in.opponents.push_back(o.position);
    for(const auto& o : theObstacleModel.obstacles)
      if(o.isOpponent() && o.center.allFinite())
        in.opponents.push_back(theRobotPose * o.center);
    for(const Teammate& t : theTeamData.teammates)
      if(t.number == 3 && t.number != theGameState.playerNumber &&
         theFrameInfo.getTimeSince(t.theFrameInfo.time) < 1200 && t.theRobotPose.isFinite())
      {
        in.frontKnown = true;
        in.front = t.theRobotPose.translation;
      }

    if(!active(in))
      return false;

    const float dir = direction(in, v, length);
    relDirection = Angle::normalize(dir - theRobotPose.rotation);

    static unsigned lastLog = 0;
    if(SystemCall::getMode() == SystemCall::simulatedRobot &&
       theFrameInfo.getTimeSince(lastLog) >= 1000)
    {
      lastLog = theFrameInfo.time;
      std::printf("BACKFIELD_OUTLET self=%d v=%d ball=%.0f,%.0f dir_deg=%.0f len=%.0f opp_mm=%.0f front=%d\n",
        theGameState.playerNumber, v,
        in.ball.x(), in.ball.y(), dir * 57.29578f, length,
        nearestOpponent(in), int(in.frontKnown));
    }
    return true;
  };

  initial_state(executeSkillRequest)
  {
    transition
    {
      if(theGameState.isKickOff())
        goto kickOff;
      else if(!theGameState.isFreeKick() &&
              (theFieldBall.positionOnField - usedGoalPost).squaredNorm() < Math::sqr(goalPostHandlingAreaRadius))
        goto atOwnGoalPost;
      {
        Angle d = 0_deg;
        float l = 0.f;
        if(activateDuel() && !backfield(d, l))   // a back-field outlet (tested variant) takes precedence over the duel
          goto zweikampf;
      }
    }
    action
    {
      Angle backfieldDirection = 0_deg;
      float backfieldLength = 0.f;
      if(!theSkillRequest.mdfzRule && backfield(backfieldDirection, backfieldLength))
        GoToBallAndKick({.targetDirection = backfieldDirection, .kickType = KickInfo::rlKickStrong,
          .alignPrecisely = KickPrecision::justHitTheBall, .length = backfieldLength});
      else
        switch(theSkillRequest.skill)
        {
          case SkillRequest::pass:
            PassToTeammate({.playerNumber = theSkillRequest.passTarget});
            break;
          case SkillRequest::dribble:
            GoToBallAndDribble({.targetDirection = Angle::normalize(theSkillRequest.target.rotation - theRobotPose.rotation)});
            break;
          case SkillRequest::clear:
            if(theSkillRequest.mdfzRule)
            {
              const Vector2f b = theFieldBall.positionOnField;
              const Vector2f t(std::min(b.x() + 8000.f, theFieldDimensions.xPosOpponentPenaltyMark), b.y() * .5f);
              GoToBallAndKick({.targetDirection = Angle::normalize((t - b).angle() - theRobotPose.rotation),
                .kickType = KickInfo::rlKickStrong, .length = std::min(8000.f, std::max(3000.f, (t - b).norm()))});
            }
            else if(tactical && theGameState.isGoalkeeper() && theGameState.isGoalKick() && theGameState.isForOwnTeam())
            {
              theClearTarget.update();
              if(theClearTarget.isValid && theFieldBall.ballWasSeen(200) &&
                 std::isfinite(theClearTarget.kickLength) && theClearTarget.kickLength >= 500.f &&
                 std::cos(theClearTarget.kickAngle + theRobotPose.rotation) > .15f)
                GoToBallAndKick({.targetDirection = theClearTarget.kickAngle,
                  .kickType = KickInfo::rlKickStrong, .length = std::min(12000.f, theClearTarget.kickLength)});
              else
              {
                LookActive({.withBall = true});
                Stand();
              }
            }
            else if(tactical && theSkillRequest.backfieldStrongOutlet)
            {
              theClearTarget.update();
              if(theClearTarget.isValid && theFieldBall.ballWasSeen(200) &&
                 std::isfinite(theClearTarget.kickLength) && theClearTarget.kickLength >= 4000.f &&
                 std::cos(theClearTarget.kickAngle + theRobotPose.rotation) > .15f)
                GoToBallAndKick({.targetDirection = theClearTarget.kickAngle,
                  .kickType = KickInfo::rlKickStrong, .length = std::min(12000.f, theClearTarget.kickLength)});
              else
                ClearBall(); // Native legal emergency clear only, never a short teammate pass.
            }
            else
              ClearBall();
            break;
          case SkillRequest::shoot:
          default:
          {
            // 每次射门瞄准对方门柱内侧 0.25 m：选离球更近的门柱，向门内收 0.25 m 作为目标点。
            theKickAtGoalEvaluation.update();
            if(theKickAtGoalEvaluation.kickInfo.isValid && theIndirectKick.allowDirectKick)
            {
              const Vector2f leftPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
              const Vector2f rightPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
              const Vector2f& nearPost =
                (leftPost - theFieldBall.positionOnField).squaredNorm() <
                (rightPost - theFieldBall.positionOnField).squaredNorm() ? leftPost : rightPost;
              const Vector2f goalTarget(nearPost.x(), nearPost.y() + (nearPost.y() > 0.f ? -0.25f : 0.25f));
              GoToBallAndKick({.targetDirection = Angle::normalize((goalTarget - theFieldBall.positionOnField).angle() - theRobotPose.rotation),
                               .kickType = theKickAtGoalEvaluation.kickInfo.kickType});
            }
            else
              KickAtGoal();
            break;
          }
        }
    }
  }

  state(atOwnGoalPost)
  {
    transition
    {
      if((theFieldBall.positionOnField - usedGoalPost).squaredNorm() >
         Math::sqr(goalPostHandlingAreaRadius * goalPostHandlingAreaHysteresisMultiplier))
      {
        if(activateDuel())
          goto zweikampf;
        goto executeSkillRequest;
      }
    }
    action
    {
      HandleBallAtOwnGoalPost();
    }
  }

  state(zweikampf)
  {
    transition
    {
      if((theFieldBall.positionOnField - usedGoalPost).squaredNorm() <
         Math::sqr(goalPostHandlingAreaRadius))
        goto atOwnGoalPost;
      {
        Angle d = 0_deg;
        float l = 0.f;
        if(theSkillRequest.mdfzRule || stopDuel() || backfield(d, l))
          goto executeSkillRequest;
      }
    }
    action
    {
      Zweikampf();
    }
  }

  state(kickOff)
  {
    transition
    {
      if(!theGameState.isKickOff())
        goto executeSkillRequest;
    }
    action
    {
      // 平台检查：不再读取环境变量，最后一个参数传 nullptr（等价于“该环境变量未设置”）
      if(WhiteFleetAutonomousKickoff::enabled(WhiteFleetSimPhysical::physical(),
           WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver(), WhiteFleetSimPhysical::simulated(),
           Global::getSettings().teamNumber, nullptr)
         && theGameState.state == GameState::ownKickOff && theGameState.isForOwnTeam())
        DirectKickOff();
      // Class base: a plain football kick-off. The ball goes to the feet of the nearest field player of the own
      // team (the native pass skill; the team-mate receives where it stands, nobody runs ahead). Alone, or with no
      // team-mate known, the native direct kick-off.
      else if(theGameState.isForOwnTeam())
      {
        int mate = -1;
        float nearest = 0.f;
        for(const auto& t : theGlobalTeammatesModel.teammates)
        {
          if(t.playerNumber < Settings::lowestValidPlayerNumber ||
             t.playerNumber > Settings::highestValidPlayerNumber ||
             theGameState.ownTeam.isGoalkeeper(t.playerNumber) ||
             !t.pose.translation.allFinite())
            continue;
          const float d = (t.pose.translation - theFieldBall.positionOnField).norm();
          if(d > 500.f && (mate < 0 || d < nearest))
          {
            mate = t.playerNumber;
            nearest = d;
          }
        }
        if(mate > 0)
          PassToTeammate({.playerNumber = mate});
        else
          DirectKickOff();
      }
      else
        DirectKickOff();
    }
  }
}
