/**
 * @file Zweikampf.cpp
 *
 * This file implements a card that duels with an opponent.
 *
 * The general procedure is as follows:
 * - Determine some kick angles based on the last kick direction
 * - For each direction, calculate the field ratings for different kick ranges (TODO only calculate ranges that are actually used for the current direction to safe computation)
 * - For each allowed kick type, determine the best field rating and apply some more ratings and hysteresis, such as:
 *   - avoid kicks that push us closer to an obstacle
 *   - worse rating for kicks with a different direction
 *   - bonus for the same kick type for goals
 * - All kicks are sorted based on their priority: goal > steal (forwardSteal kick, that is our V-Form kick) > pass > everything else
 * - Best kick is selected
 * - Afterwards calculate some precision range, to make the execution easier
 *
 * Future TODOs:
 * - If the behavior says, we want to pass, then ignore possible goal shots and soften the forwardSteal condition.
 * @author Philip Reichenberg
 */

#include "WhiteFleetSkillBehaviorControl.h"
#include "WhiteFleetSimPhysical.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "Framework/ModuleGraphRunner.h"
#include "Platform/SystemCall.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"
#include "MathBase/Approx.h"
#include <cmath>

namespace WhiteFleetZweikampf
{
  STREAMABLE(SearchParameters,
  {,
    (int) numOfAnglesNearBestDuelPose, /**< Number of direction sample for both sides around the last calculated kick direction.*/
    (Angle) rangeOfBestDuelPose, /**< Range for the sampling around the last calculated kick direction.*/
    (int) numOfOverallSearch, /**< Number of direction samples outside of the rangeOfBestDuelPose.*/
    (Angle) overallSearchRange, /**< Search range for additional kick directions.*/
    (Angle) bonusForStealBallDirectionAdjustmentSmall, /**< The checked directions are expanded by the stealBall range plus this angle offset.*/
    (Angle) bonusForStealBallDirectionAdjustmentBig, /**< The checked directions are expanded by the stealBall range plus this angle offset.*/
    (int) moreSearchAfterDoingNothing, /**< After this much time passed without an InWalkKick was possible, some more goal shot angles are added.*/
    (Angle) goalSectorWidth, /**< A goal kick must have at least this much width in the goal sector.*/
    (Rangea) minMaxAngleAngleRange, /**< The allowed robot rotation adjustment for the kick direction is interpolated between this range.*/
    (Angle) goalShotBufferAngle, /**< If possible use this buffer to the goal posts as buffer zone. */
  });

  STREAMABLE(DuelRatings,
  {,
    // rating modifiers
    (float) ratingMinMaxDifference, /**< Add this much malus, the more the robot must rotate for the kick direction. */
    (float) ratingBallLandsInOwnHalf, /**< Add this malus, if the ball would land in the own half. */
    (float) ratingPoseBlocked, /**< Add this malus, if the kick pose is blocked by an obstacle. */
    (float) ratingPassingToOpponent, /**< Add this malus the closer the ball is passed towards an opponent. */

    (float) ratingSameKick, /**< Add this bonus, if the checked kick is the same as the last best kick. */
    (float) ratingSameKickAngle, /**< Add this bonus, if the checked kick is the same as the last best kick and the direction is similar. */
    (float) ratingStealBall, /**< removes ratingBallLandsInOwnHalf and adds some bonus too. */
    (float) ratingStealBallKick, /**< Add this bonus for steal ball kicks.*/
    (float) ratingStealBallKickBetterSide, /**< Add this bonus for the forwardSteal, to get the better side.*/
    (float) ratingGoalShot, /**< Add this bonus if the kick can reach the goal.*/
    (float) ratingGoalShotBuffer, /**< Add this bonus if there is some buffer range to the goal post or obstacles. */
  });

  STREAMABLE(DuelTimings,
  {,
    (int) maxTimeDoingNothing, /**< After this much time passed without an InWalkKick was possible, the Zweikampf deactivates itself.*/
    (int) inactiveAfterDoingNothing, /**< After the Zweikampf deactivated itself because of doing nothing, this much time must pass before going active again.*/
    (int) sidewardRestrictionTime, /**< After the sideward kick was restricted once, so much time must have passed after no restriction is needed anymore, before the restriction is actually disabled again.*/
    (int) noKickMinTime, /**< If the last planned kick was not the forwardSteal, so much time must have passed of no calculated kicks, before the forwardSteal is no longer wanted.*/
    (int) noKickStealMinTime, /**< If the last planned kick was the forwardSteal, so much time must have passed of no calculated kicks, before the forwardSteal is no longer wanted.*/
    (int) ignoreSkillRequestTime, /**< Ignore the pass skill request for this long, if we decided we can not execute it. */
  });

  STREAMABLE(ObstacleHandling,
  {,
    (Vector2f) leftFootEdge, /**< Left foot edge.*/
    (Vector2f) rightFootEdge, /**< Right foot edge.*/
    (Vector2f) leftFootKickEdge, /**< Left foot edge after the InWalkKick.*/
    (Vector2f) rightFootKickEdge, /**< Right foot edge after the InWalkKick.*/
    (float) maxObstacleDistanceForWalkStealBallKick, /**< The obstacle must stand this much more away from the ball than us, to block the forwardSteal InWalkKick.*/
    (float) maxObstacleDistanceForSideStealBallKick, /**< The obstacle must stand this much more away from the ball than us, to block the forwardSteal InWalkKick.*/
    (float) obstacleDistanceForBlockingPose, /**< Calculate where the obstacle could be, when we execute the kick. This is used to decide, if one kick is worse than the mirror one. */
    (float) noKickBlockingPose, /**< distance to ball, when no kick was calculated. */
    (float) obstacleMinDistanceForBackPass, /**< The duel obstacle must be this far away to allow for a back pass from the skill request. */
    (Rangef) obstacleShiftRange, /**< Obstacles, that are this near the ball, are shifted behind the ball.*/
    (float) maxObstacleDistanceToBallForRiskyKicks,
    (float) maxObstacleDistanceToForceForwardSteal,
    (float) obstacleDistanceThreshold, /**< Towards obstacles, we want to keep a safe range to prevent passing to an opponent. Scale  */
    (float) goalSectorShiftFactor, /**< For goal shots, shift direction by this factor times sector width towards center. */
  });

  STREAMABLE(StealBallParameters,
  {,
    (Angle) stealBallTypeTooMuchRotationMinValue, /**< StealBall kick types are allowed to have higher needed rotation. */
    (Angle) bonusForStealBallMinMaxAngle,
    (Rangea) stealBallRange, /**< Field directions get a bonus, if they are in this range.*/ // TODO After Simulation Evaluation set max angle to 90 deg
    (Angle) stealBallMinRobotRotation, /**< The own rotation must be above this value, to give a bonus to directions that kick the ball sideways on the field and not in direction of the goal.*/
    (Angle) stealBallMinPositionRotation, /**< The ball position angle on the field must be above this value, to give a bonus to directions that kick the ball sideways on the field and not in direction of the goal.*/
    (Rangea) forwardStealBallOpponentPositionNormal, /**< The obstacle must have this angle relative to the ball, to allow the forwardSteal InWalkKick.*/
    (Angle) forwardStealBallOpponentPositionMalus, /**< The obstacle must have this angle relative to the ball, to allow the forwardSteal InWalkKick, if the last planned InWalkKick was no forwardSteal.*/
  });

  STREAMABLE(LastKickHysteresis,
  {,
    (float) kickLengthHysteresis, /**< Max kick length hysteresis */
    (Angle) kickDirectionHysteresis, /**< Interpolate max kick length hysteresis only if the direction is similar to previous kick. */
  });

  STREAMABLE(DistanceParameters,
  {,
    (KickInfo::KickType) mainKick,
    (int) numberOfSubRanges,
    (std::vector<float>) additionalRanges,
  });

  STREAMABLE(StealKickReplacementParameter,
  {,
    (Angle) staticShift, /**< Static direction shift for steal kick replacement. */
    (Rangef) distanceToFieldBorderRange, /**< Reduce kick strength based on field border distance. */
    (Rangef) interpolateDirectionGoalDistance, /**< Interpolate replacement kick direction towards opponent goal, closer to the goal. */
  });
  /**
   * The different types of kicks. Higher ups, like goal shots, are always better than lower types, even if the rating is worse.
   * That way, a goal kick will always be done as long as it is possible.
   */
  ENUM(TargetType,
  {,
    goalShot,
    goalDribbleShot,
    stealBall,
    pass,
    other,
  });

  struct DuelPose
  {
    float rating = 0.f; /**< Rating of the kick. Small == good, high == bad. */
    Angle kickAngle = 0_deg; /**< Kick direction. */
    Pose2f pose; /**< Kick pose. */
    bool noKick = true; /**< Is a kick requested? */
    bool preStepAllowed = true; /**< Is a pre step allowed for the kick? */
    KickInfo::KickType kickType = KickInfo::rlKickDynamic; /**< Type of the kick. */
    Rangea precision = Rangea(0_deg, 0_deg); /**< Precision range of the kick direction. */
    float range = -1.f; /**< Kick range. Used to calculate the kick power. */
    TargetType type = TargetType::other; /**< Type of the situation, in which the kick is used. */
    Vector2f lastFieldEndPoint; /**< Ball end position after the kick. */
  };

  /**
   * Local copy of theSkillRequest
   */
  struct SkillRequestPose
  {
    Pose2f target;
    Rangea passRangeOne;
    Rangea passRangeTwo;
    int passTarget = -1;
    int ignoredPassTarget = -1;
    SkillRequest::Type skill = SkillRequest::none;
  };

  /**
   * Mapping of every kick range(that should be checked) to a rating.
   */
  struct RatingMap
  {
    std::optional<float> rating; /**< Rating of the range. */
    float range = 0.f; /**< The kick range. */
    Vector2f fieldEndPoint; /**< Global field coordinate. */
    bool isTeammatePass = false; /**< Is the field coordinate considered as a pass? */
    bool isTypeSteal = false; /**< Is the field coordinate considered as a steal of the ball? */

    float ratingFunc(const FieldRating& theFieldRating, const int passTarget, const Vector2f& useBallPosition,
                     const bool isTypeSteal, const Vector2f& direction, const float rangeForGoal);
  };

  /**
   * Calculate the rating of a given kick
   *
   * @param theFieldRating the rating functions
   * @param passTarget The pass teammate from the behavior. -1 if none is planned.
   * @param useBallPosition The ball in field coordinates
   * @param isTypeSteal Is the kick a forwardSteal kick? In that case, ignore some ratings
   * @param direction Direction and length of the kick
   * @param rangeForGoal The range to reach the goal
   */
  float RatingMap::ratingFunc(const FieldRating& theFieldRating, const int passTarget, const Vector2f& useBallPosition,
                              const bool isTypeSteal, const Vector2f& direction, const float rangeForGoal)
  {
    if(this->rating.has_value())
      return this->rating.value();

    this->fieldEndPoint = useBallPosition + direction * std::min(range, rangeForGoal);
    // normal rating
    bool isTeammatePass = false;
    PotentialValue teammate;
    PotentialValue pvBallNear;

    // get most ratings, with and without obstacle potential
    PotentialValue ratingField = theFieldRating.potentialFieldOnly(this->fieldEndPoint.x(), this->fieldEndPoint.y(), false);
    if(!isTypeSteal)
    {
      theFieldRating.getObstaclePotential(ratingField, this->fieldEndPoint.x(), this->fieldEndPoint.y(), false);
      theFieldRating.duelBallNearPotential(pvBallNear, this->fieldEndPoint.x(), this->fieldEndPoint.y(), false);
    }
    theFieldRating.potentialOverall(teammate, this->fieldEndPoint.x(), this->fieldEndPoint.y(), isTeammatePass, false, passTarget);
    theFieldRating.removeBallNearFromTeammatePotential(teammate, pvBallNear);
    ratingField += teammate;
    ratingField += pvBallNear; // add ball near potential after the teammate's potential

    this->rating = ratingField.value;
    this->isTeammatePass = isTeammatePass;

    return this->rating.value();
  }

  /**
   * The struct that contains the information of the ratings on the field.
   * It is used to calculate the ratings on demand, to reduce computation time.
   */
  struct RatingMapVector
  {
    std::vector<RatingMap> ratingMap;
    const bool isTypeSteal;
    const Vector2f direction;
    const float rangeForGoal;
    const int passTarget;
    const Vector2f useBallPosition;

    RatingMapVector(const bool isTypeSteal, const Vector2f& direction, const float rangeForGoal, const int passTarget, const Vector2f& useBallPosition) :
      isTypeSteal(isTypeSteal), direction(direction), rangeForGoal(rangeForGoal), passTarget(passTarget), useBallPosition(useBallPosition) {}

    float ratingFunc(const FieldRating& theFieldRating, RatingMap& ratingMap);
  };

  /**
   * Interface to get the ratings
   */
  float RatingMapVector::ratingFunc(const FieldRating& theFieldRating, RatingMap& ratingMap)
  {
    return ratingMap.ratingFunc(
             theFieldRating, this->passTarget, this->useBallPosition, this->isTypeSteal, this->direction, this->rangeForGoal);
  }

  /**
   * Definition of a sector with an obstacle in it. Used for the sector wheel.
   */
  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };
}
using namespace WhiteFleetZweikampf;

option((WhiteFleetSkillBehaviorControl) Zweikampf,
       load((SearchParameters) searchParameters, /**< Parameters to decide which kick angles are checked. */
            (DuelRatings) duelRatings, /**< All rating modifiers. */
            (DuelTimings) duelTimings, /**< All time stamp parameters. */
            (ObstacleHandling) obstacleHandling, /**< All parameters to handle obstacles. */
            (StealBallParameters) stealBallParameters, /**< All parameters to handle kick angles near obstacles. */
            (LastKickHysteresis) lastKickHysteresis, /**< Hysteresis parameters too make sure the last decision is kept. */
            (DistanceParameters) distanceParameters, /**< Parameters to generate list of ranges to check. */
            (StealKickReplacementParameter) stealKickReplacementParameter, /**< Parameters for replacing the steal kick. */

            (Angle) smallerKickAnglePreference, /**< Interpolate ratingLargeKickAngle to 100% over this angle range. */
            (Angle) forwardStealPreferenceRange, /**< If the kick angle for the forward steal is bigger or smaller than this value, we can be sure to use the better side. */
            (float) sectorWheelObstacleBallRadiusFactor, /**< Increase obstacle with by this factor times the ball radius.*/
            (Rangea) allowPassAngle, /**< Pass request from the skill request are allowed in this field angle range. */
            (Angle) passOrientation, /**< Pass request from the skill request are allowed if the robot is orientated to the target already good enough. */
            (float) fieldBorderSafeDistance, /**< InWalkKick have some uncertainty, so the safe distance to the field border intersection is increased by up to this value.*/

            (Angle) sidewardsRotationCorrectionBonus, /**< If last planned kick was the sidewards kick, then add a bonus to the allowed rotation. */

            (Rangef) ballPositionInterpolationRange, /**< If the ball is close, interpolate the perceived and intercepted position based on this ball distance ranges. */

            // look active
            (float) lookActiveMinBallDistance, /**< If the ball is at least this far away, use lookActive with withBall = true. */

            (bool) replaceForwardWithLongGoalShot, /**< If kicking at the goal, replace this forward kick with the long version. */

            (std::vector<KickInfo::KickType>) allowedKicks, /**< The allowed InWalkKicks */
            (Rangef) minKickRange), /**< Use min kick range based on field position. */
       vars((DuelPose) theDuelPose, /**< All information about the "to be executed" kick */
            (Angle)(0_deg) lastKickAngle, /**< Last kick angle with odometry update */
            (Angle)(0_deg) lastRobotPoseRotation, /**< Last RobotPose rotation */
            (SkillRequestPose) theSkillRequestPose, /**< All information about the current behavior request */
            (unsigned)(0) timeSinceDoingNothing, /**< timestamp since no kick was calculated */
            (unsigned)(0) lastForwardSteal, /**< timestamp since last forwardSteal was the best kick */
            (unsigned)(0) lastSidewardRestrictionTimestamp, /**< Last timestamp, when sidewards kick was restricted */
            (unsigned)(0) ignoreSkillRequestTimestamp, /**< Time stamp when skill request started to be ignored */
            (std::array<unsigned, TargetType::numOfTargetTypes>)({0, 0, 0, 0, 0}) kickForcedUpTime, /**< Timestamps of the execution of the different kick types. */
            (std::vector<float>) checkKickDistancesForFR, /**< All kick ranges, that shall be checked */
            (bool)(false) forcedInactive, /**< Deactivate the Zweikampf? */
            (bool)(false) forceForwardSteal, /**< Was the forwardSteal force because the obstacle is actually close to the ball? */
            (bool)(false) shouldDribble, /**< Zweikampf can not handle kicks right now. Instead dribble for a moment. */
            (unsigned)(0) lastDribbleRequest)) /**< Should we first dribble to get in goal range? */
{
  const Vector2f leftGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal); // The position of the left post of the opponent's goal.
  const Vector2f rightGoalPost(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal); // The position of the right post of the opponent's goal.
  const Vector2f leftGoalPostOwn(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal); // The position of the left post of the own goal.
  const Vector2f rightGoalPostOwn(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal); // The position of the right post of the own goal.
  const Obstacle opponentLeftGoalPost(Matrix2f::Zero(), leftGoalPost, leftGoalPost + Vector2f(0.f, theFieldDimensions.goalPostRadius), leftGoalPost - Vector2f(0.f, theFieldDimensions.goalPostRadius), 0, Obstacle::Type::unknown);
  const Obstacle opponentRightGoalPost(Matrix2f::Zero(), rightGoalPost, rightGoalPost + Vector2f(0.f, theFieldDimensions.goalPostRadius), rightGoalPost - Vector2f(0.f, theFieldDimensions.goalPostRadius), 0, Obstacle::Type::unknown);

  const Vector2f penaltyBoxBackRight(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightPenaltyArea); // Back right corner of own penalty box.
  const Vector2f penaltyBoxFrontLeft(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea); // Front left corner of own penalty box.
  const Vector2f opponentHalfBackRight(0.f, theFieldDimensions.yPosRightTouchline); // Back right corner of opponent half.
  const Vector2f opponentHalfFrontLeft(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline); // Front left corner of opponent half.
  const Geometry::Rect opponentHalf(opponentHalfBackRight, opponentHalfFrontLeft); // Rectangle of opponent half.
  const Geometry::Rect opponentGoalArea(Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea - 100.f), Vector2f(theFieldDimensions.xPosOpponentGoalLine + 200.f, theFieldDimensions.yPosLeftGoalArea + 100.f)); // Rectangle of opponent goal area.
  const Geometry::Rect penaltyRect(penaltyBoxBackRight, penaltyBoxFrontLeft); // Rectangle of own penalty area.
  const Geometry::Rect playingField(-opponentHalfFrontLeft, opponentHalfFrontLeft);
  const Rangea boundaryCheck(-180_deg, 180_deg); // Min and max angles.
  const Rangef ballClipX(theFieldDimensions.xPosOwnGoalLine + 10.f, theFieldDimensions.xPosOpponentGoalLine - 10.f);
  const Rangef ballClipY(theFieldDimensions.yPosRightTouchline + 10.f, theFieldDimensions.yPosLeftTouchline - 10.f);
  const float baseHighestDistance = sqr(theFieldDimensions.xPosOpponentGoal * 2.f); // Just a large number as an upper bound

  // Variables written and read by different lambdas
  bool interpolateBallVelocity = true;
  Obstacle duelObstacle; // Obstacle, with which we are fighting for the ball
  Obstacle duelObstacleMovedNearBall; // duelObstacle, but shifted based on the ball position
  float distanceToClosestObstacle; // distance to the duelObstacle
  std::list<SectorWheel::Sector> kickAngles; // SectorWheel
  std::vector<Rangea> goalAngleWithBuffer; // Goal sectors with more buffer to the goal posts and other obstacles
  Vector2f useBallPositionInField = Vector2f(0.f, 0.f); // Clipped Ballposition in field coordinates
  Vector2f useBallPositionRelative = Vector2f(0.f, 0.f); // Ballposition in relative coordinates
  bool soonCloseRangeDuel = false; // Opponent is dangerous near to the ball. Start using risky kicks

  /**
   * Update the kick angle based on the robot pose change compared to the previous execution frame
   */
  const auto updateDuelPose = [&]
  {
    // Calculate kickAngle based in the planned end point
    lastKickAngle = Angle::normalize(theDuelPose.kickAngle - theRobotPose.rotation + lastRobotPoseRotation);
    lastRobotPoseRotation = theRobotPose.rotation;
    theDuelPose.kickAngle = (theDuelPose.lastFieldEndPoint - useBallPositionInField).angle() - theRobotPose.rotation;
    if(std::isnan(theDuelPose.kickAngle))
      theDuelPose.kickAngle = 0_deg;
  };

  /**
   * The behavior plans a pass to a teammate. Set up some variables
   */
  const auto updatePassEndPosition = [&]
  {
    const GlobalTeammatesModel::TeammateEstimate* teammate = nullptr;
    for(const auto& t : theGlobalTeammatesModel.teammates)
    {
      if(t.playerNumber == theSkillRequestPose.passTarget)
        teammate = &t;
    }
    if(!teammate)
      theSkillRequestPose.passTarget = -1;
    else
    {
      theSkillRequestPose.target = teammate->pose;

      // In case the kick angles are restricted, e.g. for back passes, pre compute kick angles ranges which are allowed for only passes
      const Angle relativeAngle = Angle::normalize((theSkillRequestPose.target.translation - useBallPositionInField).angle() - theRobotPose.rotation);
      const Rangea range(Angle::normalize(relativeAngle - passOrientation), Angle::normalize(relativeAngle + passOrientation));
      if(range.min > range.max)
      {
        theSkillRequestPose.passRangeOne = Rangea(range.min, 180_deg);
        theSkillRequestPose.passRangeTwo = Rangea(-180_deg, range.max);
      }
      else
      {
        theSkillRequestPose.passRangeOne = range;
        theSkillRequestPose.passRangeTwo = range;
      }
    }
  };

  // TODO Refactor Method, updatePassEndPosition() is kinda useless and code is hard to read, because it is unnecessary complicated
  // If skill request changes and it is a pass, we just check if the target exists to verify a possible target
  // If dribbling, take the target direction as a base for the search later in the Zweikampf
  // In general: Just update the last direction based on the odometry
  const auto updateSkillRequest = [&]
  {
    if((theFrameInfo.getTimeSince(ignoreSkillRequestTimestamp) > duelTimings.ignoreSkillRequestTime || theSkillRequest.skill != SkillRequest::pass || // Ignore bad to be executed passes
        (theSkillRequestPose.ignoredPassTarget != -1 && theSkillRequestPose.ignoredPassTarget != theSkillRequest.passTarget)) && // pass target changed
       (theSkillRequest.skill != theSkillRequestPose.skill ||
        theSkillRequest.passTarget != theSkillRequestPose.passTarget))
    {
      theSkillRequestPose.passTarget = theSkillRequest.skill == SkillRequest::pass ? theSkillRequest.passTarget : -1;
      theSkillRequestPose.skill = theSkillRequest.skill;
      theSkillRequestPose.target = theSkillRequest.target;
      switch(theSkillRequest.skill)
      {
        case SkillRequest::pass:
        {
          for(const auto& t : theGlobalTeammatesModel.teammates)
          {
            if(t.playerNumber == theSkillRequestPose.passTarget)
            {
              updatePassEndPosition();
              goto teammateFound;
            }
          }
          theSkillRequestPose.passTarget = -1; // Nothing more needed here. Just use the last duelPose as a base
        teammateFound:
          break;
        }
        case SkillRequest::dribble:
          theDuelPose.kickAngle = theSkillRequestPose.target.rotation - theRobotPose.rotation;

        default:
          break;
      }
    }
    else if(theSkillRequestPose.skill == SkillRequest::pass)
      updatePassEndPosition();

    if(theSkillRequestPose.passTarget == -1 && theSkillRequestPose.skill == SkillRequest::pass)
      theSkillRequestPose.skill = SkillRequest::shoot;

    updateDuelPose();
  };

  /**
   * Obstacles that are close to the ball shall be changed and moved slightly away.
   * Otherwise an obstacle can be in front of the ball by a few mm. This is an error in the perception which is corrected here.
   */
  const auto shiftObstacleBackward = [&](Vector2f& obstacleCenter, Vector2f& duelObstacleMovedNearBall)
  {
    // Currently, obstacles that are really close to the ball, but not in front of it, are perceived as being in front of the ball by a few cm.
    // We need to shift them backwards behind the ball, otherwise the sector wheel for the duel calculation will just ignore this obstacle.
    // This would just result in a forward kick -> ball bounces off the opponent and we just walk on top of the opponent feet and fall over.
    const float obstacleDistanceToBall = (obstacleCenter - useBallPositionRelative).norm();
    if(theFieldBall.timeSinceBallWasSeen < 1500.f && obstacleDistanceToBall <= std::abs(obstacleHandling.obstacleShiftRange.min) && obstacleCenter.x() < useBallPositionRelative.x() + obstacleHandling.obstacleShiftRange.max && obstacleCenter.x() > theFieldBall.positionRelative.x() + obstacleHandling.obstacleShiftRange.min)
    {
      const Angle rotationAngle = (Vector2f(theBallSpecification.radius, obstacleCenter.y() - useBallPositionRelative.y())).angle();
      obstacleCenter = Vector2f(std::max(std::abs(obstacleCenter.y() - useBallPositionRelative.y()), theBallSpecification.radius), 0.f).rotated(rotationAngle) + useBallPositionRelative;
    }
    if(duelObstacleMovedNearBall != Vector2f(1.f, 0.f) && (theRobotPose * obstacleCenter - useBallPositionInField).x() > 0.f)
      duelObstacleMovedNearBall = (duelObstacleMovedNearBall - useBallPositionRelative).normalized(obstacleHandling.obstacleDistanceForBlockingPose) + useBallPositionRelative;
  };

  /**
   * Find the obstacle which is our dueling obstacle. Important for some decisions for the forwardSteal kick.
   */
  const auto calculateClosestObstacle = [&](const ObstacleModel& model, Obstacle& nearestObstacle, Obstacle& duelObstacleMovedNearBall)
  {
    distanceToClosestObstacle = baseHighestDistance;
    duelObstacle.center = Vector2f(baseHighestDistance, 0.f);

    for(const Obstacle& omo : model.obstacles)
    {
      const float sqrNorm = (omo.center - theFieldBall.positionRelative).squaredNorm();
      if(sqrNorm < distanceToClosestObstacle)
      {
        distanceToClosestObstacle = sqrNorm;
        nearestObstacle = omo;
      }
    }
    distanceToClosestObstacle = std::sqrt(distanceToClosestObstacle);
    duelObstacleMovedNearBall = nearestObstacle;
    shiftObstacleBackward(nearestObstacle.center, duelObstacleMovedNearBall.center);
  };

  /**
   * Decide whether we should ignore the request of the behavior
   */
  const auto handleIgnoreSkillRequest = [&](const Vector2f& opponentOnField)
  {
    if(theSkillRequestPose.skill == SkillRequest::pass)
    {
      const Angle fieldAngle = (theSkillRequestPose.target.translation - useBallPositionInField).angle();
      if(!allowPassAngle.isInside(fieldAngle) &&
         (opponentOnField - useBallPositionInField).squaredNorm() < sqr(obstacleHandling.obstacleMinDistanceForBackPass) &&
         std::abs(fieldAngle - theRobotPose.rotation) > passOrientation) // the robot must be aligned good enough for the pass already
      {
        ignoreSkillRequestTimestamp = theFrameInfo.time;
        theSkillRequestPose.skill = SkillRequest::shoot;
        theSkillRequestPose.ignoredPassTarget = theSkillRequestPose.passTarget;
        theSkillRequestPose.passTarget = -1;
      }
    }
  };

  /**
   * Calculate the obstacle and goal sectors
   */
  const auto calculateSectorWheel = [&]
  {
    Vector2f dummyOB(1.f, 0.f);

    const auto calcObstacleInSector = [&](const Obstacle& obstacle, ObstacleSector& dummyObstacleSector)
    {
      Vector2f useObstacleCenter = obstacle.center;
      bool filterSector = false;
      shiftObstacleBackward(useObstacleCenter, dummyOB);
      filterSector = useObstacleCenter != obstacle.center;

      if(!filterSector && useObstacleCenter == duelObstacle.center)
      {
        Vector2f disToBall = useObstacleCenter - useBallPositionRelative;
        const float normSquared = disToBall.squaredNorm();
        const float ownDistanceToBallSquared = useBallPositionRelative.squaredNorm();
        if(normSquared > sqr(300.f))
          useObstacleCenter = useBallPositionRelative + disToBall / (std::sqrt(normSquared)) * std::max(300.f, std::sqrt(normSquared - ownDistanceToBallSquared));
      }

      // clip obstacle behind the ball
      const Vector2f obstacleOnField = theRobotPose * useObstacleCenter;
      if(obstacleOnField.x() > (theFieldDimensions.xPosOpponentGoalLine + theFieldDimensions.xPosOpponentGoal) * 0.5f)
        return false;

      const float widthScale = 1.f - Rangef::ZeroOneRange().limit((theFrameInfo.getTimeSince(obstacle.lastSeen) - 1000) / 4000.f);
      const float width = widthScale * ((obstacle.left - obstacle.right).norm() + sectorWheelObstacleBallRadiusFactor * theBallSpecification.radius);
      const float distance = (obstacleOnField - useBallPositionInField).norm();

      // TODO make it less ugly
      // In case the obstacle is "inside" the ball, the sector needs to be filtered. otherwise the possible kick angles change heavily every frame...
      const float radius = std::atan(width / (2.f * distance));
      const Angle direction = (obstacleOnField - useBallPositionInField).angle();
      Rangea obstacleSectorRange = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
      if(filterSector &&
         ((std::abs(obstacleSectorRange.min) > 90_deg && std::abs(obstacleSectorRange.max) < 90_deg) ||
          (std::abs(obst
