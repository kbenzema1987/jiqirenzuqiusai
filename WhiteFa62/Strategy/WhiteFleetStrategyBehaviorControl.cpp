void WhiteFleetStrategyBehaviorControl::update(SkillRequest& skillRequest)
{
  WhiteFleetSimEntry::activate();
  static thread_local bool fleetProviderSeen=false;
  if(!fleetProviderSeen){WhiteFleetSimEntry::witness("WhiteFleetStrategyBehaviorControl");fleetProviderSeen=true;WhiteFleetSimEntry::tactical(theBehaviorParameters);}
  if(!theGameState.isPlaying() || theGameState.playerState!=GameState::active)
    cornerPlan=CornerPlan{};
  wasPenalized = theExtendedGameState.wasPenalized() && theExtendedGameState.returnFromGameControllerPenalty && (theGameState.isPlaying() || theGameState.isReady() || theGameState.isSet());
  if(wasPenalized)
    wasPenalized = theFrameInfo.getTimeSince(theGameState.timeWhenPlayerStateStarted) < theBehaviorParameters.noSkillRequestAfterUnpenalizedTime;

  auto* self = updateAgents();

  theBehavior.preProcess();

  if(theGameState.playerState != GameState::active || wasPenalized ||
     theGameState.isPenaltyShootout() || theGameState.isInitial() || theGameState.isFinished())
  {
    // Reset provided representations.
    theStrategyStatus.proposedTactic = Tactic::none;
    theStrategyStatus.acceptedTactic = Tactic::none;
    theStrategyStatus.proposedMirror = false;
    theStrategyStatus.acceptedMirror = false;
    theStrategyStatus.proposedSetPlay = SetPlay::none;
    theStrategyStatus.acceptedSetPlay = SetPlay::none;
    theStrategyStatus.setPlayStep = -1;
    theStrategyStatus.position = Tactic::Position::none;
    theStrategyStatus.role = Role::none;
    skillRequest = SkillRequest::Builder::stand();
    outletPlan=OutletPlan{};supportHalf.reset();
    if(theGameState.isInitial()||theGameState.isFinished())
    {kickoffThroughSession.reset();kickoffThroughEvent=0;isolatedKickoffPhase.reset();}
    else if(isolatedKickoffPhase.event)isolatedKickoffPhase.finish("player_inactive_or_reentry");
  }
  else
  {
    ASSERT(self);

    // C15 owns the two field players BEFORE the ordinary role/request update.
    if(runIsolatedKickoff(skillRequest,*self))
    {outletPlan=OutletPlan{};supportHalf.reset();theBehavior.postProcess();return;}

    skillRequest = theBehavior.update(strategy, *self, agents);
    // This playbook has one shared kicker, not two independent local cost
    // elections. Ordinary team roles resume immediately after ownKickOff.
    const bool autonomousKickoff=WhiteFleetAutonomousKickoff::enabled(WhiteFleetSimPhysical::physical(),
        WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver(),WhiteFleetSimPhysical::simulated(),
        Global::getSettings().teamNumber,std::getenv("MDFZ_AUTONOMOUS_KICKOFF_V4_ENABLE"))
        &&theGameState.state==GameState::ownKickOff&&theGameState.isForOwnTeam();
    if(autonomousKickoff&&!theGameState.ownTeam.isGoalkeeper(self->number))
    {
      const bool kicker=WhiteFleetAutonomousKickoff::isKicker(self->number)&&
          theGameState.ownTeam.playerStates[WhiteFleetAutonomousKickoff::kickerNumber-Settings::lowestValidPlayerNumber]==GameState::active;
      self->role=kicker?ActiveRole::toRole(ActiveRole::playBall):Role::none;
      skillRequest=kicker?SkillRequest::Builder::shoot():SkillRequest::Builder::stand();
    }

    theStrategyStatus.proposedTactic = self->proposedTactic;
    theStrategyStatus.acceptedTactic = self->acceptedTactic;
    theStrategyStatus.proposedMirror = self->proposedMirror;
    theStrategyStatus.acceptedMirror = self->acceptedMirror;
    theStrategyStatus.proposedSetPlay = self->proposedSetPlay;
    theStrategyStatus.acceptedSetPlay = self->acceptedSetPlay;
    theStrategyStatus.setPlayStep = self->setPlayStep;
    theStrategyStatus.position = self->position;
    theStrategyStatus.role = self->role;
    if(WhiteFleetDefenseCounterSwitch::enabled()&&!autonomousKickoff)
      applyDefenseCounter(skillRequest,*self);

    // ===== MODIFIED: 对方角球/边线球/门球时进行人盯人防守 =====
    // 条件：
    //   - 不是自主开球
    //   - 当前不是 C15 / restart pass 流程
    //   - 对方发球（!isForOwnTeam()）且是角球、边线球或门球
    //   - 本机器人不是门将、且不是 active role（freeKickWall / playBall 由原逻辑负责）
    // 动作：对 theObstacleModel 中最接近自己的新鲜对手调用 mark。
    // 说明：mark 在 applyMdfzRules 的 case 3 已使用过（SkillRequest::Builder::mark(Vector2f)），签名有依据。
    // 若 reference/ 中 isCornerKick()/isThrowIn() 名称不符，可退化为
    //   (theGameState.isFreeKick() || theGameState.isGoalKick())
    // 会涵盖普通任意球，范围略宽但可编译。
    if(!autonomousKickoff && !skillRequest.kickoffThrough && !skillRequest.restartPassRequired &&
       !theGameState.isForOwnTeam() &&
       (theGameState.isCornerKick() || theGameState.isThrowIn() || theGameState.isGoalKick()) &&
       !self->isGoalkeeper && !Role::isActiveRole(self->role))
    {
      float bestDist = std::numeric_limits<float>::infinity();
      Vector2f bestOpponent = Vector2f::Zero();
      bool found = false;
      for(const auto& obstacle : theObstacleModel.obstacles)
      {
        if(obstacle.isOpponent() && obstacle.lastSeen &&
           obstacle.lastSeen <= theFrameInfo.time &&
           theFrameInfo.time - obstacle.lastSeen <= 1000 &&
           obstacle.center.allFinite())
        {
          const Vector2f p = theRobotPose * obstacle.center;
          if(p.allFinite())
          {
            const float d = (p - theRobotPose.translation).norm();
            if(d < bestDist) { bestDist = d; bestOpponent = p; found = true; }
          }
        }
      }
      if(found)
        skillRequest = SkillRequest::Builder::mark(bestOpponent);
    }
    // ===== MODIFIED END =====

    if(WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff)&&
       (theGameState.state==GameState::setupOwnKickOff||theGameState.state==GameState::waitForOwnKickOff))
      applyKickoffThrough(skillRequest,*self); // Legal READY preparation and SET hold only.
    if(!autonomousKickoff&&!skillRequest.kickoffThrough&&!skillRequest.restartPassRequired)
    {
      {const bool outlet=applyBackfieldOutlet(skillRequest,*self);if(outlet&&mdfzFormation.enabled)++mdfzFormation.n[7];if(!outlet&&!applyMdfzRules(skillRequest,*self)&&!applyMdfzFormation(skillRequest,*self))applyOpponentSidePosition(skillRequest,*self);}
    }
    else {outletPlan=OutletPlan{};supportHalf.reset();}
  }

  if(WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff)&&
     isolatedKickoffPhase.finished)
  {
    skillRequest.isolatedKickoffFinished=true;
    skillRequest.isolatedKickoffEvent=isolatedKickoffPhase.event;
    if(self&&kickoffThroughSession.hasContract&&self->number==kickoffThroughSession.receiver&&
       theGameState.state==GameState::playing)
    {
      skillRequest.kickoffThroughRetired=true;
      skillRequest.kickoffThroughKicker=kickoffThroughSession.sender;
      skillRequest.kickoffThroughReceiver=kickoffThroughSession.receiver;
      skillRequest.kickoffThroughEvent=kickoffThroughSession.event;
      skillRequest.kickoffThroughTarget={kickoffThroughSession.committed.target.x,kickoffThroughSession.committed.target.y};
    }
  }

  theBehavior.postProcess();

  if(theBehaviorParameters.requireFreshTeamActivity && theFrameInfo.getTimeSince(lastTeamActivityDiagnostic) >= 2000)
  {
    lastTeamActivityDiagnostic = theFrameInfo.time;
    int owner = -1;
    for(const Agent& agent : agents)
      if(Role::isActiveRole(agent.role) && agent.role != ActiveRole::toRole(ActiveRole::closestToTeamBall))
        owner = agent.number;
    std::printf("MATCH_STRATEGY pid=%d goalkeeper=%d playing=%d own_setplay=%d agents=%zu owner=%d position=%s role=%s skill=%s ball_age_ms=%d team_ball_valid=%d pose_mm_rad=%.1f,%.1f,%.3f\n",
                theGameState.playerNumber, theGameState.ownTeam.isGoalkeeper(theGameState.playerNumber),
                theGameState.isPlaying(), theGameState.isForOwnTeam(), agents.size(), owner,
                TypeRegistry::getEnumName(theStrategyStatus.position), TypeRegistry::getEnumName(theStrategyStatus.role),
                TypeRegistry::getEnumName(skillRequest.skill), theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen),
                theTeamBallModel.isValid, theRobotPose.translation.x(), theRobotPose.translation.y(), static_cast<float>(theRobotPose.rotation));
  }
}
