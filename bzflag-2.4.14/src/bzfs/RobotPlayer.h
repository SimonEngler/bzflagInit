/* bzflag
 * Copyright (c) 1993-2018 Tim Riker
 *
 * This package is free software;  you can redistribute it and/or
 * modify it under the terms of the license found in the file
 * named COPYING that should have accompanied this file.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *
 */

#ifndef	BZF_ROBOT_PLAYER_H
#define	BZF_ROBOT_PLAYER_H

#include "common.h"

/* system interface headers */
#include <vector>

/* interface header */
#include "LocalPlayer.h"

/* local interface headers */
#include "Region.h"
#include "RegionPriorityQueue.h"
#include "ServerLink.h"

#include "ControlPanel.h"
/* lines added by David Chin */
#include "AStarNode.h" // needed for A* search
/* end of lines added by David Chin */

class RobotPlayer : public LocalPlayer {
  public:
			RobotPlayer(const PlayerId&,
				const char* name, ServerLink*,
				const char* _motto);

    float		getTargetPriority(const Player*) const;
    const Player*	getTarget() const;
    void		setTarget(const Player*);
    static void		setObstacleList(std::vector<BzfRegion*>*);

    void		restart(const float* pos, float azimuth);
    void		explodeTank();
/* code added by David Chin */
	bool		RobotPlayer::amAlive(float dt);
	bool		returnTrue(float dt);
	bool		returnFalse(float dt);
	void		RobotPlayer::a1(float dt);
	void		RobotPlayer::a2(float dt);
	void		RobotPlayer::a3(float dt);
	void		RobotPlayer::a4(float dt);
	void		RobotPlayer::a5(float dt);

        /* lines added by Brendt McFeeley */

        bool nonIsExpired(float dt);
        bool firingReady(float dt);
        bool shotTimerElapsed(float dt);
        bool shotMissHalfLength(float dt);
        bool buildingsInWay(float dt);
        bool teamMatesInWay(float dt);
        bool botHoldingFlag(float dt);
        bool holdingFlagSticky(float dt);
        bool isFlagTeamFlag(float dt);
        bool isFlagMyTeam(float dt);
        void		RobotPlayer::b1(float dt);
        void		RobotPlayer::b2(float dt);
        void		RobotPlayer::b3(float dt);
        void		RobotPlayer::b4(float dt);
        void		RobotPlayer::b5(float dt);

        void		RobotPlayer::c1(float dt);
        void		RobotPlayer::c2(float dt);
        void		RobotPlayer::c3(float dt);
        void		RobotPlayer::c4(float dt);
        void		RobotPlayer::c5(float dt);

        /* end lines added by Brendt McFeeley*/

/* end of code added by David Chin */

  private:
    void		doUpdate(float dt);
    void		doUpdateMotion(float dt);
    BzfRegion*		findRegion(const float p[2], float nearest[2]) const;
    float		getRegionExitPoint(
				const float p1[2], const float p2[2],
				const float a[2], const float targetPoint[2],
				float mid[2], float& priority);
     void		findPath(RegionPriorityQueue& queue,
				BzfRegion* region, BzfRegion* targetRegion,
				const float targetPoint[2], int mailbox);

     void		projectPosition(const Player *targ,const float t,float &x,float &y,float &z) const;
     void		getProjectedPosition(const Player *targ, float *projpos) const;
/* lines added by David Chin */
     void		findHomeBase(TeamColor teamColor, float location[3]);
     bool		myTeamHoldingOpponentFlag(void);
     void		findOpponentFlag(float location[3]);
     int		computeCenterOfMass(float neighborhoodSize, float cmOut[3]);
     int		computeRepulsion(float neighborhoodSize, float repulseOut[3]);
     int		computeAlign(float neighborhoodSize, float avVOut[3], float* avAzimuthOut);
     float 		computeWander(void);
     Player* 		lookupLocalPlayer(PlayerId id);
     void		aStarSearch(const float startPos[3], const float goalPos[3],
										 std::vector< std::vector< AStarNode > >& paths);

     static const float		CohesionW;
     static const float		SeparationW;
     static const float		AlignW;
     static const float		PathW;
private:
    static float		wanderAzimuth;
    static TimeKeeper   tick;
    std::vector< std::vector< AStarNode > > paths; // planner result paths
    AStarNode pathGoalNode;	// goal position for current planner result
/* end of lines added by David Chin */
    const Player*	target;
    std::vector<RegionPoint>	path;
    int			pathIndex;
    float		timerForShot;
    bool		drivingForward;
    static std::vector<BzfRegion*>* obstacleList;
};

#endif // BZF_ROBOT_PLAYER_H

// Local Variables: ***
// mode: C++ ***
// tab-width: 8 ***
// c-basic-offset: 2 ***
// indent-tabs-mode: t ***
// End: ***
// ex: shiftwidth=2 tabstop=8
