/*
 * Defines the classes used for decision trees.
 *
 * Part of the Artificial Intelligence for Games system.
 *
 * Copyright (c) Ian Millington 2003-2006. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */
#include "dectree.h"

namespace aicore
{

    DecisionTreeNode* Decision::makeDecision(RobotPlayer* bot, float dt)
    {
        // Choose a branch based on the getBranch method
        if (getBranch(bot, dt)) {
            // Make sure its not null before recursing.
            if (trueBranch == NULL) return NULL;
            else return trueBranch->makeDecision(bot, dt);
        } else {
            // Make sure its not null before recursing.
            if (falseBranch == NULL) return NULL;
            else return falseBranch->makeDecision(bot, dt);
        }
    }
    
	DecisionTreeNode* DecisionPtr::makeDecision(RobotPlayer* bot, float dt)
    {
        // Choose a branch based on the getBranch method
        if ( getBranch(bot, dt) ) {
            // Make sure its not null before recursing.
            if (trueBranch == NULL) return NULL;
            else return trueBranch->makeDecision(bot, dt);
        } else {
            // Make sure its not null before recursing.
            if (falseBranch == NULL) return NULL;
            else return falseBranch->makeDecision(bot, dt);
        }
    }

	bool DecisionPtr::getBranch(RobotPlayer* bot, float dt)
	{
		return (bot->*decFuncPtr)(dt);
	}

        /* code modified by Mariah Gaoiran */

	// Set up the trees
	void DecisionTrees::init()
	{
                // doUpdateMotion
                doUpdateMotionDecisions[0].decFuncPtr = &RobotPlayer::amAlive;          // is the robot alive?
                doUpdateMotionDecisions[0].trueBranch = &doUpdateMotionDecisions[1];      // yes: Q1
                doUpdateMotionDecisions[0].falseBranch = &doUpdateActions[0];             // no: A0: do nothing

                doUpdateMotionDecisions[1].decFuncPtr = &RobotPlayer::shotHitBot;       //non-invisible, non-isExpired shot about to hit bot?
                doUpdateMotionDecisions[1].trueBranch = &doUpdateActions[1];              // yes: A1: evade
                doUpdateMotionDecisions[1].falseBranch = &doUpdateActions[2];             // no: A2: A* search

                // doUpdate (Shot)
                doUpdateShotDecisions[0].decFuncPtr = &RobotPlayer::amAlive;              // is the robot alive?
                doUpdateShotDecisions[0].trueBranch = &doUpdateShotDecisions[1];            // yes: Q1
                doUpdateShotDecisions[0].falseBranch = &doUpdateActions[0];                 // no: A0: do nothing

                doUpdateShotDecisions[1].decFuncPtr = &RobotPlayer::firingStatusReady;    // is firing status ready?
                doUpdateShotDecisions[1].trueBranch = &doUpdateShotDecisions[2];            // yes: Q2
                doUpdateShotDecisions[1].falseBranch = &doUpdateActions[0];                 // no: A0: do nothing

                doUpdateShotDecisions[2].decFuncPtr = &RobotPlayer::shotTimerElapsed;     // has shot timer elapsed?
                doUpdateShotDecisions[2].trueBranch = &doUpdateShotDecisions[3];           // yes: Q3
                doUpdateShotDecisions[2].falseBranch = &doUpdateActions[0];                // no: A0: do nothing

                doUpdateShotDecisions[3].decFuncPtr = &RobotPlayer::shotMissTarget;       // will shot miss target by < 1/2 tank length
                doUpdateShotDecisions[3].trueBranch = &doUpdateShotDecisions[4];           // yes: Q4
                doUpdateShotDecisions[3].falseBranch = &doUpdateActions[0];                // no: A0: do nothing

                doUpdateShotDecisions[4].decFuncPtr = &RobotPlayer::buildingsInWay;       // any buildings in the way?
                doUpdateShotDecisions[4].trueBranch = &doUpdateActions[0];                 // yes: A0: do nothing
                doUpdateShotDecisions[4].falseBranch = &doUpdateShotDecisions[5];          // no: Q5

                doUpdateShotDecisions[5].decFuncPtr = &RobotPlayer::teammateInWay;        // any teammates in the way?
                doUpdateShotDecisions[5].trueBranch = &doUpdateActions[3];                 // yes: A3: set shot timer to 0.1f
                doUpdateShotDecisions[5].falseBranch = &doUpdateActions[4];                // no: A4: fireShot() and reset shot timer

                //doUpdate (Flag)
                doUpdateFlagDecisions[0].decFuncPtr = &RobotPlayer::amAlive;          // is the robot alive?
                doUpdateFlagDecisions[0].trueBranch = &doUpdateFlagDecisions[1];       // yes: Q1
                doUpdateFlagDecisions[0].falseBranch = &doUpdateActions[0];            // no: A0: do nothing

                doUpdateFlagDecisions[1].decFuncPtr = &RobotPlayer::isholdingFlag;    // holding a flag?
                doUpdateFlagDecisions[1].trueBranch = &doUpdateFlagDecisions[2];       // yes: Q2
                doUpdateFlagDecisions[1].falseBranch = &doUpdateActions[0];            // no: A0: do nothing

                doUpdateFlagDecisions[2].decFuncPtr = &RobotPlayer::isflagSticky;     // is the flag sticky?
                doUpdateFlagDecisions[2].trueBranch = &doUpdateActions[0];             // yes: A0: do nothing
                doUpdateFlagDecisions[2].falseBranch = &doUpdateFlagDecisions[3];      // no: Q3

                doUpdateFlagDecisions[3].decFuncPtr = &RobotPlayer::isTeamFlag;       // is the flag a team flag?
                doUpdateFlagDecisions[3].trueBranch = &doUpdateFlagDecisions[4];       // yes: Q4
                doUpdateFlagDecisions[3].falseBranch = &doUpdateActions[5];            // no: A5: drop flag

                doUpdateFlagDecisions[4].decFuncPtr = &RobotPlayer::isMyTeamFlag;     // is the flag my team?
                doUpdateFlagDecisions[4].trueBranch = &doUpdateActions[5];             // yes: A5: drop flag
                doUpdateFlagDecisions[4].falseBranch = &doUpdateActions[0];            // no: A0: do nothing

                doUpdateActions[0].actFuncPtr = &RobotPlayer::a0;   // do nothing
                doUpdateActions[1].actFuncPtr = &RobotPlayer::a1;   // evade
                doUpdateActions[2].actFuncPtr = &RobotPlayer::a2;   // A* search
                doUpdateActions[3].actFuncPtr = &RobotPlayer::a3;   // set shot timer to 0.1f
                doUpdateActions[4].actFuncPtr = &RobotPlayer::a4;   // fireShot() and reset shot timer
                doUpdateActions[5].actFuncPtr = &RobotPlayer::a5;   // drop flag
	}

        DecisionPtr DecisionTrees::doUpdateMotionDecisions[2];
        DecisionPtr DecisionTrees::doUpdateShotDecisions[6];
        DecisionPtr DecisionTrees::doUpdateFlagDecisions[7];
	ActionPtr DecisionTrees::doUpdateActions[6];

        /* end of code modified by Mariah Gaoiran */

}; // end of namespace
