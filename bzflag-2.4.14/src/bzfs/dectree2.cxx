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
#include "dectree2.h"

namespace aicore2
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

	// Set up the trees
	void DecisionTrees::init()
	{
		doUpdateMotionDecisions[0].decFuncPtr = &RobotPlayer::amAlive;
		doUpdateMotionDecisions[0].trueBranch = &doUpdateMotionDecisions[1];
		doUpdateMotionDecisions[0].falseBranch = &doUpdateMotionActions[0];

		doUpdateMotionDecisions[1].decFuncPtr = &RobotPlayer::firingReady;
		doUpdateMotionDecisions[1].trueBranch = &doUpdateMotionDecisions[2];
		doUpdateMotionDecisions[1].falseBranch = &doUpdateMotionActions[0];

		doUpdateMotionDecisions[2].decFuncPtr = &RobotPlayer::shotTimerElapsed;
		doUpdateMotionDecisions[2].trueBranch = &doUpdateMotionDecisions[3];
		doUpdateMotionDecisions[2].falseBranch = &doUpdateMotionActions[0];

		doUpdateMotionDecisions[3].decFuncPtr = &RobotPlayer::shotMissHalfLength;
		doUpdateMotionDecisions[3].trueBranch = &doUpdateMotionDecisions[4];
		doUpdateMotionDecisions[3].falseBranch = &doUpdateMotionActions[0];

		doUpdateMotionDecisions[4].decFuncPtr = &RobotPlayer::buildingsInWay;
		doUpdateMotionDecisions[4].trueBranch = &doUpdateMotionActions[0];
		doUpdateMotionDecisions[4].falseBranch = &doUpdateMotionDecisions[5];

		doUpdateMotionDecisions[5].decFuncPtr = &RobotPlayer::teamMatesInWay;
		doUpdateMotionDecisions[5].trueBranch = &doUpdateMotionActions[1];
		doUpdateMotionDecisions[5].falseBranch = &doUpdateMotionDecisions[2];

		doUpdateMotionDecisions[6].decFuncPtr = &RobotPlayer::returnTrue;
		doUpdateMotionDecisions[6].trueBranch = &doUpdateMotionActions[4];
		doUpdateMotionDecisions[6].falseBranch = &doUpdateMotionActions[2];

		doUpdateMotionActions[0].actFuncPtr = &RobotPlayer::b1; // do nothing
		doUpdateMotionActions[1].actFuncPtr = &RobotPlayer::b2; // set shot timer to 0.1f
		doUpdateMotionActions[2].actFuncPtr = &RobotPlayer::b3; // fireShot() and reset the shot timer
		doUpdateMotionActions[3].actFuncPtr = &RobotPlayer::b4;
		doUpdateMotionActions[4].actFuncPtr = &RobotPlayer::b5;
	}

	DecisionPtr DecisionTrees::doUpdateMotionDecisions[7];
	ActionPtr DecisionTrees::doUpdateMotionActions[5];

}; // end of namespace
