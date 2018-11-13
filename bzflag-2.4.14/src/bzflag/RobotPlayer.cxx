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

  /* lines added by David Chin */
  //#define TRACE
  //#define TRACE2
  //#define TRACE_PLANNER
  /* end of lines added by David Chin */

  // interface header
#include "RobotPlayer.h"

// common implementation headers
#include "BZDBCache.h"

// local implementation headers
#include "World.h"
#include "Intersect.h"
#include "TargetingUtils.h"

/* lines added by David Chin */
#include "playing.h" // needed for numFlags, controlPanel, serverLink
#include "Roster.h" // needed for robots[]
#include "BZDBCache.h" // needed for worldSize, tankRadius
#include <time.h>  // needed for clock_t, clock, CLOCKS_PER_SECOND
#include "dectree.h" // needed for decision trees
/* end of lines added by David Chin */

std::vector<BzfRegion*>* RobotPlayer::obstacleList = NULL;

/* lines added by David Chin */
const float RobotPlayer::CohesionW = 1.0f;
const float RobotPlayer::SeparationW = 1000.0f;
const float RobotPlayer::AlignW = 1.0f;
const float RobotPlayer::PathW = 10.0f;
TimeKeeper RobotPlayer::tick = TimeKeeper::getTick();
float RobotPlayer::wanderAzimuth = 0.0f;
/* end of lines added by David Chin */

RobotPlayer::RobotPlayer(const PlayerId& _id, const char* _name,
  ServerLink* _server,
  const char* _motto = "") :
  LocalPlayer(_id, _name, _motto),
  target(NULL),
  /* lines modified by David Chin */
  pathIndex(-1),
  /* end of lines modified by David Chin */
  timerForShot(0.0f),
  drivingForward(true)
{
  gettingSound = false;
  server = _server;
}

// estimate a player's position at now+t, similar to dead reckoning
void RobotPlayer::projectPosition(const Player *targ, const float t, float &x, float &y, float &z) const
{
  double hisx = targ->getPosition()[0];
  double hisy = targ->getPosition()[1];
  double hisz = targ->getPosition()[2];
  double hisvx = targ->getVelocity()[0];
  double hisvy = targ->getVelocity()[1];
  double hisvz = targ->getVelocity()[2];
  double omega = fabs(targ->getAngularVelocity());
  double sx, sy;

  if ((targ->getStatus() & PlayerState::Falling) || fabs(omega) < 2 * M_PI / 360 * 0.5)
  {
    sx = t * hisvx;
    sy = t * hisvy;
  }
  else
  {
    double hisspeed = hypotf(hisvx, hisvy);
    double alfa = omega * t;
    double r = hisspeed / fabs(omega);
    double dx = r * sin(alfa);
    double dy2 = r * (1 - cos(alfa));
    double beta = atan2(dy2, dx) * (targ->getAngularVelocity() > 0 ? 1 : -1);
    double gamma = atan2(hisvy, hisvx);
    double rho = gamma + beta;
    sx = hisspeed * t * cos(rho);
    sy = hisspeed * t * sin(rho);
  }
  x = (float)hisx + (float)sx;
  y = (float)hisy + (float)sy;
  z = (float)hisz + (float)hisvz*t;
  if (targ->getStatus() & PlayerState::Falling)
    z += 0.5f * BZDBCache::gravity * t * t;
  if (z < 0) z = 0;
}


// get coordinates to aim at when shooting a player; steps:
// 1. estimate how long it will take shot to hit target
// 2. calc position of target at that point of time
// 3. jump to 1., using projected position, loop until result is stable
void RobotPlayer::getProjectedPosition(const Player *targ, float *projpos) const
{
  double myx = getPosition()[0];
  double myy = getPosition()[1];
  double hisx = targ->getPosition()[0];
  double hisy = targ->getPosition()[1];
  double deltax = hisx - myx;
  double deltay = hisy - myy;
  double distance = hypotf(deltax, deltay) - BZDB.eval(StateDatabase::BZDB_MUZZLEFRONT) - BZDBCache::tankRadius;
  if (distance <= 0) distance = 0;
  double shotspeed = BZDB.eval(StateDatabase::BZDB_SHOTSPEED)*
    (getFlag() == Flags::Laser ? BZDB.eval(StateDatabase::BZDB_LASERADVEL) :
      getFlag() == Flags::RapidFire ? BZDB.eval(StateDatabase::BZDB_RFIREADVEL) :
      getFlag() == Flags::MachineGun ? BZDB.eval(StateDatabase::BZDB_MGUNADVEL) : 1) +
    hypotf(getVelocity()[0], getVelocity()[1]);

  double errdistance = 1.0;
  float tx, ty, tz;
  for (int tries = 0; errdistance > 0.05 && tries < 4; tries++)
  {
    float t = (float)distance / (float)shotspeed;
    projectPosition(targ, t + 0.05f, tx, ty, tz); // add 50ms for lag
    double distance2 = hypotf(tx - myx, ty - myy);
    errdistance = fabs(distance2 - distance) / (distance + ZERO_TOLERANCE);
    distance = distance2;
  }
  projpos[0] = tx; projpos[1] = ty; projpos[2] = tz;

  // projected pos in building -> use current pos
  if (World::getWorld()->inBuilding(projpos, 0.0f, BZDBCache::tankHeight)) {
    projpos[0] = targ->getPosition()[0];
    projpos[1] = targ->getPosition()[1];
    projpos[2] = targ->getPosition()[2];
  }
}


void			RobotPlayer::doUpdate(float dt)
{
  LocalPlayer::doUpdate(dt);

  float tankRadius = BZDBCache::tankRadius;
  const float shotRange = BZDB.eval(StateDatabase::BZDB_SHOTRANGE);
  const float shotRadius = BZDB.eval(StateDatabase::BZDB_SHOTRADIUS);

  // fire shot if any available
  timerForShot -= dt;
  if (timerForShot < 0.0f)
    timerForShot = 0.0f;

  if (getFiringStatus() != Ready)
    return;

  bool	shoot = false;
  const float azimuth = getAngle();
  // Allow shooting only if angle is near and timer has elapsed
/* lines added by David Chin */
  if (timerForShot <= 0.0f && target) {
    /* end of lines added by David Chin */
    float p1[3];
    getProjectedPosition(target, p1);
    const float* p2 = getPosition();
    float shootingAngle = atan2f(p1[1] - p2[1], p1[0] - p2[0]);
    if (shootingAngle < 0.0f)
      shootingAngle += (float)(2.0 * M_PI);
    float azimuthDiff = shootingAngle - azimuth;
    if (azimuthDiff > M_PI)
      azimuthDiff -= (float)(2.0 * M_PI);
    else
      if (azimuthDiff < -M_PI)
        azimuthDiff += (float)(2.0 * M_PI);

    const float targetdistance = hypotf(p1[0] - p2[0], p1[1] - p2[1]) -
      BZDB.eval(StateDatabase::BZDB_MUZZLEFRONT) - tankRadius;

    const float missby = fabs(azimuthDiff) *
      (targetdistance - BZDBCache::tankLength);
    // only shoot if we miss by less than half a tanklength and no building inbetween
    if (missby < 0.5f * BZDBCache::tankLength &&
      p1[2] < shotRadius) {
      float pos[3] = { getPosition()[0], getPosition()[1],
                      getPosition()[2] + BZDB.eval(StateDatabase::BZDB_MUZZLEHEIGHT) };
      float dir[3] = { cosf(azimuth), sinf(azimuth), 0.0f };
      Ray tankRay(pos, dir);
      float maxdistance = targetdistance;
      shoot = true;
      // try to not aim at teammates
      for (int i = 0; i <= World::getWorld()->getCurMaxPlayers(); i++)
      {
        Player *p = 0;
        if (i < World::getWorld()->getCurMaxPlayers())
          p = World::getWorld()->getPlayer(i);
        else
          p = LocalPlayer::getMyTank();
        if (!p || p->getId() == getId() || validTeamTarget(p) ||
          !p->isAlive()) continue;
        float relpos[3] = { getPosition()[0] - p->getPosition()[0],
                           getPosition()[1] - p->getPosition()[1],
                           getPosition()[2] - p->getPosition()[2] };
        Ray ray(relpos, dir);
        float impact = rayAtDistanceFromOrigin(ray, 5 * BZDBCache::tankRadius);
        if (impact > 0 && impact < shotRange)
        {
          shoot = false;
          timerForShot = 0.1f;
          break;
        }
      }
      if (shoot && fireShot()) {
        // separate shot by 0.2 - 0.8 sec (experimental value)
        timerForShot = float(bzfrand()) * 0.6f + 0.2f;
      }
    }
  }
  /* lines added by David Chin */
    // drop any flags that are not opponent team flags
    // that can be dropped (i.e., are not FlagSticky)
  FlagType* myFlag = getFlag();
  if (myFlag && (myFlag != Flags::Null) &&
    (myFlag->endurance != FlagSticky) &&
    ((myFlag->flagTeam == NoTeam) || (myFlag->flagTeam == getTeam())))
    serverLink->sendDropFlag(getId(), getPosition());
  /* end of lines added by David Chin */
}

void			RobotPlayer::doUpdateMotion(float dt)
{
  /* Lines added by Brendt McFeeley and Kian Kobayashi */

  if (RobotPlayer::amAlive(dt)) {
    while (!RobotPlayer::foundEnemyFlag(dt)) {
      RobotPlayer::getEnemyFlag(dt);
    }
    while (!RobotPlayer::haveEnemyFlag(dt)) {
      RobotPlayer::returnEnemyFlag(dt);
    }
    while (!RobotPlayer::returnedEnemyFlag(dt)) {
    RobotPlayer:getTeamFlag(dt);
    }
    while (!RobotPlayer::haveTeamFlag(dt)) {
      RobotPlayer::returnTeamFlag(dt);
    }
  }

  /* End of lines added by Brendt McFeeley and Kian Kobayashi */
  LocalPlayer::doUpdateMotion(dt);
}

void			RobotPlayer::explodeTank()
{
  LocalPlayer::explodeTank();
  target = NULL;
  path.clear();
  /* lines added by David Chin */
  paths.clear();
  /* end of lines added by David Chin */
}

void			RobotPlayer::restart(const float* pos, float _azimuth)
{
  LocalPlayer::restart(pos, _azimuth);
  // no target
  path.clear();
  target = NULL;
  /* lines added/modified by David Chin */
  paths.clear();
  pathIndex = -1;
  /* end of lines added/modified by David Chin */

}

float			RobotPlayer::getTargetPriority(const
  Player* _target) const
{
  // don't target teammates or myself
  if (!this->validTeamTarget(_target))
    return 0.0f;

  // go after closest player
  // FIXME -- this is a pretty stupid heuristic
  const float worldSize = BZDBCache::worldSize;
  const float* p1 = getPosition();
  const float* p2 = _target->getPosition();

  float basePriority = 1.0f;
  // give bonus to non-paused player
  if (!_target->isPaused())
    basePriority += 2.0f;
  // give bonus to non-deadzone targets
  if (obstacleList) {
    float nearest[2];
    const BzfRegion* targetRegion = findRegion(p2, nearest);
    if (targetRegion && targetRegion->isInside(p2))
      basePriority += 1.0f;
  }
  return basePriority
    - 0.5f * hypotf(p2[0] - p1[0], p2[1] - p1[1]) / worldSize;
}

void		    RobotPlayer::setObstacleList(std::vector<BzfRegion*>*
  _obstacleList)
{
  obstacleList = _obstacleList;
  /* lines added by David Chin */
  //aicore::DecisionTrees::init();
  /* end of lines added by David Chin */
}

const Player*		RobotPlayer::getTarget() const
{
  return target;
}

void			RobotPlayer::setTarget(const Player* _target)
{

  /* lines added/modified by David Chin */
    //static int mailbox = 0;

    //path.clear();
  target = _target;
  //if (!target) return;

  TeamColor myteam = getTeam();
  float goalPos[3];
  if (myTeamHoldingOpponentFlag())
    findHomeBase(myteam, goalPos);
  else
    findOpponentFlag(goalPos);

  AStarNode goalNode(goalPos);
  if (!paths.empty() && goalNode == pathGoalNode)
    return; // same goal so no need to plan again

  clock_t start_s = clock();
  aStarSearch(getPosition(), goalPos, paths);
  clock_t stop_s = clock();
  float sum = (float)(stop_s - start_s) / CLOCKS_PER_SEC;
  char buffer[128];
  sprintf(buffer, "\nA* search took %f seconds", sum);
  controlPanel->addMessage(buffer);
  if (!paths.empty()) {
    pathGoalNode.setX(paths[0][0].getX());
    pathGoalNode.setY(paths[0][0].getY());
    pathIndex = paths[0].size() - 2; // last index is start node
  }
#ifdef TRACE2
  char buffer[128];
  sprintf(buffer, "\nNumber of paths: %d\nPath coordinates: \n[ ", paths.size());
  controlPanel->addMessage(buffer);
  for (int a = 0; a < paths[0].size(); a++) {
    sprintf(buffer, "[%d, %d]; ", paths[0][a].getX(), paths[0][a].getY());
    controlPanel->addMessage(buffer);
  }
  controlPanel->addMessage(" ]\n\n");
#endif

  /*
    // work backwards (from target to me)
    float proj[3];
    getProjectedPosition(target, proj);
    const float *p1 = proj;
    const float* p2 = getPosition();
    float q1[2], q2[2];
    BzfRegion* headRegion = findRegion(p1, q1);
    BzfRegion* tailRegion = findRegion(p2, q2);
    if (!headRegion || !tailRegion) {
  #ifdef TRACE2
            char buffer[128];
            sprintf (buffer, "setTarget cannot find path between Regions head=%d and tail=%d",
                    (int)headRegion, (int)tailRegion);
            controlPanel->addMessage(buffer);
  #endif
      return;
    }

    mailbox++;
    headRegion->setPathStuff(0.0f, NULL, q1, mailbox);
    RegionPriorityQueue queue;
    queue.insert(headRegion, 0.0f);
    BzfRegion* next;
    while (!queue.isEmpty() && (next = queue.remove()) != tailRegion)
      findPath(queue, next, tailRegion, q2, mailbox);

    // get list of points to go through to reach the target
    next = tailRegion;
    do {
      p1 = next->getA();
      path.push_back(p1);
      next = next->getTarget();
    } while (next && next != headRegion);
    if (next || tailRegion == headRegion)
      path.push_back(q1);
    else
      path.clear();
    pathIndex = 0;
  */
  /* end of lines added/modified by David Chin */
}

BzfRegion*		RobotPlayer::findRegion(const float p[2],
  float nearest[2]) const
{
  nearest[0] = p[0];
  nearest[1] = p[1];
  const int count = obstacleList->size();
  for (int o = 0; o < count; o++)
    if ((*obstacleList)[o]->isInside(p))
      return (*obstacleList)[o];

  // point is outside: find nearest region
  float      distance = maxDistance;
  BzfRegion* nearestRegion = NULL;
  for (int i = 0; i < count; i++) {
    float currNearest[2];
    float currDistance = (*obstacleList)[i]->getDistance(p, currNearest);
    if (currDistance < distance) {
      nearestRegion = (*obstacleList)[i];
      distance = currDistance;
      nearest[0] = currNearest[0];
      nearest[1] = currNearest[1];
    }
  }
  return nearestRegion;
}

float			RobotPlayer::getRegionExitPoint(
  const float p1[2], const float p2[2],
  const float a[2], const float targetPoint[2],
  float mid[2], float& priority)
{
  float b[2];
  b[0] = targetPoint[0] - a[0];
  b[1] = targetPoint[1] - a[1];
  float d[2];
  d[0] = p2[0] - p1[0];
  d[1] = p2[1] - p1[1];

  float vect = d[0] * b[1] - d[1] * b[0];
  float t = 0.0f;  // safe value
  if (fabs(vect) > ZERO_TOLERANCE) {
    // compute intersection along (p1,d) with (a,b)
    t = (a[0] * b[1] - a[1] * b[0] - p1[0] * b[1] + p1[1] * b[0]) / vect;
    if (t > 1.0f)
      t = 1.0f;
    else if (t < 0.0f)
      t = 0.0f;
  }

  mid[0] = p1[0] + t * d[0];
  mid[1] = p1[1] + t * d[1];

  const float distance = hypotf(a[0] - mid[0], a[1] - mid[1]);
  // priority is to minimize distance traveled and distance left to go
  priority = distance + hypotf(targetPoint[0] - mid[0], targetPoint[1] - mid[1]);
  // return distance traveled
  return distance;
}

void			RobotPlayer::findPath(RegionPriorityQueue& queue,
  BzfRegion* region,
  BzfRegion* targetRegion,
  const float targetPoint[2],
  int mailbox)
{
  const int numEdges = region->getNumSides();
  for (int i = 0; i < numEdges; i++) {
    BzfRegion* neighbor = region->getNeighbor(i);
    if (!neighbor) continue;

    const float* p1 = region->getCorner(i).get();
    const float* p2 = region->getCorner((i + 1) % numEdges).get();
    float mid[2], priority;
    float total = getRegionExitPoint(p1, p2, region->getA(),
      targetPoint, mid, priority);
    priority += region->getDistance();
    if (neighbor == targetRegion)
      total += hypotf(targetPoint[0] - mid[0], targetPoint[1] - mid[1]);
    total += region->getDistance();
    if (neighbor->test(mailbox) || total < neighbor->getDistance()) {
      neighbor->setPathStuff(total, region, mid, mailbox);
      queue.insert(neighbor, priority);
    }
  }
}


/* lines added/modified by David Chin */
/**
 * Return the number of teammates (not including self)
 * within neighborhoodSize distance of self; also return
 * their center of mass position (in cmOut)
 */
int		RobotPlayer::computeCenterOfMass(float neighborhoodSize, float cmOut[3])
{
  cmOut[0] = cmOut[1] = cmOut[2] = 0.0f;
  const float* mypos = getPosition();
  int numTeammates = 0;
  TeamColor myTeam = getTeam();
#ifdef TRACE
  char buffer[128];
  sprintf(buffer, "my (id=%d) team color is %d", getId(), myTeam);
  controlPanel->addMessage(buffer);
  sprintf(buffer, "curMaxPlayers() is  %d", World::getWorld()->getCurMaxPlayers());
  controlPanel->addMessage(buffer);
#endif
  for (int i = 0; i <= World::getWorld()->getCurMaxPlayers(); i++)
  {
    Player* p = NULL;
    const float* pos; // p's position
    double distance = 0; // distance from p to this robot
    if (i < World::getWorld()->getCurMaxPlayers())
      p = World::getWorld()->getPlayer(i);
    else
      p = LocalPlayer::getMyTank();

#ifdef TRACE
    if (p) sprintf(buffer, "getPlayer(%d), id=%d has team color %d",
      i, p->getId(), p->getTeam());
    controlPanel->addMessage(buffer);
#endif
    if (p && p->getTeam() == myTeam && p->getId() != getId()) {
      pos = p->getPosition();
      double deltax = pos[0] - mypos[0];
      double deltay = pos[1] - mypos[1];
      distance = hypotf(deltax, deltay);
#ifdef TRACE
      sprintf(buffer, "getPlayer(%d), id=%d has location (%f, %f, %f)",
        i, p->getId(), pos[0], pos[1], pos[2]);
      controlPanel->addMessage(buffer);
      sprintf(buffer, "distance = %f, neighborhood = %f",
        distance, neighborhoodSize);
      controlPanel->addMessage(buffer);
#endif
      if (distance < neighborhoodSize) {
        numTeammates++;
        cmOut[0] += pos[0];
        cmOut[1] += pos[1];
        cmOut[2] += pos[2];
      }
    }
  }
#ifdef TRACE
  sprintf(buffer, "numTeammates = %d",
    numTeammates);
  controlPanel->addMessage(buffer);
#endif
  if (numTeammates) {
    cmOut[0] /= numTeammates;
    cmOut[1] /= numTeammates;
    cmOut[2] /= numTeammates;
  }
  return numTeammates;
}

/**
 * Return the number of teammates (not including self)
 * within neighborhoodSize distance of self; also return
 * the inverse square repulsion of those teammates in repulseOut[3]
 */
int		RobotPlayer::computeRepulsion(float neighborhoodSize, float repulseOut[3])
{
  repulseOut[0] = repulseOut[1] = repulseOut[2] = 0.0f;
  const float* mypos = getPosition();
  int numTeammates = 0;
  TeamColor myTeam = getTeam();
#ifdef TRACE
  char buffer[128];
  sprintf(buffer, "my (id=%d) team color is %d", getId(), myTeam);
  controlPanel->addMessage(buffer);
  sprintf(buffer, "curMaxPlayers() is  %d", World::getWorld()->getCurMaxPlayers());
  controlPanel->addMessage(buffer);
#endif
  for (int i = 0; i <= World::getWorld()->getCurMaxPlayers(); i++)
  {
    Player* p = NULL;
    const float* pos; // p's position
    float direction[3]; // flee direction
    float distance = 0; // distance from p to this robot
    if (i < World::getWorld()->getCurMaxPlayers())
      p = World::getWorld()->getPlayer(i);
    else
      p = LocalPlayer::getMyTank();

#ifdef TRACE
    if (p) sprintf(buffer, "getPlayer(%d), id=%d has team color %d",
      i, p->getId(), p->getTeam());
    controlPanel->addMessage(buffer);
#endif
    if (p && p->getTeam() == myTeam && p->getId() != getId()) {
      pos = p->getPosition();
      direction[0] = mypos[0] - pos[0];
      direction[1] = mypos[1] - pos[1];
      distance = hypotf(direction[0], direction[1]);
      if (distance == 0.0) {
        distance = 0.0001f; // in case tanks on top of each other
        direction[0] = float(bzfrand());
        direction[1] = float(bzfrand());
      }
#ifdef TRACE
      sprintf(buffer, "getPlayer(%d), id=%d has location (%f, %f, %f)",
        i, p->getId(), pos[0], pos[1], pos[2]);
      controlPanel->addMessage(buffer);
      sprintf(buffer, "distance = %f, neighborhood = %f",
        distance, neighborhoodSize);
      controlPanel->addMessage(buffer);
#endif
      if (distance < neighborhoodSize) {
        numTeammates++;
        // normalize to unit vector and inverse square law
        float dd = distance * distance * distance;
        repulseOut[0] += direction[0] / dd;
        repulseOut[1] += direction[1] / dd;
        repulseOut[2] += direction[2] / dd;
      }
    }
  }
#ifdef TRACE
  sprintf(buffer, "numTeammates = %d",
    numTeammates);
  controlPanel->addMessage(buffer);
#endif
  return numTeammates;
}

/**
 * Return the number of teammates (not including self)
 * within neighborhoodSize distance of self; also return
 * their average velocity (in avVOut) and their average angle (in avAzimuthOut)
 */
int		RobotPlayer::computeAlign(float neighborhoodSize, float avVOut[3], float* avAzimuthOut)
{
  avVOut[0] = avVOut[1] = avVOut[2] = 0.0f;
  const float* mypos = getPosition();
  int numTeammates = 0;
  TeamColor myTeam = getTeam();
  *avAzimuthOut = 0;
#ifdef TRACE
  char buffer[128];
  sprintf(buffer, "my (id=%d) team color is %d", getId(), myTeam);
  controlPanel->addMessage(buffer);
  sprintf(buffer, "curMaxPlayers() is  %d", World::getWorld()->getCurMaxPlayers());
  controlPanel->addMessage(buffer);
#endif
  for (int i = 0; i <= World::getWorld()->getCurMaxPlayers(); i++)
  {
    Player* p = NULL;
    const float* pos; // position of p
    const float* v; // velocity of v
    //const float* azimuth; // angle of v
    double distance = 0; // distance from p to this robot
    if (i < World::getWorld()->getCurMaxPlayers())
      p = World::getWorld()->getPlayer(i);
    else
      p = LocalPlayer::getMyTank();

#ifdef TRACE
    if (p) sprintf(buffer, "getPlayer(%d), id=%d has team color %d",
      i, p->getId(), p->getTeam());
    controlPanel->addMessage(buffer);
#endif
    if (p && p->getTeam() == myTeam && p->getId() != getId()) {
      pos = p->getPosition();
      double deltax = pos[0] - mypos[0];
      double deltay = pos[1] - mypos[1];
      distance = hypotf(deltax, deltay);
      v = p->getVelocity();
#ifdef TRACE
      sprintf(buffer, "getPlayer(%d), id=%d has velocity (%f, %f, %f)",
        i, p->getId(), v[0], v[1], v[2]);
      controlPanel->addMessage(buffer);
      sprintf(buffer, "distance = %f, neighborhood = %f",
        distance, neighborhoodSize);
      controlPanel->addMessage(buffer);
#endif
      if (distance < neighborhoodSize) {
        numTeammates++;
        avVOut[0] += v[0];
        avVOut[1] += v[1];
        avVOut[2] += v[2];
        *avAzimuthOut += p->getAngle();
      }
    }
  }
#ifdef TRACE
  sprintf(buffer, "numTeammates = %d",
    numTeammates);
  controlPanel->addMessage(buffer);
#endif
  if (numTeammates) {
    avVOut[0] /= numTeammates;
    avVOut[1] /= numTeammates;
    avVOut[2] /= numTeammates;
    *avAzimuthOut /= numTeammates;
  }
  return numTeammates;
}

/*
 * Return the location (in location[3]) of the first home base for team color teamColor
 */
void		RobotPlayer::findHomeBase(TeamColor teamColor, float location[3])
{
  World* world = World::getWorld();
  if (!world->allowTeamFlags()) return;
  const float* baseParms = world->getBase(teamColor, 0);
#ifdef TRACE2
  char buffer[128];
  sprintf(buffer, "Base for color %d is at (%f, %f, %f)",
    teamColor, baseParms[0], baseParms[1], baseParms[2]);
  controlPanel->addMessage(buffer);
#endif
  location[0] = baseParms[0];
  location[1] = baseParms[1];
  location[2] = baseParms[2];
}

/*
 * Return true if any player on my team
 * is holding an opponent team flag, and false otherwise
 */
bool		RobotPlayer::myTeamHoldingOpponentFlag(void)
{
  TeamColor myTeamColor = getTeam();
  if (!World::getWorld()->allowTeamFlags()) return false;
  for (int i = 0; i < numFlags; i++) {
    Flag& flag = World::getWorld()->getFlag(i);
    TeamColor flagTeamColor = flag.type->flagTeam;
    if (flagTeamColor != NoTeam && flagTeamColor != myTeamColor
      && flag.status == FlagOnTank) {
      PlayerId ownerId = flag.owner;
#ifdef TRACE2
      char buffer[128];
      sprintf(buffer, "Looking for a Player with id=%d",
        ownerId);
      controlPanel->addMessage(buffer);
#endif
      Player* p = lookupLocalPlayer(ownerId);
      if (p && (p->getTeam() == myTeamColor)) {
#ifdef TRACE2
        sprintf(buffer, "Player id=%d, TeamColor=%d holds flag %d, robots[0]=%d",
          p->getId(), p->getTeam(), myTeamColor, robots[0]->getId());
        controlPanel->addMessage(buffer);
#endif
        return true;
      }
    }
  }
#ifdef TRACE2
  char buffer[128];
  sprintf(buffer, "Robot(%d)'s team is not holding a team flag",
    getId());
  controlPanel->addMessage(buffer);
#endif
  return false;
}

/*
 * Find any opponent flag and return its location
 */
void		RobotPlayer::findOpponentFlag(float location[3])
{
  TeamColor myTeamColor = getTeam();
  if (!World::getWorld()->allowTeamFlags()) return;
  for (int i = 0; i < numFlags; i++) {
    Flag& flag = World::getWorld()->getFlag(i);
    TeamColor flagTeamColor = flag.type->flagTeam;
    if (flagTeamColor != NoTeam && flagTeamColor != myTeamColor) {
      location[0] = flag.position[0];
      location[1] = flag.position[1];
      location[2] = flag.position[2];
#ifdef TRACE2
      char buffer[128];
      sprintf(buffer, "Robot(%d) found a flag at (%f, %f, %f)",
        getId(), location[0], location[1], location[2]);
      controlPanel->addMessage(buffer);
#endif
      return;
    }
  }
}

/*
 * Compute a random wander angle between -0.05 and 0.05
 */
float 		RobotPlayer::computeWander(void)
{
  TimeKeeper now = TimeKeeper::getTick();
  if (tick == now) {
    return wanderAzimuth;
  }
  else {
    tick = now;
    return wanderAzimuth = 1.0f + (float(bzfrand()) - 0.5f) * 0.1f;
  }
}

/*
 * Given a PlayerId, find the corresponding local Player
 */
Player*		RobotPlayer::lookupLocalPlayer(PlayerId id)
{
  for (int i = 0; i <= World::getWorld()->getCurMaxPlayers(); i++)
  {
    Player* p = NULL;
    if (i < World::getWorld()->getCurMaxPlayers())
      p = World::getWorld()->getPlayer(i);
    else
      p = LocalPlayer::getMyTank();
    if (p && p->getId() == id) return p;
  }
  return NULL;
}

/*
 * Use A* search to find a path from start to goal
 * store the results (a vector of vector of AStarNodes) in paths
 * typically only paths[0] will be non-empty
 * Note that paths[0][0] is the same as the goal node
 * and paths[0][paths.size()-1] is the same as the start node
 */
void		RobotPlayer::aStarSearch(const float startPos[3], const float goalPos[3],
  std::vector< std::vector< AStarNode > >& paths)
{
  // Profiling observation: Using int instead of double cost provides marginal improvement (~10%)
  GenericSearchGraphDescriptor<AStarNode, double> AStarGraph;
  GraphFunctionContainer fun_cont(BZDBCache::worldSize);
  AStarGraph.func_container = &fun_cont;
  // Set other variables
  AStarGraph.hashTableSize = 1024; // Since in this problem, "getHashBin" can return a max of value 1024.
  AStarGraph.hashBinSizeIncreaseStep = 256; // By default it's 128. For this problem, we choose a higher value. 
  AStarGraph.SeedNode = AStarNode(startPos); // Start node
  AStarGraph.TargetNode = AStarNode(goalPos); // Goal node
  A_star_planner<AStarNode, double> planner;
  planner.setParams(1.0, 10); // optional.
  planner.init(&AStarGraph);
  planner.plan();
  if (!paths.empty()) paths.clear();
  paths = planner.getPlannedPaths();
  if (paths.empty()) {
    char buffer[128];
    sprintf(buffer, "***RobotPlayer::aStarSearch: R%d-%d could not find a path from (%f, %f) to (%f, %f)***",
      getTeam(), getId(), startPos[0], startPos[1], goalPos[0], goalPos[1]);
    controlPanel->addMessage(buffer);
  }
#ifdef TRACE_PLANNER
  char buffer[128];
  sprintf(buffer, "R%d-%d planning from (%f, %f) to (%f, %f) with plan size %d",
    getTeam(), getId(), startPos[0], startPos[1], goalPos[0], goalPos[1], paths[0].size());
  controlPanel->addMessage(buffer);
  for (int a = 0; a < paths[0].size(); a++) {
    sprintf(buffer, "[%f, %f]; ", paths[0][a].getScaledX(), paths[0][a].getScaledY());
    controlPanel->addMessage(buffer);
  }
  controlPanel->addMessage(" ]\n\n");
#endif
}

bool			RobotPlayer::returnTrue(float dt)
{
  return true;
}

bool			RobotPlayer::returnFalse(float dt)
{
  return false;
}

/* end of lines added by David Chin */

/* Lines added by Brendt McFeeley, Kian Kobayashi */

/*
        State functions for ICS 462 Assignment 5
        This code assumes that it will be inside RobotPlayer.cxx and that the user has applied David Chin's patch to allow robot tanks to pick up and drop flags.
        Shamelessly based off of code from RobotPlayer.cxx.
        Written by Kian Kobayashi
*/

// Search for an enemy flag, then path to it
void			RobotPlayer::getEnemyFlag(float dt)
{
  controlPanel->addMessage("A1. Get enemy flag.");
  if (isAlive()) {
    bool evading = false;
    // record previous position
    const float oldAzimuth = getAngle();
    const float* oldPosition = getPosition();
    float position[3];
    position[0] = oldPosition[0];
    position[1] = oldPosition[1];
    position[2] = oldPosition[2];
    float azimuth = oldAzimuth;
    float tankAngVel = BZDB.eval(StateDatabase::BZDB_TANKANGVEL);
    float tankSpeed = BZDBCache::tankSpeed;

    // when we are not evading, follow the path
    if (!evading && dt > 0.0 && pathIndex < (int)path.size()) {
      float distance;
      float v[2];

      /* lines modified or added by Kian Kobayashi */

      //Check whether carrying noTeam flag or own team flag, drop if possible
      if (Player::getFlag() != NULL && (Player::getFlag()->flagTeam == NoTeam || Player::getFlag()->flagTeam == getTeam())) {
        if (Player::getFlag()->endurance != FlagSticky) {
          server->sendDropFlag(getId(), position);
        }
      }

      float* endPoint;
      Flag flag;
      Flag* flagpoint = &flag;
      //Loop through all flags and check for a teamed non-myteam flag
      for (int i = 0; i < World::getWorld()->getMaxFlags(); i++) {
        flag = World::getWorld()->getFlag(i);
        flagpoint = &flag;
        if (flagpoint->type->flagTeam != NoTeam && flagpoint->type->flagTeam != getTeam()) {
          break;
        }
      }
      endPoint = flagpoint->position;

      /* end of lines modified or added by Kian Kobayashi */

      // find how long it will take to get to next path segment
      v[0] = endPoint[0] - position[0];
      v[1] = endPoint[1] - position[1];
      distance = hypotf(v[0], v[1]);
      float tankRadius = BZDBCache::tankRadius;
      // smooth path a little by turning early at corners, might get us stuck, though
      if (distance <= 2.5f * tankRadius)
        pathIndex++;

      float segmentAzimuth = atan2f(v[1], v[0]);
      float azimuthDiff = segmentAzimuth - azimuth;
      if (azimuthDiff > M_PI) azimuthDiff -= (float)(2.0 * M_PI);
      else if (azimuthDiff < -M_PI) azimuthDiff += (float)(2.0 * M_PI);
      if (fabs(azimuthDiff) > 0.01f) {
        // drive backward when target is behind, try to stick to last direction
        if (drivingForward)
          drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.9 ? true : false;
        else
          drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.3 ? true : false;
        setDesiredSpeed(drivingForward ? 1.0f : -1.0f);
        // set desired turn speed
        if (azimuthDiff >= dt * tankAngVel) {
          setDesiredAngVel(1.0f);
        }
        else if (azimuthDiff <= -dt * tankAngVel) {
          setDesiredAngVel(-1.0f);
        }
        else {
          setDesiredAngVel(azimuthDiff / dt / tankAngVel);
        }
      }
      else {
        drivingForward = true;
        // tank doesn't turn while moving forward
        setDesiredAngVel(0.0f);
        // find how long it will take to get to next path segment
        if (distance <= dt * tankSpeed) {
          pathIndex++;
          // set desired speed
          setDesiredSpeed(distance / dt / tankSpeed);
        }
        else {
          setDesiredSpeed(1.0f);
        }
      }
    }
  }

}

// Assumes holding an enemy flag. Path back to base and drop it.
void			RobotPlayer::returnEnemyFlag(float dt)
{
  controlPanel->addMessage("A2. Return enemy flag.");

  if (isAlive()) {
    bool evading = false;
    // record previous position
    const float oldAzimuth = getAngle();
    const float* oldPosition = getPosition();
    float position[3];
    position[0] = oldPosition[0];
    position[1] = oldPosition[1];
    position[2] = oldPosition[2];
    float azimuth = oldAzimuth;
    float tankAngVel = BZDB.eval(StateDatabase::BZDB_TANKANGVEL);
    float tankSpeed = BZDBCache::tankSpeed;

    /* lines modified or added by Kian Kobayashi */


    /* end of lines modified or added by Kian Kobayashi */

    // when we are not evading, follow the path
    if (!evading && dt > 0.0 && pathIndex < (int)path.size()) {
      float distance;
      float v[2];

      /* lines modified or added by Kian Kobayashi */

      float* endPoint;

      //If at home base, drop flag
      const float *basePoint = World::getWorld()->getBase(getTeam(), 0);
      if (((basePoint[0] == position[0] && basePoint[1] == position[1])) &&
        getFlag()->flagTeam != getTeam()) {
        server->sendDropFlag(getId(), getPosition());
      }
      //else go home
      endPoint = new float[2];
      endPoint[0] = basePoint[0];
      endPoint[1] = basePoint[1];

      /* end of lines modified or added by Kian Kobayashi */

      // find how long it will take to get to next path segment
      v[0] = endPoint[0] - position[0];
      v[1] = endPoint[1] - position[1];
      distance = hypotf(v[0], v[1]);
      float tankRadius = BZDBCache::tankRadius;
      // smooth path a little by turning early at corners, might get us stuck, though
      if (distance <= 2.5f * tankRadius)
        pathIndex++;

      float segmentAzimuth = atan2f(v[1], v[0]);
      float azimuthDiff = segmentAzimuth - azimuth;
      if (azimuthDiff > M_PI) azimuthDiff -= (float)(2.0 * M_PI);
      else if (azimuthDiff < -M_PI) azimuthDiff += (float)(2.0 * M_PI);
      if (fabs(azimuthDiff) > 0.01f) {
        // drive backward when target is behind, try to stick to last direction
        if (drivingForward)
          drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.9 ? true : false;
        else
          drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.3 ? true : false;
        setDesiredSpeed(drivingForward ? 1.0f : -1.0f);
        // set desired turn speed
        if (azimuthDiff >= dt * tankAngVel) {
          setDesiredAngVel(1.0f);
        }
        else if (azimuthDiff <= -dt * tankAngVel) {
          setDesiredAngVel(-1.0f);
        }
        else {
          setDesiredAngVel(azimuthDiff / dt / tankAngVel);
        }
      }
      else {
        drivingForward = true;
        // tank doesn't turn while moving forward
        setDesiredAngVel(0.0f);
        // find how long it will take to get to next path segment
        if (distance <= dt * tankSpeed) {
          pathIndex++;
          // set desired speed
          setDesiredSpeed(distance / dt / tankSpeed);
        }
        else {
          setDesiredSpeed(1.0f);
        }
      }
    }
  }

}

void			RobotPlayer::getTeamFlag(float dt)
{
  controlPanel->addMessage("A3. Get team flag.");

  if (isAlive()) {
    bool evading = false;
    // record previous position
    const float oldAzimuth = getAngle();
    const float* oldPosition = getPosition();
    float position[3];
    position[0] = oldPosition[0];
    position[1] = oldPosition[1];
    position[2] = oldPosition[2];
    float azimuth = oldAzimuth;
    float tankAngVel = BZDB.eval(StateDatabase::BZDB_TANKANGVEL);
    float tankSpeed = BZDBCache::tankSpeed;

    /* lines modified or added by Kian Kobayashi */


    /* end of lines modified or added by Kian Kobayashi */

    // when we are not evading, follow the path
    if (!evading && dt > 0.0 && pathIndex < (int)path.size()) {
      float distance;
      float v[2];

      /* lines modified or added by Kian Kobayashi */

      float* endPoint;

      //Check whether carrying noTeam flag drop if possible
      if (Player::getFlag() != NULL && (Player::getFlag()->flagTeam == NoTeam)) {
        if (Player::getFlag()->endurance != FlagSticky) {
          server->sendDropFlag(getId(), position);
        }
      }


      //Find the next flag to target
      Flag flag;
      Flag* flagpoint = &flag;
      //Loop through all flags and check for a teamed myteam flag
      for (int i = 0; i < World::getWorld()->getMaxFlags(); i++) {
        flag = World::getWorld()->getFlag(i);
        flagpoint = &flag;
        if (flagpoint->type->flagTeam != NoTeam && flagpoint->type->flagTeam == getTeam()) {
          break;
        }
      }
      endPoint = flagpoint->position;

      /* end of lines modified or added by Kian Kobayashi */

      // find how long it will take to get to next path segment
      v[0] = endPoint[0] - position[0];
      v[1] = endPoint[1] - position[1];
      distance = hypotf(v[0], v[1]);
      float tankRadius = BZDBCache::tankRadius;
      // smooth path a little by turning early at corners, might get us stuck, though
      if (distance <= 2.5f * tankRadius)
        pathIndex++;

      float segmentAzimuth = atan2f(v[1], v[0]);
      float azimuthDiff = segmentAzimuth - azimuth;
      if (azimuthDiff > M_PI) azimuthDiff -= (float)(2.0 * M_PI);
      else if (azimuthDiff < -M_PI) azimuthDiff += (float)(2.0 * M_PI);
      if (fabs(azimuthDiff) > 0.01f) {
        // drive backward when target is behind, try to stick to last direction
        if (drivingForward)
          drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.9 ? true : false;
        else
          drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.3 ? true : false;
        setDesiredSpeed(drivingForward ? 1.0f : -1.0f);
        // set desired turn speed
        if (azimuthDiff >= dt * tankAngVel) {
          setDesiredAngVel(1.0f);
        }
        else if (azimuthDiff <= -dt * tankAngVel) {
          setDesiredAngVel(-1.0f);
        }
        else {
          setDesiredAngVel(azimuthDiff / dt / tankAngVel);
        }
      }
      else {
        drivingForward = true;
        // tank doesn't turn while moving forward
        setDesiredAngVel(0.0f);
        // find how long it will take to get to next path segment
        if (distance <= dt * tankSpeed) {
          pathIndex++;
          // set desired speed
          setDesiredSpeed(distance / dt / tankSpeed);
        }
        else {
          setDesiredSpeed(1.0f);
        }
      }
    }
  }

}

void			RobotPlayer::returnTeamFlag(float dt)
{
  controlPanel->addMessage("A4. Return team flag.");

  if (isAlive()) {
    bool evading = false;
    // record previous position
    const float oldAzimuth = getAngle();
    const float* oldPosition = getPosition();
    float position[3];
    position[0] = oldPosition[0];
    position[1] = oldPosition[1];
    position[2] = oldPosition[2];
    float azimuth = oldAzimuth;
    float tankAngVel = BZDB.eval(StateDatabase::BZDB_TANKANGVEL);
    float tankSpeed = BZDBCache::tankSpeed;

    // when we are not evading, follow the path
    if (!evading && dt > 0.0 && pathIndex < (int)path.size()) {
      float distance;
      float v[2];

      /* lines modified or added by Kian Kobayashi */

      float* endPoint;

      //If carrying team flag
      if (Player::getFlag() != NULL && Player::getFlag()->flagTeam != NoTeam && Player::getFlag()->flagTeam == getTeam()) {
        //If at home base, drop flag
        const float *basePoint = World::getWorld()->getBase(getTeam(), 0);
        if (((basePoint[0] == position[0] && basePoint[1] == position[1])) &&
          getFlag()->flagTeam != getTeam()) {
          server->sendDropFlag(getId(), getPosition());
        }
        //else go home
        endPoint = new float[2];
        endPoint[0] = basePoint[0];
        endPoint[1] = basePoint[1];
      }

      /* end of lines modified or added by Kian Kobayashi */

      // find how long it will take to get to next path segment
      v[0] = endPoint[0] - position[0];
      v[1] = endPoint[1] - position[1];
      distance = hypotf(v[0], v[1]);
      float tankRadius = BZDBCache::tankRadius;
      // smooth path a little by turning early at corners, might get us stuck, though
      if (distance <= 2.5f * tankRadius)
        pathIndex++;

      float segmentAzimuth = atan2f(v[1], v[0]);
      float azimuthDiff = segmentAzimuth - azimuth;
      if (azimuthDiff > M_PI) azimuthDiff -= (float)(2.0 * M_PI);
      else if (azimuthDiff < -M_PI) azimuthDiff += (float)(2.0 * M_PI);
      if (fabs(azimuthDiff) > 0.01f) {
        // drive backward when target is behind, try to stick to last direction
        if (drivingForward)
          drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.9 ? true : false;
        else
          drivingForward = fabs(azimuthDiff) < M_PI / 2 * 0.3 ? true : false;
        setDesiredSpeed(drivingForward ? 1.0f : -1.0f);
        // set desired turn speed
        if (azimuthDiff >= dt * tankAngVel) {
          setDesiredAngVel(1.0f);
        }
        else if (azimuthDiff <= -dt * tankAngVel) {
          setDesiredAngVel(-1.0f);
        }
        else {
          setDesiredAngVel(azimuthDiff / dt / tankAngVel);
        }
      }
      else {
        drivingForward = true;
        // tank doesn't turn while moving forward
        setDesiredAngVel(0.0f);
        // find how long it will take to get to next path segment
        if (distance <= dt * tankSpeed) {
          pathIndex++;
          // set desired speed
          setDesiredSpeed(distance / dt / tankSpeed);
        }
        else {
          setDesiredSpeed(1.0f);
        }
      }
    }
  }

}

bool		RobotPlayer::amAlive(float dt)
{
  return isAlive();
}

bool		RobotPlayer::foundEnemyFlag(float dt)
{
  if (Player::getFlag() != NULL && Player::getFlag()->flagTeam != NoTeam && Player::getFlag()->flagTeam != getTeam()) return true;
  return false;
}
  bool		RobotPlayer::returnedEnemyFlag(float dt)
  {
    if (Player::getFlag() != NULL && Player::getFlag()->flagTeam != NoTeam && Player::getFlag()->flagTeam != getTeam()) return false;
    return true;
  }

  bool		RobotPlayer::haveTeamFlag(float dt)
  {
    if (Player::getFlag() != NULL && Player::getFlag()->flagTeam != NoTeam && Player::getFlag()->flagTeam == getTeam()) return true;
    return false;
  }

  bool		RobotPlayer::haveEnemyFlag(float dt)
  {
    if (Player::getFlag() != NULL && Player::getFlag()->flagTeam != NoTeam && Player::getFlag()->flagTeam != getTeam()) return true;
    return false;
  }

  bool		RobotPlayer::returnedTeamFlag(float dt)
  {
    if (Player::getFlag() != NULL && Player::getFlag()->flagTeam != NoTeam && Player::getFlag()->flagTeam == getTeam()) return false;
    return true;
  }



// Local Variables: ***
// mode: C++ ***
// tab-width: 8 ***
// c-basic-offset: 2 ***
// indent-tabs-mode: t ***
// End: ***
// ex: shiftwidth=2 tabstop=8
