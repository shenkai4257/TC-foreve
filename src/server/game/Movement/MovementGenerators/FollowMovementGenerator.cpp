/*
 * This file is part of the TrinityCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "FollowMovementGenerator.h"
#include "Creature.h"
#include "CreatureAI.h"
#include "MoveSpline.h"
#include "MoveSplineInit.h"
#include "Optional.h"
#include "PathGenerator.h"
#include "Pet.h"
#include "Unit.h"
#include "Player.h"
#include "WorldSession.h"

// Target drift (yards) a playerbot follower tolerates before re-pathing a follow spline that is
// still in flight. 2.5y at run speed is ~0.35 s of leader movement.
static constexpr float FOLLOW_BOT_REPATH_DIST = 2.5f;

// AC parity (TargetedMovementGenerator): a follower only settles once its leader has
// stopped drifting from the point the current follow spline was launched at. While the
// leader keeps moving, the follower keeps running - even when already inside follow
// range - so companions of a faster player track continuously instead of stuttering
// through run/stop/run/stop cycles. AC's speed inheritance / interception only apply
// to pets and guardians (not to playerbot companions, which are Players), so they are
// intentionally NOT ported: companions run at their own natural speed.
static constexpr float FOLLOW_TARGET_DRIFT_SETTLE = 0.5f;

static void DoMovementInform(Unit* owner, Unit* target)
{
    if (owner->GetTypeId() != TYPEID_UNIT)
        return;

    if (CreatureAI* AI = owner->ToCreature()->AI())
        AI->MovementInform(FOLLOW_MOTION_TYPE, target->GetGUID().GetCounter());
}

FollowMovementGenerator::FollowMovementGenerator(Unit* target, float range, Optional<ChaseAngle> angle, Optional<Milliseconds> duration,
    bool ignoreTargetWalk /*= false*/, Scripting::v2::ActionResultSetter<MovementStopReason>&& scriptResult /*= {}*/)
    : AbstractFollower(ASSERT_NOTNULL(target)), _range(range), _angle(angle), _ignoreTargetWalk(ignoreTargetWalk), _checkTimer(CHECK_INTERVAL)
{
    Priority = MOTION_PRIORITY_NORMAL;
    Flags = MOVEMENTGENERATOR_FLAG_INITIALIZATION_PENDING;
    BaseUnitState = UNIT_STATE_FOLLOW;
    ScriptResult = std::move(scriptResult);
    if (duration)
        _duration.emplace(*duration);
}
FollowMovementGenerator::~FollowMovementGenerator() = default;

static bool PositionOkay(Unit* owner, Unit* target, float range, Optional<ChaseAngle> angle = {})
{
    if (!owner->IsInDist(target, owner->GetCombatReach() + target->GetCombatReach() + range))
        return false;

    return !angle || angle->IsAngleOkay(target->GetRelativeAngle(owner));
}

// AC semantics: a follower only settles when its leader has stopped drifting from the
// point the current follow spline was launched at. While the leader keeps moving, the
// follower keeps running - even when already inside follow range.
static bool PositionOkay(Unit* owner, Unit* target, float range, Optional<ChaseAngle> angle, Optional<Position> const& launchPos)
{
    if (!PositionOkay(owner, target, range, angle))
        return false;

    if (launchPos && launchPos->GetExactDistSq(target->GetPosition()) > FOLLOW_TARGET_DRIFT_SETTLE * FOLLOW_TARGET_DRIFT_SETTLE)
        return false;

    return true;
}

bool FollowMovementGenerator::Initialize(Unit* owner)
{
    RemoveFlag(MOVEMENTGENERATOR_FLAG_INITIALIZATION_PENDING | MOVEMENTGENERATOR_FLAG_DEACTIVATED);
    AddFlag(MOVEMENTGENERATOR_FLAG_INITIALIZED | MOVEMENTGENERATOR_FLAG_INFORM_ENABLED);

    UpdatePetSpeed(owner);
    _path = nullptr;
    _lastTargetPosition.reset();
    _launchTargetPosition.reset();
    // Pets and guardians of a player are throttled too: MoveSplineInit sends them the same stop packet before every new spline
    // (PlayerSideUnit), so a pet re-pathing on every step of its bot owner (a monster spline moves the owner every map update) was
    // shown sliding in its run pose, as if hovering (Karrz, Ironforge 2026-10-26: a bot hunter's wolf).
    Player const* playerOwner = owner->ToPlayer();
    _throttleRepath = (playerOwner && playerOwner->GetSession() && playerOwner->GetSession()->IsBot())
        || (!owner->IsPlayer() && owner->GetCharmerOrOwnerGUID().IsPlayer());
    return true;
}

bool FollowMovementGenerator::Reset(Unit* owner)
{
    RemoveFlag(MOVEMENTGENERATOR_FLAG_DEACTIVATED);

    return Initialize(owner);
}

bool FollowMovementGenerator::Update(Unit* owner, uint32 diff)
{
    // owner might be dead or gone
    if (!owner || !owner->IsAlive())
        return false;

    // our target might have gone away
    Unit* const target = GetTarget();
    if (!target || !target->IsInWorld())
        return false;

    if (_duration)
    {
        _duration->Update(diff);
        if (_duration->Passed())
        {
            owner->StopMoving();
            DoMovementInform(owner, target);
            return false;
        }
    }

    if (owner->HasUnitState(UNIT_STATE_NOT_MOVE) || owner->IsMovementPreventedByCasting())
    {
        _path = nullptr;
        owner->StopMoving();
        _lastTargetPosition.reset();
        _launchTargetPosition.reset();
        return true;
    }

    float range = _range;
    if (Creature* cOwner = owner->ToCreature())
        if (cOwner->IsIgnoringChaseRange())
            range = 0.0f;

    // AC parity (playerPet): while the leader is actively moving (movement flags from
    // the client), a follower inside follow range never stops - otherwise the discrete
    // position heartbeat makes the drift check read zero between heartbeats and the
    // follower stutter-stops every half second while the leader is running.
    bool const leaderMoving = target->m_movementInfo.HasMovementFlag(MOVEMENTFLAG_MASK_MOVING);

    _checkTimer.Update(diff);
    if (_checkTimer.Passed())
    {
        _checkTimer.Reset(CHECK_INTERVAL);
        if (HasFlag(MOVEMENTGENERATOR_FLAG_INFORM_ENABLED) && PositionOkay(owner, target, range, _angle, _launchTargetPosition) && !leaderMoving)
        {
            RemoveFlag(MOVEMENTGENERATOR_FLAG_INFORM_ENABLED);
            _path = nullptr;
            owner->StopMoving();
            // AC parity: a settled follower turns to face the same direction as its leader.
            owner->SetFacingTo(target->GetOrientation());
            _lastTargetPosition.reset();
            _launchTargetPosition.reset();
            DoMovementInform(owner, target);
            return true;
        }
    }

    if (owner->HasUnitState(UNIT_STATE_FOLLOW_MOVE) && owner->movespline->Finalized())
    {
        RemoveFlag(MOVEMENTGENERATOR_FLAG_INFORM_ENABLED);
        _path = nullptr;
        owner->ClearUnitState(UNIT_STATE_FOLLOW_MOVE);
        DoMovementInform(owner, target);
    }

    // Playerbot followers: while a follow spline is still in flight, re-path only once the
    // target has drifted FOLLOW_BOT_REPATH_DIST from where it stood at the last launch.
    // The stock rule re-paths and re-launches on ANY target movement, i.e. every map update
    // while the leader runs; with a raid of bot followers that is tens of SMSG_ON_MONSTER_MOVE
    // per bot per second to every viewer (and a pathfind each), which chokes the client.
    // _lastTargetPosition is left untouched while holding, so the drift accumulates; a
    // finished spline, a speed change (reset) or a large drift re-paths exactly as before.
    if (_throttleRepath && _lastTargetPosition && owner->HasUnitState(UNIT_STATE_FOLLOW_MOVE) && !owner->movespline->Finalized()
        && _lastTargetPosition->GetExactDistSq(target->GetPosition()) < FOLLOW_BOT_REPATH_DIST * FOLLOW_BOT_REPATH_DIST)
        return true;

    // AC parity: while the leader is actively moving, keep the follower's spline
    // continuously re-launched (no stand-still gap inside range+tolerance).
    if (!_lastTargetPosition || _lastTargetPosition->GetExactDistSq(target->GetPosition()) > 0.0f || target->isMoving())
    {
        _lastTargetPosition = target->GetPosition();
        if (owner->HasUnitState(UNIT_STATE_FOLLOW_MOVE) || !PositionOkay(owner, target, range + FOLLOW_RANGE_TOLERANCE, _angle, _launchTargetPosition) || target->isMoving())
        {
            if (!_path)
                _path = std::make_unique<PathGenerator>(owner);

            float x, y, z;

            // select angle
            float tAngle;
            float const curAngle = target->GetRelativeAngle(owner);
            if (!_angle || _angle->IsAngleOkay(curAngle))
                tAngle = curAngle;
            else
            {
                float const diffUpper = Position::NormalizeOrientation(curAngle - _angle->UpperBound());
                float const diffLower = Position::NormalizeOrientation(_angle->LowerBound() - curAngle);
                if (diffUpper < diffLower)
                    tAngle = _angle->UpperBound();
                else
                    tAngle = _angle->LowerBound();
            }

            target->GetNearPoint(owner, x, y, z, range, target->ToAbsoluteAngle(tAngle));

            // pets are allowed to "cheat" on pathfinding when following their master
            bool allowShortcut = false;
            if (Pet* oPet = owner->ToPet())
            {
                if (target->GetGUID() == oPet->GetOwnerGUID())
                    allowShortcut = true;
            }

            bool success = _path->CalculatePath(x, y, z, allowShortcut);
            if (!success || (_path->GetPathType() & PATHFIND_NOPATH))
            {
                owner->StopMoving();
                return true;
            }

            _launchTargetPosition = target->GetPosition();

            owner->AddUnitState(UNIT_STATE_FOLLOW_MOVE);
            AddFlag(MOVEMENTGENERATOR_FLAG_INFORM_ENABLED);

            Movement::MoveSplineInit init(owner);
            init.MovebyPath(_path->GetPath());
            if (!_ignoreTargetWalk)
                init.SetWalk(target->IsWalking());
            init.SetFacing(target->GetOrientation());
            init.Launch();
        }
    }
    return true;
}

void FollowMovementGenerator::Deactivate(Unit* owner)
{
    AddFlag(MOVEMENTGENERATOR_FLAG_DEACTIVATED);
    RemoveFlag(MOVEMENTGENERATOR_FLAG_TRANSITORY | MOVEMENTGENERATOR_FLAG_INFORM_ENABLED);
    owner->ClearUnitState(UNIT_STATE_FOLLOW_MOVE);
}

void FollowMovementGenerator::Finalize(Unit* owner, bool active, bool movementInform)
{
    AddFlag(MOVEMENTGENERATOR_FLAG_FINALIZED);
    if (active)
    {
        owner->ClearUnitState(UNIT_STATE_FOLLOW_MOVE);
        UpdatePetSpeed(owner);
        if (movementInform)
            SetScriptResult(MovementStopReason::Finished);
    }
}

void FollowMovementGenerator::UpdatePetSpeed(Unit* owner)
{
    if (Pet* oPet = owner->ToPet())
    {
        if (!GetTarget() || GetTarget()->GetGUID() == owner->GetOwnerGUID())
        {
            oPet->UpdateSpeed(MOVE_RUN);
            oPet->UpdateSpeed(MOVE_WALK);
            oPet->UpdateSpeed(MOVE_SWIM);
        }
    }
}
