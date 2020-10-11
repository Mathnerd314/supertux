//  SuperTux
//  Copyright (C) 2006 Matthias Braun <matze@braunis.de>
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "badguy/flyingsnowball.hpp"

#include "math/random.hpp"
#include "math/util.hpp"
#include "object/sprite_particle.hpp"
#include "object/player.hpp"
#include "sprite/sprite.hpp"
#include "supertux/sector.hpp"

namespace {
const float PUFF_INTERVAL_MIN = 4.0f; /**< spawn new puff of smoke at most that often */
const float PUFF_INTERVAL_MAX = 8.0f; /**< spawn new puff of smoke at least that often */

const float FLY_HEIGHT = 96.0f; // 4 * 32 / (4/3)
// in milestone1: it goes up for 1s at speed 100 px/s, then down similarly, so total travel was 100px.
// in 0.3.x the travel was changed to 4*32 + extra. The extra was usually around half a tile or less
// but there was a random boost that could make the snowball fly up an extra tile or two
// here we use exactly 4*32=128 so that level design is easier
// our chosen function height height has range [-2/3,2/3] hence 4/3

const float GLOBAL_SPEED_MULT = 0.883883476483184f; /**< the overall movement speed multiplier k, f(k*t) */
// we can look at this 3 ways:
// 1. position - the time to get from bottom to top
// - pi in total, but 1.91 if we exclude the direction reversal
// to scale this to 1s we would use k = pi or k = 1.91
// 2. velocity - our function's max speed is 0.968 and avg speed is 0.518, unscaled
// we scale like FLY_HEIGHT * k * unscaled speed = scaled speed
// so to get 100px/s speed like milestone1 we would use mult = 1.0764359221215134 (max) or 2.0081203041412046 (avg)
// 3. acceleration - it was gravity * 0.2 at the extremes, in 0.3.x. gravity is 10 but scaled to 1000, so a=200
// we scale like FLY_HEIGHT * k^2 * unscaled accel = scaled accel
// our function has max acceleration 8/3 unscaled. So we use mult = 0.883883476483184

const float OFFSET_MULT = -1.0f / (2.f * 32.f * math::TAU);
// Tux travels right at 320 px/s running / 400 px/s air
// his jump follows v t + .5 a t (t+dt_sec)
// v is 620 (air) or 580 (run), a is -1000
// air we hit at th=1.224375, run we hit at th=1.144375
// so jump length is jx=489.75px air, jx=366.2px run
// we want each of Tux's jumps to be at the same height.
// so one of the maxima of our function at t = th*k should be within +- 32px of x= jx * k

// we work with cos(o*(jx*k+f)+g*(th*k)). the cos is so the maximum is at 0
// the f is the 32px fudge factor. the k can be factored out, o*f+(o*jx+g*th)*k
// we need integer values of k to be maxima, so o*jx+g*th = 2*pi*m for integer m
// to be continued...
}




FlyingSnowBall::FlyingSnowBall(const ReaderMapping& reader) :
  BadGuy(reader, "images/creatures/flying_snowball/flying_snowball.sprite"),
  start_time(g_game_time),
  puff_timer()
{
  m_physic.enable_gravity(false);
}

void
FlyingSnowBall::initialize()
{
  m_sprite->set_action(m_dir == Direction::LEFT ? "left" : "right");
}

void
FlyingSnowBall::activate()
{
  puff_timer.start(static_cast<float>(gameRandom.randf(PUFF_INTERVAL_MIN, PUFF_INTERVAL_MAX)));
}

bool
FlyingSnowBall::collision_squished(GameObject& object)
{
  m_sprite->set_action(m_dir == Direction::LEFT ? "squished-left" : "squished-right");
  m_physic.enable_gravity(true);
  m_physic.set_acceleration_y(0);
  m_physic.set_velocity_y(0);
  kill_squished(object);
  return true;
}

void
FlyingSnowBall::collision_solid(const CollisionHit& hit)
{
  if (hit.top || hit.bottom) {
    m_physic.set_velocity_y(0);
  }
}

void
FlyingSnowBall::active_update(float dt_sec)
{
  float t = GLOBAL_SPEED_MULT * (g_game_time - start_time);
  float x = OFFSET_MULT * m_start_position.x;
  // sum-expanded version of sin(t+x)
  float s = sinf(t) * cosf(x) + cosf(t) * sin(x);
  // graph sin(x)^3 - sin(x)/3 -- it is a wavy curve
  float targetHgt = std::pow(s, 3.f) - s / 3.f;
  targetHgt = targetHgt * FLY_HEIGHT + m_start_position.y;
  m_col.m_movement = Vector(0, targetHgt - get_pos().y);

  auto player = get_nearest_player();
  if (player) {
    m_dir = (player->get_pos().x > get_pos().x) ? Direction::RIGHT : Direction::LEFT;
    m_sprite->set_action(m_dir == Direction::LEFT ? "left" : "right");
  }

  // spawn smoke puffs
  if (puff_timer.check()) {
    Vector ppos = m_col.m_bbox.get_middle();
    Vector pspeed = Vector(gameRandom.randf(-10, 10), 150);
    Vector paccel = Vector(0,0);
    Sector::get().add<SpriteParticle>("images/particles/smoke.sprite",
                                           "default",
                                           ppos, ANCHOR_MIDDLE, pspeed, paccel,
                                           LAYER_OBJECTS-1);
    puff_timer.start(gameRandom.randf(PUFF_INTERVAL_MIN, PUFF_INTERVAL_MAX));
  }

}

/* EOF */
