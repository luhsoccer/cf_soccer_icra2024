#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Duplicate of BlockEnemyLineOfSight, just with a variable radius
 *
 * The skill is endless and uses 2 features, a line and a circle. The line is
 * from enemy through ball and the the circle is around the enemy, the radius is
 * defined by a required_double. The weight of the circle is defined by a
 * config parameter.
 *
 * If the line from enemy through ball crosses the circle feature outside of the
 * field, the line is projected into the field. This guarantees the robot to
 * stay inside the field margins. If this occurs at a goalline, the line is
 * projected onto the ally goal centre. If this occurd at a touchline, the line
 * is rotated back onto the touchline, closest to the original LoS. If this
 * occurs at a corner, the goalline case takes effect.

 *
 * @p related_robot EnemyBallCarrier<br>
 * @p required_point <br>
 * @p required_double radius<br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class BlockEnemyLosVariableBuild : public SkillBuilder
{
   public:
    BlockEnemyLosVariableBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills