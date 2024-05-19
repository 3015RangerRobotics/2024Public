package frc.lib.behaviorTree

import org.littletonrobotics.junction.Logger

class BehaviorTreeDebug {
  companion object Companion {
    private val activeUUIDs: MutableSet<String> = HashSet()

    fun setActive(uuid: String) = activeUUIDs.add(uuid)

    fun setInactive(uuid: String) = activeUUIDs.remove(uuid)

    fun pushActive() = Logger.recordOutput("BehaviorTree/ActiveNodes", activeUUIDs.toTypedArray())
  }
}
