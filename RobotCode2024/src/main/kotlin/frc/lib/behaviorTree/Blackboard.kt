package frc.lib.behaviorTree

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d

class Blackboard {
  private val variables: MutableMap<String, Any> = HashMap()

  fun setVariable(key: String, value: Any) {
    variables[key] = value
  }

  fun getVariable(key: String): Any? {
    return variables[key]
  }

  fun getInt(key: String): Int? {
    val value = getVariable(key)

    return if (value is Int) value else null
  }

  fun getDouble(key: String): Double? {
    val value = getVariable(key)

    return if (value is Double) value else null
  }

  fun getBoolean(key: String): Boolean? {
    val value = getVariable(key)

    return if (value is Boolean) value else null
  }

  fun getString(key: String): String? {
    val value = getVariable(key)

    return if (value is String) value else null
  }

  fun getTranslation2d(key: String): Translation2d? {
    val value = getVariable(key)

    return if (value is Translation2d) value else null
  }

  fun getRotation2d(key: String): Rotation2d? {
    val value = getVariable(key)

    return if (value is Rotation2d) value else null
  }

  fun getTranslation3d(key: String): Translation3d? {
    val value = getVariable(key)

    return if (value is Translation3d) value else null
  }

  fun getRotation3d(key: String): Rotation3d? {
    val value = getVariable(key)

    return if (value is Rotation3d) value else null
  }

  fun getPose2d(key: String): Pose2d? {
    val value = getVariable(key)

    return if (value is Pose2d) value else null
  }

  fun getPose3d(key: String): Pose3d? {
    val value = getVariable(key)

    return if (value is Pose3d) value else null
  }

  fun isVariableSet(key: String): Boolean {
    return getVariable(key) != null
  }

  fun clearVariable(key: String) {
    variables.remove(key)
  }

  fun clear() {
    variables.clear()
  }
}
