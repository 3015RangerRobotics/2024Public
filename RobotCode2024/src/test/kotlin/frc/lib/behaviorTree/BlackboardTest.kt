package frc.lib.behaviorTree

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class BlackboardTest {
  @Test
  fun testVariable() {
    val blackboard = Blackboard()

    blackboard.setVariable("test", "wow")

    Assertions.assertTrue(blackboard.isVariableSet("test"))
    Assertions.assertEquals("wow", blackboard.getVariable("test"))

    blackboard.clearVariable("test")

    Assertions.assertFalse(blackboard.isVariableSet("test"))

    blackboard.setVariable("test", "wow")

    blackboard.clear()

    Assertions.assertFalse(blackboard.isVariableSet("test"))
  }

  @Test
  fun testTyped() {
    val blackboard = Blackboard()

    blackboard.setVariable("int", 10)
    Assertions.assertEquals(10, blackboard.getInt("int"))

    blackboard.setVariable("double", 1.5)
    Assertions.assertEquals(1.5, blackboard.getDouble("double"))

    blackboard.setVariable("boolean", true)
    Assertions.assertEquals(true, blackboard.getBoolean("boolean"))

    blackboard.setVariable("string", "test")
    Assertions.assertEquals("test", blackboard.getString("string"))

    blackboard.setVariable("translation2d", Translation2d(1.0, 2.0))
    Assertions.assertEquals(Translation2d(1.0, 2.0), blackboard.getTranslation2d("translation2d"))

    blackboard.setVariable("rotation2d", Rotation2d(1.0))
    Assertions.assertEquals(Rotation2d(1.0), blackboard.getRotation2d("rotation2d"))

    blackboard.setVariable("translation3d", Translation3d(1.0, 2.0, 3.0))
    Assertions.assertEquals(
        Translation3d(1.0, 2.0, 3.0), blackboard.getTranslation3d("translation3d"))

    blackboard.setVariable("rotation3d", Rotation3d(1.0, 2.0, 3.0))
    Assertions.assertEquals(Rotation3d(1.0, 2.0, 3.0), blackboard.getRotation3d("rotation3d"))

    blackboard.setVariable("pose2d", Pose2d(1.0, 2.0, Rotation2d()))
    Assertions.assertEquals(Pose2d(1.0, 2.0, Rotation2d()), blackboard.getPose2d("pose2d"))

    blackboard.setVariable("pose3d", Pose3d(1.0, 2.0, 3.0, Rotation3d()))
    Assertions.assertEquals(Pose3d(1.0, 2.0, 3.0, Rotation3d()), blackboard.getPose3d("pose3d"))
  }
}
