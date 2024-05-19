package frc.lib.behaviorTree

enum class ExecutionStatus {
  Running,
  Success,
  Failure;

  fun invert(): ExecutionStatus {
    return when (this) {
      Success -> Failure
      Failure -> Success
      else -> this
    }
  }
}
