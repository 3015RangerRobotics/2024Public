enum Outcome {
  redWin,
  blueWin,
  tie,
}

class Match {
  final String matchNumber;
  final List<int> redTeams;
  final List<int> blueTeams;
  final bool? weAreRed;

  const Match({
    required this.matchNumber,
    required this.redTeams,
    required this.blueTeams,
    this.weAreRed,
  });
}

class UpcomingMatch extends Match {
  final DateTime estimatedStartTime;

  const UpcomingMatch({
    required super.matchNumber,
    required super.redTeams,
    required super.blueTeams,
    required this.estimatedStartTime,
    super.weAreRed,
  });
}

class FinishedMatch extends Match {
  final Outcome outcome;
  final int redScore;
  final int blueScore;
  final int redRP;
  final int blueRP;

  const FinishedMatch({
    required super.matchNumber,
    required super.redTeams,
    required super.blueTeams,
    required this.outcome,
    required this.redScore,
    required this.blueScore,
    required this.redRP,
    required this.blueRP,
    super.weAreRed,
  });
}
