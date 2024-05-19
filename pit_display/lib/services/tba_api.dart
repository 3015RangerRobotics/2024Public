import 'dart:convert';

import 'package:http/http.dart' as http;
import 'package:pit_display/model/match.dart';
import 'package:shared_preferences/shared_preferences.dart';

class TBA {
  static const _apiKey =
      '79z8fUgi0DztS0TNFnKd9h5ykVImqwGSjumHHqQWo3aAyy6VMU82ruvExfMhFmUd';

  static SharedPreferences? _prefs;

  static Future<Map<String, dynamic>?> getTeamRankingData() async {
    _prefs ??= await SharedPreferences.getInstance();

    int team = _prefs?.getInt('teamNumber') ?? 0;
    String event = _prefs?.getString('eventKey') ?? '';

    var url = Uri.parse(
        'https://www.thebluealliance.com/api/v3/team/frc$team/event/$event/status');
    var response = await http.get(url,
        headers: {'accept': 'application/json', 'X-TBA-Auth-Key': _apiKey});

    if (response.body == 'null') {
      return null;
    }

    Map<String, dynamic> responseJson = jsonDecode(response.body);

    int playoffW = 0;
    int playoffL = 0;
    int playoffT = 0;

    if (responseJson['playoff'] != null) {
      playoffW = (responseJson['playoff']['record']['wins'] as num).toInt();
      playoffL = (responseJson['playoff']['record']['losses'] as num).toInt();
      playoffT = (responseJson['playoff']['record']['ties'] as num).toInt();
    }

    return {
      'rank': (responseJson['qual']['ranking']['rank'] as num).toInt(),
      'rankingScore':
          (responseJson['qual']['ranking']['sort_orders'][0] as num).toDouble(),
      'avgCoop':
          (responseJson['qual']['ranking']['sort_orders'][1] as num).toDouble(),
      'avgMatch':
          (responseJson['qual']['ranking']['sort_orders'][2] as num).toDouble(),
      'avgAuto':
          (responseJson['qual']['ranking']['sort_orders'][3] as num).toDouble(),
      'avgStage':
          (responseJson['qual']['ranking']['sort_orders'][4] as num).toDouble(),
      'wins':
          (responseJson['qual']['ranking']['record']['wins'] as num).toInt() +
              playoffW,
      'losses':
          (responseJson['qual']['ranking']['record']['losses'] as num).toInt() +
              playoffL,
      'ties':
          (responseJson['qual']['ranking']['record']['ties'] as num).toInt() +
              playoffT,
    };
  }

  static Future<List<Match>?> getTeamMatchSchedule() async {
    _prefs ??= await SharedPreferences.getInstance();

    int team = _prefs?.getInt('teamNumber') ?? 0;
    String event = _prefs?.getString('eventKey') ?? '';

    var url = Uri.parse(
        'https://www.thebluealliance.com/api/v3/team/frc$team/event/$event/matches');
    var response = await http.get(url,
        headers: {'accept': 'application/json', 'X-TBA-Auth-Key': _apiKey});

    if (response.body == 'null') {
      return null;
    }

    List<dynamic> responseLst = jsonDecode(response.body);
    List<Map<String, dynamic>> responseJson = responseLst.cast();
    responseJson.sort((a, b) {
      return (a['predicted_time'] as num)
          .compareTo((b['predicted_time'] as num));
    });

    List<Match> matches = [];
    for (Map<String, dynamic> matchJson in responseJson) {
      String compLevel = matchJson['comp_level'];
      compLevel = compLevel.toUpperCase();
      if (compLevel == 'QM') {
        compLevel = 'Q';
      }

      String matchNum = matchJson['match_number'].toString();
      if (compLevel != 'Q') {
        matchNum += '-${matchJson['set_number']}';
      }

      List<String> redTeamsStr =
          (matchJson['alliances']['red']['team_keys'] as List<dynamic>).cast();
      List<int> redTeams = redTeamsStr
          .map((teamStr) => int.parse(teamStr.substring(3)))
          .toList();
      List<String> blueTeamsStr =
          (matchJson['alliances']['blue']['team_keys'] as List<dynamic>).cast();
      List<int> blueTeams = blueTeamsStr
          .map((teamStr) => int.parse(teamStr.substring(3)))
          .toList();

      if (matchJson['actual_time'] != null) {
        Outcome outcome = Outcome.tie;
        if (matchJson['winning_alliance'] == 'red') {
          outcome = Outcome.redWin;
        } else if (matchJson['winning_alliance'] == 'blue') {
          outcome = Outcome.blueWin;
        }

        matches.add(FinishedMatch(
          matchNumber: '$compLevel$matchNum',
          redTeams: redTeams,
          blueTeams: blueTeams,
          outcome: outcome,
          redScore: (matchJson['alliances']['red']['score'] as num).toInt(),
          blueScore: (matchJson['alliances']['blue']['score'] as num).toInt(),
          redRP: (matchJson['score_breakdown']['red']['rp'] as num).toInt(),
          blueRP: (matchJson['score_breakdown']['blue']['rp'] as num).toInt(),
          weAreRed: redTeams.contains(team),
        ));
      } else {
        matches.add(UpcomingMatch(
          matchNumber: '$compLevel$matchNum',
          redTeams: redTeams,
          blueTeams: blueTeams,
          estimatedStartTime: DateTime.fromMillisecondsSinceEpoch(
              (matchJson['predicted_time'] as num).toInt() * 1000),
          weAreRed: redTeams.contains(team),
        ));
      }
    }

    return matches;
  }

  static Future<List<Match>?> getShortEventMatchSchedule() async {
    _prefs ??= await SharedPreferences.getInstance();

    String event = _prefs?.getString('eventKey') ?? '';

    var url = Uri.parse(
        'https://www.thebluealliance.com/api/v3/event/$event/matches');
    var response = await http.get(url,
        headers: {'accept': 'application/json', 'X-TBA-Auth-Key': _apiKey});

    if (response.body == 'null') {
      return null;
    }

    List<dynamic> responseLst = jsonDecode(response.body);
    List<Map<String, dynamic>> responseJson = responseLst.cast();
    responseJson.sort((a, b) {
      return (a['predicted_time'] as num)
          .compareTo((b['predicted_time'] as num));
    });

    int prevMatchIdx = responseJson.length - 4;
    for (int i = prevMatchIdx; i >= 0; i--) {
      if (responseJson[i]['actual_time'] != null) {
        prevMatchIdx = i;
        break;
      }
    }

    List<Match> matches = [];
    for (int i = prevMatchIdx; i < prevMatchIdx + 4; i++) {
      if (i >= responseJson.length) {
        break;
      }

      Map<String, dynamic> matchJson = responseJson[i];

      String compLevel = matchJson['comp_level'];
      compLevel = compLevel.toUpperCase();
      if (compLevel == 'QM') {
        compLevel = 'Q';
      }

      String matchNum = matchJson['match_number'].toString();
      if (compLevel != 'Q') {
        matchNum += '-${matchJson['set_number']}';
      }

      List<String> redTeamsStr =
          (matchJson['alliances']['red']['team_keys'] as List<dynamic>).cast();
      List<int> redTeams = redTeamsStr
          .map((teamStr) => int.parse(teamStr.substring(3)))
          .toList();
      List<String> blueTeamsStr =
          (matchJson['alliances']['blue']['team_keys'] as List<dynamic>).cast();
      List<int> blueTeams = blueTeamsStr
          .map((teamStr) => int.parse(teamStr.substring(3)))
          .toList();

      if (matchJson['actual_time'] != null) {
        Outcome outcome = Outcome.tie;
        if (matchJson['winning_alliance'] == 'red') {
          outcome = Outcome.redWin;
        } else if (matchJson['winning_alliance'] == 'blue') {
          outcome = Outcome.blueWin;
        }

        matches.add(FinishedMatch(
          matchNumber: '$compLevel$matchNum',
          redTeams: redTeams,
          blueTeams: blueTeams,
          outcome: outcome,
          redScore: (matchJson['alliances']['red']['score'] as num).toInt(),
          blueScore: (matchJson['alliances']['blue']['score'] as num).toInt(),
          redRP: (matchJson['score_breakdown']['red']['rp'] as num).toInt(),
          blueRP: (matchJson['score_breakdown']['blue']['rp'] as num).toInt(),
        ));
      } else {
        matches.add(UpcomingMatch(
          matchNumber: '$compLevel$matchNum',
          redTeams: redTeams,
          blueTeams: blueTeams,
          estimatedStartTime: DateTime.fromMillisecondsSinceEpoch(
              (matchJson['predicted_time'] as num).toInt() * 1000),
        ));
      }
    }

    return matches;
  }
}
