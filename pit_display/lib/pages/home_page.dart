import 'package:flutter/material.dart';
import 'package:pit_display/pages/event_page.dart';
import 'package:pit_display/pages/systems_page.dart';
import 'package:shared_preferences/shared_preferences.dart';

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  int _selectedIndex = 0;
  late PageController _pageController;
  late SharedPreferences _prefs;

  @override
  void initState() {
    super.initState();

    _pageController = PageController(initialPage: _selectedIndex);

    SharedPreferences.getInstance().then((value) => _prefs = value);
  }

  @override
  void dispose() {
    _pageController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Focus(
        skipTraversal: true,
        canRequestFocus: false,
        descendantsAreFocusable: false,
        descendantsAreTraversable: false,
        child: Row(
          children: [
            Stack(
              children: [
                NavigationRail(
                  selectedIndex: _selectedIndex,
                  backgroundColor: Colors.black.withOpacity(0.15),
                  labelType: NavigationRailLabelType.all,
                  elevation: 8,
                  onDestinationSelected: (int index) {
                    setState(() {
                      _selectedIndex = index;
                      _pageController.animateToPage(_selectedIndex,
                          duration: const Duration(milliseconds: 200),
                          curve: Curves.easeInOut);
                    });
                  },
                  destinations: const [
                    NavigationRailDestination(
                      icon: Icon(Icons.space_dashboard_outlined),
                      selectedIcon: Icon(Icons.space_dashboard),
                      label: Text('Event'),
                    ),
                    NavigationRailDestination(
                      icon: Icon(Icons.fact_check_outlined),
                      selectedIcon: Icon(Icons.fact_check),
                      label: Text('Systems'),
                    ),
                  ],
                ),
                Positioned(
                  bottom: 8,
                  left: 16,
                  child: IconButton(
                    iconSize: 28,
                    icon: const Icon(Icons.settings_rounded),
                    onPressed: () {
                      showDialog(
                        context: context,
                        builder: (context) {
                          return AlertDialog(
                            title: const Text('Settings'),
                            content: Column(
                              mainAxisSize: MainAxisSize.min,
                              children: [
                                TextFormField(
                                  decoration: const InputDecoration(
                                    border: OutlineInputBorder(),
                                    labelText: 'Twitch Channel',
                                  ),
                                  initialValue:
                                      _prefs.getString('twitchChannel'),
                                  onFieldSubmitted: ((value) {
                                    _prefs.setString('twitchChannel', value);
                                  }),
                                ),
                                const SizedBox(height: 16),
                                TextFormField(
                                  decoration: const InputDecoration(
                                    border: OutlineInputBorder(),
                                    labelText: 'Team Number',
                                  ),
                                  initialValue:
                                      _prefs.getInt('teamNumber')?.toString(),
                                  onFieldSubmitted: ((value) {
                                    int? number = int.tryParse(value);
                                    if (number != null) {
                                      _prefs.setInt('teamNumber', number);
                                    }
                                  }),
                                ),
                                const SizedBox(height: 16),
                                TextFormField(
                                  decoration: const InputDecoration(
                                    border: OutlineInputBorder(),
                                    labelText: 'Event Key',
                                  ),
                                  initialValue: _prefs.getString('eventKey'),
                                  onFieldSubmitted: ((value) {
                                    _prefs.setString('eventKey', value);
                                  }),
                                ),
                              ],
                            ),
                            actions: [
                              TextButton(
                                onPressed: () {
                                  Navigator.of(context).pop();
                                },
                                child: const Text('Close'),
                              ),
                            ],
                          );
                        },
                      );
                    },
                  ),
                ),
              ],
            ),
            Expanded(
              child: PageView(
                controller: _pageController,
                physics: const NeverScrollableScrollPhysics(),
                onPageChanged: (int index) {
                  setState(() {
                    _selectedIndex = index;
                  });
                },
                children: const [
                  EventPage(),
                  SystemsPage(),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
