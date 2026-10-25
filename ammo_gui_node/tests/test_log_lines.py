"""Check the bridge's regexes against the real text in the mission nodes.

The bridge reads a handful of lines out of `/rosout` to turn them into
events - a ford crossing, a stall, a spot where the gripper found nothing.
Every one of those is a line the node already wrote; none were added for the
GUI. If somebody rewords one upstream, the matching event quietly stops
appearing rather than anything breaking, which is exactly the kind of fault
nobody notices until a demo.

So this test does not run the bridge at all. It looks in the mission nodes'
source for the fixed part of each line and fails if it has gone. The list is
the same one in `integration/NOTES.md`.
"""
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: F401,E402  (sets sys.path and the headless Qt platform)

from ammo_gui_node import mission_bridge as mb

SOURCES = {
    'rover': ('quad_zcams', 'quad_zcams', 'rover_capture_node.py'),
    'cage': ('ammo_mission_control', 'ammo_mission_control', 'cage_capture_node.py'),
    'quad': ('quad_zcams', 'quad_zcams', 'zcam_quad_node.py'),
}

# regex name in mission_bridge -> the literal fragment that must survive
FRAGMENTS = {
    'RE_STATE': "f'-> {s}'",
    'RE_QUEUED': 'Waypoint queued (',
    'RE_COLLECTED': 'Collected {',
    'RE_GAVE_UP': 'Given up after repeated deferrals',
    'RE_DEFER': 'Deferring to the back of the queue (attempt',
    'RE_LEFT': 'waypoint(s) left',
    'RE_FORD': '- driving the lane across myself',
    'RE_FORD_FAIL': 'crossing timed out at',
    'RE_WEDGED': 'Boxed in at',
    'RE_STALL': 'Nav2 leg stalled: moved <',
    'RE_EMPTY': '- later handoffs within',
    'RE_GAVE_UP_HERE': '- later routes avoid it',
    'RE_REROUTE': "No route on the quad's map - routing on",
    'RE_NO_ROUTE': "on the quad's map around",
    'RE_RETRY': 'Queue empty - retrying the',
    'RE_UNREACHABLE': 'shows no clear side of',
    'RE_RIVER': 'River blocked on the route map:',
    'RE_NAV_TO': 'via point(s)',
    'RE_COVERAGE': '% of the area seen',
    'RE_NATIVE': 'native mushroom {',
}

# the publishers added to the mission nodes, which the bridge prefers
PUBLISHERS = {
    'cage': ["'husky1/rover/state'", 'self.state_pub.publish'],
    'quad': ["'parrot1/quad_state'", 'def _publish_state'],
    'rover': ['self.state_pub.publish'],
}


def main():
    text = {}
    missing_files = []
    for key, parts in SOURCES.items():
        path = os.path.join(_paths.REPO, *parts)
        if not os.path.exists(path):
            missing_files.append(path)
            text[key] = ''
            continue
        text[key] = io.open(path, encoding='utf-8').read()
    joined = '\n'.join(text.values())

    bad = []
    for path in missing_files:
        bad.append(f'source not found: {path}')

    for name, fragment in sorted(FRAGMENTS.items()):
        if not hasattr(mb, name):
            bad.append(f'{name}: the bridge no longer defines this regex')
        elif fragment not in joined:
            bad.append(f'{name}: no mission node still writes {fragment!r}')
        else:
            print(f'  ok   {name:18} <- {fragment}')

    for key, needles in sorted(PUBLISHERS.items()):
        for needle in needles:
            if needle not in text.get(key, ''):
                bad.append(f'{key}: the state publisher is missing {needle!r}')
            else:
                print(f'  ok   {key:18} <- {needle}')

    total = len(FRAGMENTS) + sum(len(v) for v in PUBLISHERS.values())
    print(f'\n{total - len(bad)} / {total} anchored in the real source')
    for line in bad:
        print('  FAIL', line)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
