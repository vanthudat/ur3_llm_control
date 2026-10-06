"""Camera scene validation and deterministic clearance planning (no joint commands)."""
import math
from copy import deepcopy
from .task_validator import ALLOWED_OBJECTS, ALLOWED_ZONES, PlanValidationError, validate_plan

ZONES = {'zone_a': [0.40, 0.14, 0.740], 'zone_b': [0.40, 0.0, 0.740],
         'zone_c': [0.40, -0.14, 0.740]}
# A broad grid of candidate centers across the table. Availability is tested
# against the latest camera observation before a candidate is used.
TEMPORARY = {
    f'temp_{i}': [x, y, 0.740]
    for i, (x, y) in enumerate(
        (x, y) for x in (0.22, 0.26, 0.30, 0.34, 0.38)
        for y in (-0.27, -0.24, 0.24, 0.27))
}
DESTINATIONS = {**ZONES, **TEMPORARY}
CLEARANCE = 0.095  # cube half-size + gripper envelope + margin
TEMPORARY_CLEARANCE = 0.12  # extra room for camera position error and transfer motion


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def validate_scene(scene, now, max_age=2.0):
    """Check capture/processing age in wall time, independently of ROS /clock delivery.

    ``stamp`` remains the Gazebo frame timestamp; ``now`` and ``observed_at``
    are system-clock seconds shared by nodes on the same machine.
    """
    if not isinstance(scene, dict) or scene.get('source') != 'rgbd_camera':
        raise PlanValidationError('No RGB-D camera observation')
    stamp = scene.get('stamp')
    if type(stamp) not in (float, int) or not math.isfinite(stamp) or stamp < 0:
        raise PlanValidationError('Camera has an invalid frame timestamp')
    observed_at = scene.get('observed_at')
    if type(observed_at) not in (float, int) or not math.isfinite(observed_at):
        raise PlanValidationError('Camera timing metadata missing; rebuild and restart camera_state')
    age = now - observed_at
    if not math.isfinite(age) or not -0.25 <= age <= max_age:
        raise PlanValidationError(f'Camera frame is not fresh (wall-clock age={age:.2f}s)')
    objects = scene.get('objects', {})
    if not isinstance(objects, dict):
        raise PlanValidationError('Camera observation has an invalid objects field')
    missing = sorted(ALLOWED_OBJECTS - set(objects))
    if missing:
        raise PlanValidationError(
            'Camera did not detect: ' + ', '.join(missing) +
            '; inspect /table_camera/detections for occlusion or color threshold issues')
    if set(objects) - ALLOWED_OBJECTS:
        raise PlanValidationError('Camera observation contains unknown object names')
    for name, p in objects.items():
        if not isinstance(p, list) or len(p) != 3 or not all(
                type(v) in (int, float) and math.isfinite(v) for v in p):
            raise PlanValidationError(f'Invalid observed position: {name}')
        if not (0.10 <= p[0] <= 0.58 and abs(p[1]) <= 0.28 and abs(p[2]-0.740) <= 0.040):
            raise PlanValidationError(f'Block outside supported table workspace: {name}')
    return deepcopy(objects)


def occupants(positions, target, exclude=None):
    return [name for name, p in positions.items() if name != exclude and distance(p, target) < CLEARANCE]


def expand_clearance_plan(intent, scene, now):
    """Expand LLM pick/place intentions using observed occupancy; simulate every move."""
    intent = validate_plan(intent)
    positions = validate_scene(scene, now)
    steps = intent['plan']
    result = []
    i = 0
    while i < len(steps):
        step = steps[i]
        if step['skill'] != 'pick':
            if step['skill'] not in {'home', 'move_above', 'move_to_zone', 'open_gripper', 'close_gripper'}:
                raise PlanValidationError('Expected a pick/place pair')
            result.append(step)
            i += 1
            continue
        if i+1 >= len(steps) or steps[i+1]['skill'] != 'place':
            raise PlanValidationError('Bai 03 requires adjacent pick/place pairs')
        place = steps[i+1]
        name, zone = step['object'], place['zone']
        if zone not in ALLOWED_ZONES:
            raise PlanValidationError('Only environment logic selects temporary destinations')
        target = DESTINATIONS[zone]
        if distance(positions[name], target) < 0.025:
            i += 2
            continue
        for blocker in occupants(positions, target, name):
            free = next((slot for slot, point in TEMPORARY.items()
                         if not any(distance(p, point) < TEMPORARY_CLEARANCE
                                    for p in positions.values()) and
                         all(distance(point, z) >= TEMPORARY_CLEARANCE
                             for z in ZONES.values())), None)
            if free is None:
                raise PlanValidationError('No observed free temporary position')
            result.extend([{'skill': 'pick', 'object': blocker},
                           {'skill': 'place', 'object': blocker, 'zone': free}])
            positions[blocker] = DESTINATIONS[free][:]
        result.extend([step, place])
        positions[name] = target[:]
        i += 2
    if not result or result[-1]['skill'] != 'home':
        result.append({'skill': 'home'})
    expanded = validate_plan({'plan': result})
    validate_environment_plan(expanded, scene, now)
    return expanded


def validate_environment_plan(plan, scene, now):
    positions = validate_scene(scene, now)
    held = None
    for step in validate_plan(plan)['plan']:
        if step['skill'] == 'pick':
            held = step['object']
        elif step['skill'] == 'place':
            target = DESTINATIONS[step['zone']]
            if occupants(positions, target, held):
                raise PlanValidationError(f"Destination {step['zone']} is occupied")
            positions[held] = target[:]
            held = None
        elif held is not None:
            raise PlanValidationError('Only place is allowed while holding a block')
    return plan
