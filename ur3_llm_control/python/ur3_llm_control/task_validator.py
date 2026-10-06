"""Strict validation for plans returned by the LLM."""

from __future__ import annotations

import json
from typing import Any


ALLOWED_OBJECTS = frozenset({"red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"})
TEMPORARY_ZONES = frozenset(f"temp_{i}" for i in range(20))
ALLOWED_ZONES = frozenset({"zone_a", "zone_b", "zone_c"})
SKILL_FIELDS = {
    "home": frozenset({"skill"}),
    "move_above": frozenset({"skill", "object"}),
    "open_gripper": frozenset({"skill"}),
    "close_gripper": frozenset({"skill"}),
    "pick": frozenset({"skill", "object"}),
    "place": frozenset({"skill", "object", "zone"}),
    "move_to_zone": frozenset({"skill", "zone"}),
}
MAX_PLAN_STEPS = 40


class PlanValidationError(ValueError):
    """Raised when an LLM response is not an executable robot plan."""


def extract_json_object(content: str) -> dict[str, Any]:
    """Parse one JSON object, accepting an optional Markdown code fence."""
    text = content.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if len(lines) < 3 or not lines[-1].strip().startswith("```"):
            raise PlanValidationError("JSON code fence is incomplete")
        text = "\n".join(lines[1:-1]).strip()

    try:
        value = json.loads(text)
    except json.JSONDecodeError as error:
        raise PlanValidationError(f"LLM response is not valid JSON: {error.msg}") from error
    if not isinstance(value, dict):
        raise PlanValidationError("LLM response must be a JSON object")
    return value


def validate_plan(value: Any) -> dict[str, list[dict[str, str]]]:
    """Return a normalized plan or raise PlanValidationError."""
    if not isinstance(value, dict) or set(value) != {"plan"}:
        raise PlanValidationError("Top level must contain only the 'plan' field")

    steps = value["plan"]
    if not isinstance(steps, list) or not steps:
        raise PlanValidationError("'plan' must be a non-empty list")
    if len(steps) > MAX_PLAN_STEPS:
        raise PlanValidationError(f"Plan has more than {MAX_PLAN_STEPS} steps")

    normalized: list[dict[str, str]] = []
    held_object: str | None = None
    for index, raw_step in enumerate(steps):
        label = f"plan[{index}]"
        if not isinstance(raw_step, dict):
            raise PlanValidationError(f"{label} must be an object")

        skill = raw_step.get("skill")
        if not isinstance(skill, str) or skill not in SKILL_FIELDS:
            raise PlanValidationError(f"{label} contains unsupported skill {skill!r}")
        if set(raw_step) != SKILL_FIELDS[skill]:
            raise PlanValidationError(f"{label} has missing or unexpected fields")
        if not all(isinstance(item, str) for item in raw_step.values()):
            raise PlanValidationError(f"{label} fields must be strings")

        object_name = raw_step.get("object")
        zone_name = raw_step.get("zone")
        if object_name is not None and object_name not in ALLOWED_OBJECTS:
            raise PlanValidationError(f"{label} contains invalid object {object_name!r}")
        if zone_name is not None and zone_name not in ALLOWED_ZONES | TEMPORARY_ZONES:
            raise PlanValidationError(f"{label} contains invalid zone {zone_name!r}")

        if skill == "pick":
            if held_object is not None:
                raise PlanValidationError(f"{label} tries to pick while holding {held_object}")
            held_object = object_name
        elif skill == "place":
            if held_object != object_name:
                raise PlanValidationError(
                    f"{label} must place the object picked immediately before it"
                )
            held_object = None
        elif skill == "home" and index != len(steps) - 1:
            raise PlanValidationError(f"{label} must be the final step")

        normalized.append(dict(raw_step))

    if held_object is not None:
        raise PlanValidationError(f"Plan ends while still holding {held_object}")
    return {"plan": normalized}


def parse_and_validate(content: str) -> dict[str, list[dict[str, str]]]:
    return validate_plan(extract_json_object(content))


def format_step(step: dict[str, str]) -> str:
    """Format a validated skill as a readable function call."""
    skill = step["skill"]
    if skill == "home" or skill in {"open_gripper", "close_gripper"}:
        return f"{skill}()"
    if skill in {"move_above", "pick"}:
        return f"{skill}({step['object']})"
    if skill == "move_to_zone":
        return f"move_to_zone({step['zone']})"
    return f"place({step['object']}, {step['zone']})"


def format_plan(plan: dict[str, list[dict[str, str]]]) -> str:
    return ", ".join(format_step(step) for step in plan["plan"])


def format_numbered_plan(plan: dict[str, list[dict[str, str]]]) -> str:
    return "\n".join(
        f"{index}. {format_step(step)}" for index, step in enumerate(plan["plan"], start=1)
    )
