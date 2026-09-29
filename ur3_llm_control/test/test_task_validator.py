import json

import pytest

from ur3_llm_control.task_validator import (
    PlanValidationError,
    parse_and_validate,
    validate_plan,
)


def test_accepts_valid_vietnamese_task_plan():
    raw = json.dumps(
        {
            "plan": [
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                {"skill": "home"},
            ]
        }
    )
    assert parse_and_validate(raw)["plan"][1]["zone"] == "zone_b"


def test_accepts_valid_english_task_plan():
    plan = {
        "plan": [
            {"skill": "pick", "object": "blue_cube"},
            {"skill": "place", "object": "blue_cube", "zone": "zone_c"},
            {"skill": "home"},
        ]
    }
    assert validate_plan(plan) == plan


def test_accepts_json_code_fence():
    raw = '```json\n{"plan":[{"skill":"home"}]}\n```'
    assert parse_and_validate(raw) == {"plan": [{"skill": "home"}]}


@pytest.mark.parametrize(
    "plan",
    [
        {"plan": [{"skill": "move_joint", "joint": 1}]},
        {"plan": [{"skill": "pick", "object": "green_cube"}]},
        {"plan": [{"skill": "place", "object": "red_cube", "zone": "zone_z"}]},
        {"plan": [{"skill": "pick", "object": "red_cube", "joints": [0, 1]}]},
        {"plan": [{"skill": "home", "object": "red_cube"}]},
    ],
)
def test_rejects_non_whitelisted_values(plan):
    with pytest.raises(PlanValidationError):
        validate_plan(plan)


def test_rejects_place_without_matching_pick():
    with pytest.raises(PlanValidationError):
        validate_plan(
            {
                "plan": [
                    {"skill": "pick", "object": "red_cube"},
                    {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
                ]
            }
        )


def test_accepts_optional_robot_skills():
    plan = {
        "plan": [
            {"skill": "move_above", "object": "yellow_cube"},
            {"skill": "open_gripper"},
            {"skill": "close_gripper"},
            {"skill": "move_to_zone", "zone": "zone_c"},
        ]
    }
    assert validate_plan(plan) == plan


def test_accepts_three_object_arrangement_plan():
    plan = {
        "plan": [
            {"skill": "pick", "object": "yellow_cube"},
            {"skill": "place", "object": "yellow_cube", "zone": "zone_a"},
            {"skill": "pick", "object": "blue_cube"},
            {"skill": "place", "object": "blue_cube", "zone": "zone_b"},
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_c"},
            {"skill": "home"},
        ]
    }
    assert validate_plan(plan) == plan


def test_rejects_home_between_multi_object_steps():
    with pytest.raises(PlanValidationError, match="final step"):
        validate_plan(
            {
                "plan": [
                    {"skill": "pick", "object": "yellow_cube"},
                    {"skill": "place", "object": "yellow_cube", "zone": "zone_a"},
                    {"skill": "home"},
                    {"skill": "pick", "object": "blue_cube"},
                    {"skill": "place", "object": "blue_cube", "zone": "zone_b"},
                ]
            }
        )
