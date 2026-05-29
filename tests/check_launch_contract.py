#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import sys


def normalize_key_part(part):
    text = getattr(part, "text", None)
    if text is not None:
        return text
    return str(part)


def normalize_value_part(part):
    variable_name = getattr(part, "variable_name", None)
    if variable_name is not None:
        return normalize_value(variable_name)
    text = getattr(part, "text", None)
    if text is not None:
        return text
    return str(part)


def normalize_key(key):
    if isinstance(key, (list, tuple)):
        return "".join(normalize_key_part(part) for part in key)
    return normalize_key_part(key)


def normalize_value(value):
    if isinstance(value, (list, tuple)):
        return "".join(normalize_value_part(part) for part in value)
    return normalize_value_part(value)


def get_node_attr(node, public_name, private_name):
    try:
        return normalize_value(getattr(node, public_name))
    except (AttributeError, RuntimeError):
        pass
    return normalize_value(getattr(node, private_name))


def main() -> int:
    launch_file = sys.argv[1]
    spec = importlib.util.spec_from_file_location("base_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    desc = module.generate_launch_description()
    node = None
    default_values = {}

    for entity in desc.entities:
        cls_name = entity.__class__.__name__
        if cls_name == "DeclareLaunchArgument":
            default_values[entity.name] = normalize_value(entity.default_value)
        elif cls_name == "Node":
            node = entity

    expected_defaults = {
        "send_hz": "20.0",
        "odom_hz": "50.0",
        "cmd_vel_timeout": "0.4",
        "publish_tf": "true",
        "odom_topic": "odom",
        "odom_frame": "odom",
        "base_frame": "base_footprint",
        "wheel_diameter": "0.067",
        "wheel_base": "0.28",
        "motor1_factor": "1.0",
        "motor2_factor": "1.0",
        "reduction_ratio": "56.0",
        "ff_factor": "0.3",
        "pid_kp": "0.05",
        "pid_ki": "0.2",
        "pid_kd": "0.01",
        "cfg_send_on_startup": "true",
        "feedback_enable": "false",
        "rpmsg_ctrl_dev": "/dev/rpmsg_ctrl0",
        "rpmsg_data_dev": "/dev/rpmsg0",
        "rpmsg_service_name": "rpmsg:motor_ctrl",
        "rpmsg_local_addr": "1003",
        "rpmsg_remote_addr": "1002",
    }

    assert default_values.keys() == expected_defaults.keys(), (
        f"unexpected launch arguments: {sorted(default_values.keys())!r}"
    )

    for key, expected in expected_defaults.items():
        assert default_values.get(key) == expected, (
            f"unexpected default for {key}: {default_values.get(key)!r}"
        )

    assert node is not None, "Node action missing"
    assert get_node_attr(node, "node_package", "_Node__package") == "base", (
        "unexpected node package"
    )
    assert get_node_attr(node, "node_executable", "_Node__executable") == "esos_base_control_node", (
        "unexpected node executable"
    )
    assert get_node_attr(node, "node_name", "_Node__node_name") == "esos_base_control_node", (
        "unexpected node name"
    )

    parameters = getattr(node, "_Node__parameters")
    assert len(parameters) == 1, f"unexpected parameter blocks: {len(parameters)}"

    parameter_map = {
        normalize_key(key): normalize_value(value)
        for key, value in parameters[0].items()
    }

    for key in expected_defaults:
        assert parameter_map.get(key) == key, (
            f"parameter {key} is not wired to LaunchConfiguration({key}); "
            f"got {parameter_map.get(key)!r}"
        )

    print("BASE_LAUNCH_CONTRACT_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
