#!/usr/bin/env python3
"""Offline point-to-point control and safety-contract verification.

This check intentionally uses only the Python standard library.  It does not
replace a ROS 2/Keil build or a propeller-off hardware test.
"""

from __future__ import annotations

import ast
import math
import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Tuple

sys.dont_write_bytecode = True

ROOT = Path(__file__).resolve().parents[1]
MISSION_PACKAGE = ROOT / "onboard_ws" / "src" / "p_to_p_mission"
sys.path.insert(0, str(MISSION_PACKAGE))

from p_to_p_mission.geometry import body_offset_to_map, normalize_angle_deg  # noqa: E402


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def parse_scalar(value: str):
    value = value.split("#", 1)[0].strip()
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    try:
        return ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return value


def read_ros_parameters(path: Path, node_name: str) -> Dict[str, object]:
    params: Dict[str, object] = {}
    current_node = ""
    in_parameters = False
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        indent = len(raw_line) - len(raw_line.lstrip())
        stripped = raw_line.strip()
        if indent == 0 and stripped.endswith(":"):
            current_node = stripped[:-1]
            in_parameters = False
            continue
        if current_node != node_name:
            continue
        if indent == 2 and stripped == "ros__parameters:":
            in_parameters = True
            continue
        if in_parameters and indent == 4 and ":" in stripped:
            key, value = stripped.split(":", 1)
            params[key.strip()] = parse_scalar(value)
    require(bool(params), f"No parameters found for {node_name}")
    return params


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def map_to_body(vx: float, vy: float, yaw_deg: float) -> Tuple[float, float]:
    yaw = math.radians(yaw_deg)
    return (
        math.cos(yaw) * vx + math.sin(yaw) * vy,
        -math.sin(yaw) * vx + math.cos(yaw) * vy,
    )


def body_to_map(vx: float, vy: float, yaw_deg: float) -> Tuple[float, float]:
    yaw = math.radians(yaw_deg)
    return (
        math.cos(yaw) * vx - math.sin(yaw) * vy,
        math.sin(yaw) * vx + math.cos(yaw) * vy,
    )


def simulate_target(
    pose: Tuple[float, float, float, float],
    target: Tuple[float, float, float, float],
    mission: Dict[str, object],
    pid: Dict[str, object],
    stable_required_sec: float,
    timeout_sec: float = 120.0,
) -> Tuple[Tuple[float, float, float, float], float, Dict[str, float]]:
    x, y, z, yaw = pose
    tx, ty, tz, tyaw = target
    dt = 0.02
    stable_sec = 0.0
    elapsed = 0.0
    peaks = {"xy": 0.0, "z": 0.0, "yaw": 0.0}

    while elapsed < timeout_sec:
        error_x = tx - x
        error_y = ty - y
        distance = math.hypot(error_x, error_y)
        if distance > 1e-9:
            speed = min(float(pid["max_linear_velocity"]), float(pid["kp_xy"]) * distance)
            map_vx = speed * error_x / distance
            map_vy = speed * error_y / distance
        else:
            map_vx = 0.0
            map_vy = 0.0

        body_vx, body_vy = map_to_body(map_vx, map_vy, yaw)
        applied_vx, applied_vy = body_to_map(body_vx, body_vy, yaw)
        vz = clamp(float(pid["kp_z"]) * (tz - z), float(pid["max_vertical_velocity"]))
        yaw_rate = clamp(
            float(pid["kp_yaw"]) * normalize_angle_deg(tyaw - yaw),
            float(pid["max_angular_velocity"]),
        )

        peaks["xy"] = max(peaks["xy"], math.hypot(body_vx, body_vy))
        peaks["z"] = max(peaks["z"], abs(vz))
        peaks["yaw"] = max(peaks["yaw"], abs(yaw_rate))

        x += applied_vx * dt
        y += applied_vy * dt
        z += vz * dt
        yaw = normalize_angle_deg(yaw + yaw_rate * dt)
        elapsed += dt

        reached = (
            math.hypot(tx - x, ty - y) <= float(mission["position_tolerance_cm"])
            and abs(tz - z) <= float(mission["height_tolerance_cm"])
            and abs(normalize_angle_deg(tyaw - yaw)) <= float(mission["yaw_tolerance_deg"])
        )
        stable_sec = stable_sec + dt if reached else 0.0
        if stable_sec >= stable_required_sec:
            return (x, y, z, yaw), elapsed, peaks

    raise AssertionError(f"Target was not reached within {timeout_sec:.0f}s: {target}")


def verify_simulation(config_path: Path) -> None:
    mission = read_ros_parameters(config_path, "p_to_p_mission")
    pid = read_ros_parameters(config_path, "position_pid_controller")

    for yaw in (0.0, 35.0, 90.0, -135.0, 179.0):
        for velocity in ((30.0, 0.0), (0.0, -20.0), (17.0, 23.0)):
            body = map_to_body(*velocity, yaw)
            restored = body_to_map(*body, yaw)
            require(math.isclose(restored[0], velocity[0], abs_tol=1e-9), "X rotation mismatch")
            require(math.isclose(restored[1], velocity[1], abs_tol=1e-9), "Y rotation mismatch")

    total_time = 0.0
    global_peaks = {"xy": 0.0, "z": 0.0, "yaw": 0.0}
    for start_yaw in (0.0, 90.0, -135.0):
        start = (100.0, -50.0, 0.0, start_yaw)
        b_x, b_y = body_offset_to_map(
            start[0],
            start[1],
            start_yaw,
            float(mission["b_offset_x_cm"]),
            float(mission["b_offset_y_cm"]),
        )
        takeoff_target = (start[0], start[1], float(mission["cruise_height_cm"]), start_yaw)
        b_target = (b_x, b_y, float(mission["cruise_height_cm"]), start_yaw)
        landing_target = (b_x, b_y, float(mission["landing_height_cm"]), start_yaw)

        pose, elapsed, peaks = simulate_target(
            start, takeoff_target, mission, pid, float(mission["reached_stable_sec"])
        )
        total_time += elapsed
        pose, elapsed, peaks_b = simulate_target(
            pose, b_target, mission, pid, float(mission["reached_stable_sec"])
        )
        total_time += elapsed
        pose, elapsed, peaks_land = simulate_target(
            pose, landing_target, mission, pid, float(mission["reached_stable_sec"])
        )
        total_time += elapsed
        yaw_target = (pose[0], pose[1], pose[2], normalize_angle_deg(start_yaw + 120.0))
        pose, elapsed, peaks_yaw = simulate_target(
            pose, yaw_target, mission, pid, float(mission["reached_stable_sec"])
        )
        total_time += elapsed

        for name in global_peaks:
            global_peaks[name] = max(
                peaks[name], peaks_b[name], peaks_land[name], peaks_yaw[name], global_peaks[name]
            )
        require(math.hypot(b_x - pose[0], b_y - pose[1]) <= float(mission["position_tolerance_cm"]),
                "Landing XY position is outside tolerance")
        require(abs(float(mission["landing_height_cm"]) - pose[2]) <=
                float(mission["height_tolerance_cm"]), "Landing height is outside tolerance")

    require(global_peaks["xy"] <= float(pid["max_linear_velocity"]) + 1e-9,
            "Horizontal velocity limit exceeded")
    require(global_peaks["z"] <= float(pid["max_vertical_velocity"]) + 1e-9,
            "Vertical velocity limit exceeded")
    require(global_peaks["yaw"] <= float(pid["max_angular_velocity"]) + 1e-9,
            "Yaw-rate limit exceeded")
    print(
        "SIMULATION_OK "
        f"scenarios=3 elapsed={total_time:.1f}s "
        f"peaks=({global_peaks['xy']:.1f},{global_peaks['z']:.1f},{global_peaks['yaw']:.1f})"
    )


def verify_lidar_velocity_math() -> None:
    dt = 0.10
    local_vx = (0.03 - 0.00) * 100.0 / dt
    local_vy = (-0.04 - 0.00) * 100.0 / dt
    require(math.isclose(math.hypot(local_vx, local_vy), 50.0),
            "Lidar pose differencing has incorrect cm/s scaling")

    body_vx, body_vy = map_to_body(local_vx, local_vy, 90.0)
    require(math.isclose(body_vx, -40.0, abs_tol=1e-9),
            "Lidar velocity X rotation mismatch")
    require(math.isclose(body_vy, -30.0, abs_tol=1e-9),
            "Lidar velocity Y rotation mismatch")

    payload = bytearray(12)
    struct.pack_into("<hhh", payload, 6, round(body_vx), round(body_vy), 0)
    require(payload[:6] == bytes(6), "Legacy 0x32 reserved bytes must remain zero")
    require(struct.unpack_from("<hhh", payload, 6) == (-40, -30, 0),
            "Legacy 0x32 measured-velocity payload mismatch")
    print("LIDAR_VELOCITY_MATH_OK speed=50.0cm/s frame=0x32")


def require_markers(path: Path, markers) -> None:
    text = path.read_text(encoding="utf-8", errors="ignore")
    for marker in markers:
        require(marker in text, f"Missing safety marker in {path}: {marker}")


def verify_safety_contracts() -> None:
    mission_py = ROOT / "onboard_ws" / "src" / "p_to_p_mission" / "p_to_p_mission" / "mission_node.py"
    pid_cpp = ROOT / "onboard_ws" / "src" / "pid_control_pkg" / "src" / "pid_controller.cpp"
    bridge_cpp = ROOT / "onboard_ws" / "src" / "uart_to_stm32" / "src" / "uart_to_stm32.cpp"
    bridge_hpp = ROOT / "onboard_ws" / "src" / "uart_to_stm32" / "include" / "uart_to_stm32" / "uart_to_stm32.hpp"
    fc_rx = ROOT / "flight_controller" / "FcSrc" / "AnoDTRasp.c"
    fc_sensor = ROOT / "flight_controller" / "FcSrc" / "LX_FC_EXT_Sensor.c"
    fc_loop = ROOT / "flight_controller" / "FcSrc" / "ANO_LX.c"
    fc_user = ROOT / "flight_controller" / "FcSrc" / "User_Task.c"
    fc_rc = ROOT / "flight_controller" / "DriversBsp" / "Drv_BSP.c"
    fc_rc_header = ROOT / "flight_controller" / "DriversBsp" / "Drv_BSP.h"
    keil = ROOT / "flight_controller" / "ProjectSTM32F407" / "ANO_LX_STM32F407.uvprojx"
    config = MISSION_PACKAGE / "config" / "p_to_p.yaml"
    mission_launch = MISSION_PACKAGE / "launch" / "p_to_p.launch.py"

    mission_text = mission_py.read_text(encoding="utf-8")
    require(mission_text.count("self.route_kick_remaining = 1") == 2,
            "Route enable must be a single explicit event for start and abort")
    require("self.route_kick_remaining = 20" not in mission_text and
            "self.route_kick_remaining = 10" not in mission_text,
            "Repeated route enable could defeat the command watchdog")
    require_markers(mission_py, [
        '"FC_LANDING"',
        "self.mission_complete_pub.publish(Empty())",
        "self.active_target = None",
    ])
    require_markers(pid_cpp, [
        "pid_z_.setOutputLimits(max_vertical_vel_, -max_vertical_vel_)",
        "hasFreshHeightData(now_time)",
        "publishSensorFailsafeZero(\"TF unavailable or stale\")",
    ])
    require_markers(bridge_cpp, [
        "sendMissionControlToSerial(true)",
        "sendMissionControlToSerial(false)",
        '"/route_choice", rclcpp::QoS(1).reliable()',
        "command_timeout_latched_ = true",
        "route_task_active_ = false",
        '"/lidar_velocity_body"',
        "processLidarVelocityTransform(transform)",
        "sendLidarVelocityToSerial(body_vx_cmps, body_vy_cmps)",
        "std::vector<uint8_t> data(12, 0U)",
        "writeInt16LittleEndian(data, 6, vel_x)",
        "writeInt16LittleEndian(data, 8, vel_y)",
    ])
    require_markers(bridge_hpp, ["LIDAR_VELOCITY_FRAME_ID = 0x32"])
    bridge_text = bridge_cpp.read_text(encoding="utf-8")
    require('"/velocity_map"' not in bridge_text,
            "Legacy /velocity_map path bypasses point-to-point task gating")
    require("LASER_GROUND_HEIGHT_FRAME_ID" not in bridge_text,
            "Unused 0x07 height forwarding should not consume serial bandwidth")
    for removed_topic in ("/servo_control", "/electromagnet_control", "/buzzer_led_control"):
        require(removed_topic not in bridge_text,
                f"Task-specific actuator topic remains in point-to-point bridge: {removed_topic}")
    require_markers(fc_rx, [
        "ROS_VELOCITY_TIMEOUT_MS 300U",
        "case 0x31",
        "case 0x66",
        "case 0x67",
        "one_key_land_pending",
        "OneKey_Land()",
        "takeoff_ready = 1",
        "Set_m_speed(0, 0, 0, 0)",
        "case 0x32",
        "rxFrame.frame.dataLen >= 12U",
        "Set_m_speed_now(rosData.tloc[0],rosData.tloc[1],rosData.tloc[2])",
    ])
    require_markers(fc_sensor, [
        "source_speed=0",
        "EXTERNAL_VELOCITY_TIMEOUT_MS 300U",
        "external_velocity_timeout_latched=1",
        "last_external_update_cnt",
        "ext_sens.gen_vel.st_data.hca_velocity_cmps[2] = 0x8000",
    ])
    require_markers(fc_loop, ["AnoDTRaspRunTask1Ms();"])
    require_markers(fc_rc, ["//DrvRcPpmInit();\n\tDrvRcSbusInit();"])
    require_markers(fc_rc_header, ["s16 ppm_ch[10];"])
    require_markers(fc_user, [
        "ch6_low_seen",
        "manual_start_active",
        "Con_flag || manual_start_active",
    ])
    require("..\\FcSrc\\AnoDTRasp.c" in keil.read_text(encoding="utf-8"),
            "Keil project does not reference patched AnoDTRasp.c")
    mission_params = read_ros_parameters(config, "p_to_p_mission")
    require(mission_params.get("height_topic") == "/height",
            "Point-to-point mission must use the flight-controller /height topic")
    bridge_params = read_ros_parameters(config, "uart_to_stm32_node")
    require(bridge_params.get("lidar_velocity_enabled") is True,
            "Lidar measured velocity forwarding must be explicitly enabled")
    require(bridge_params.get("lidar_velocity_frame") == "odom",
            "Lidar velocity must use the continuous local odom frame")
    require(0.0 < float(bridge_params["lidar_velocity_filter_alpha"]) <= 1.0,
            "Lidar velocity filter alpha must be in (0, 1]")
    require(float(bridge_params["lidar_velocity_max_cmps"]) < 60.0,
            "Lidar velocity must stay within the STM32 legacy input range")
    launch_text = mission_launch.read_text(encoding="utf-8")
    require("laser_array_ground_node" not in launch_text,
            "Main launch must not require an uninstalled laser array")
    require("/laser_array/ground_height" not in launch_text,
            "PID height input must not be remapped to the absent laser array")
    print("SAFETY_CONTRACTS_OK layers=mission,pid,bridge,stm32")


def verify_project_structure() -> None:
    python_files = list((ROOT / "onboard_ws" / "src" / "p_to_p_mission").rglob("*.py"))
    for path in python_files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    xml_files = list((ROOT / "onboard_ws" / "src").rglob("package.xml"))
    xml_files += list((ROOT / "flight_controller").rglob("*.uvprojx"))
    for path in xml_files:
        ET.parse(path)

    package_names = []
    for path in (ROOT / "onboard_ws" / "src").rglob("package.xml"):
        package_names.append(ET.parse(path).getroot().findtext("name"))
    require(len(package_names) == len(set(package_names)), "Duplicate ROS package names")

    print(f"STRUCTURE_OK packages={len(package_names)} python={len(python_files)} xml={len(xml_files)}")


def main() -> int:
    try:
        config = MISSION_PACKAGE / "config" / "p_to_p.yaml"
        verify_project_structure()
        verify_safety_contracts()
        verify_lidar_velocity_math()
        verify_simulation(config)
    except Exception as exc:  # noqa: BLE001 - command-line verifier reports the contract failure
        print(f"VERIFY_FAILED: {exc}", file=sys.stderr)
        return 1
    print("OFFLINE_VERIFY_OK (does not replace ROS 2/Keil/hardware validation)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
