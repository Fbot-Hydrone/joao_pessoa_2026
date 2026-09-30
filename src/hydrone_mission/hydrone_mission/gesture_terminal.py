#!/usr/bin/env python3
"""
gesture_terminal — the operator's gestures from the keyboard, for BiguaSim.

There is nobody in the simulator for the camera to film, so this publishes
exactly what the gesture recogniser will publish on the real drone
(hydrone_msgs/HumanGesture on /hydrone/vision/human_gesture). The mission
cannot tell the two apart; swapping in the camera later changes nothing
downstream.

    ros2 run hydrone_mission gesture_terminal

It must run in its own terminal (it reads the keyboard), not inside a launch.
Type `ajuda` for the commands. The operator is placed `operator_distance` m in
front of the drone, so human_position reads like a real detection.
"""

import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger

from hydrone_msgs.msg import HumanGesture

from hydrone_mission.phase3 import core

MISSION_NODE = "/phase3_hri_node"

# (words, what it does, help)
EXTRA = [
    (("iniciar", "start"), "decolagem + aproximação (quando auto_start:=false)"),
    (("abortar", "abort"), "LAND onde estiver, fim da missão"),
    (("passo <m>",), "tamanho do passo horizontal, ex.: passo 1.0"),
    (("altura <m>",), "tamanho do passo vertical, ex.: altura 0.5"),
    (("giro <graus>",), "tamanho do giro, ex.: giro 90"),
    (("status",), "estado atual da missão"),
    (("ajuda", "?"), "esta lista"),
    (("sair",), "fecha o terminal (o drone segura onde está)"),
]


def help_text():
    lines = ["", "Gestos (publicados como a câmera publicaria):"]
    for words, gesture, text in core.TERMINAL_COMMANDS:
        lines.append(f"  {' / '.join(words):34s} {text}   [{gesture}]")
    lines.append("Terminal:")
    for words, text in EXTRA:
        lines.append(f"  {' / '.join(words):34s} {text}")
    lines.append("Direções no referencial do DRONE (direita = direita do drone).")
    return "\n".join(lines)


class GestureTerminal(Node):

    def __init__(self):
        super().__init__("gesture_terminal")
        self.declare_parameter("gesture_topic", "/hydrone/vision/human_gesture")
        self.declare_parameter("operator_distance", 2.0)
        self.pub = self.create_publisher(
            HumanGesture, self.get_parameter("gesture_topic").value, 10)
        self.pose = None
        self.status = "(sem status ainda — a missão está rodando?)"
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, qos_profile_sensor_data)
        self.create_subscription(String, "/hydrone/phase3/status", self._cb_status, 10)
        self.cli_start = self.create_client(Trigger, "/hydrone/phase3/start")
        self.cli_abort = self.create_client(Trigger, "/hydrone/phase3/abort")
        self.cli_params = self.create_client(SetParameters, f"{MISSION_NODE}/set_parameters")

    def _cb_pose(self, m):
        self.pose = m

    def _cb_status(self, m):
        self.status = m.data

    def send(self, gesture):
        msg = HumanGesture()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.gesture_name = gesture
        msg.confidence = 1.0
        if self.pose is not None:
            p, q = self.pose.pose.position, self.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            d = self.get_parameter("operator_distance").value
            msg.human_position.x = p.x + d * math.cos(yaw)
            msg.human_position.y = p.y + d * math.sin(yaw)
        self.pub.publish(msg)
        print(f"  -> {gesture}  ({core.action_for(gesture)})")

    def trigger(self, client):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"  {client.srv_name} indisponível — a missão está rodando?")
            return
        res = self._wait(client.call_async(Trigger.Request()))
        print(f"  {res.message if res else 'sem resposta'}")

    def set_param(self, name, value):
        if not self.cli_params.wait_for_service(timeout_sec=2.0):
            print(f"  {MISSION_NODE} indisponível")
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))]
        res = self._wait(self.cli_params.call_async(req))
        ok = res is not None and res.results and res.results[0].successful
        print(f"  {name} = {value}" if ok else f"  falhou: {res}")

    @staticmethod
    def _wait(future, timeout=10.0):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        done.wait(timeout)
        return future.result() if future.done() else None


def repl(node):
    print(help_text())
    while rclpy.ok():
        try:
            line = input("\nfase3> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        word, *rest = line.split()
        word = word.lower()
        if word in ("sair", "exit", "quit"):
            break
        if word in ("ajuda", "help", "?"):
            print(help_text())
        elif word == "status":
            print(f"  {node.status}")
        elif word in ("iniciar", "start"):
            node.trigger(node.cli_start)
        elif word in ("abortar", "abort"):
            node.trigger(node.cli_abort)
        elif word in ("passo", "altura", "giro"):
            name = {"passo": "step_m", "altura": "step_z_m", "giro": "yaw_step_deg"}[word]
            try:
                node.set_param(name, float(rest[0]))
            except (IndexError, ValueError):
                print(f"  uso: {word} <número>")
        else:
            gesture = core.parse_terminal(word)
            if gesture is None:
                print(f"  '{word}'? — digite ajuda")
            else:
                node.send(gesture)


def main():
    rclpy.init()
    node = GestureTerminal()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        repl(node)
    except ExternalShutdownException:
        pass
    finally:
        # stop the spinning thread before tearing the node down under it
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
