#!/usr/bin/env python3

import threading
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray

from biguasim.ardubridge.bridge import ArduPilotBridge
from biguasim.ardubridge import ArduBiguaSimRunner, VEHICLE_REGISTRY
from biguasim_main.interface import BiguaSimInterface

GPS_ORIGIN = (33.810313, -118.393867)

# Which competition phases fly the KopisX8, whose Livox Mid-360 is simulated by
# spinning a depth camera. Phases 1 and 2 fly the HolybroX500, whose config
# declares a DepthCamera too -- the ZED's -- so the spin has to be gated on
# something. Rotating that one would corrupt the depth stream feeding
# zed_mimic, the ZED point cloud and the visual odometry, silently.
SPINNING_PHASES = (3, 4)
# Fallbacks only — package_name/world come from config.yaml (biguasim_scenario).
DEFAULT_PACKAGE_NAME = "Competition"
DEFAULT_WORLD = "CompetionMap"


class ArduBridgeNode(Node):
    def __init__(self):
        super().__init__('ardubridge_node')
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        if not file_path:
            raise RuntimeError("params_file nao encontrado!")

        # 1. Parseia o YAML só para pegar configurações (sem criar env)
        self.interface = BiguaSimInterface(file_path, init=False, node=self)
        scenario_cfg = self.interface.scenario
        agent_cfg = scenario_cfg['agents'][0]
        agent_type = agent_cfg['agent_type']

        match = next((k for k in VEHICLE_REGISTRY if k.upper() == agent_type.upper()), None)
        if match is None:
            raise RuntimeError(f"Veiculo desconhecido: {agent_type}")
        self.profile = VEHICLE_REGISTRY[match]
        # self.profile["name"] = agent_cfg["agent_name"]

        # 2. Builda o scenario com os sensores CERTOS para o ArduPilot
        #    MAS adiciona os sensores extras do YAML (para publicação ROS2)
        ardu_scenario = ArduBiguaSimRunner.build_scenario(
            self.profile,
            package_name=scenario_cfg.get('package_name', DEFAULT_PACKAGE_NAME),
            world=scenario_cfg.get('world', DEFAULT_WORLD),
            agent_name=agent_cfg['agent_name'],
            ticks_per_sec=scenario_cfg.get('ticks_per_sec', 200),
            location=agent_cfg.get('location', [0, 0, 5]),
            rotation=agent_cfg.get('rotation', [0, 0, 0]),
        )

        # Adiciona sensores extras do YAML que não estão no build_scenario
        ardu_sensor_types = {s['sensor_type'] for s in ardu_scenario['agents'][0]['sensors']}
        yaml_sensor_types = {s['sensor_type'] for s in agent_cfg.get('sensors', [])}
        extras = yaml_sensor_types - ardu_sensor_types
        for sensor in agent_cfg.get('sensors', []):
            if sensor['sensor_type'] in extras:
                ardu_scenario['agents'][0]['sensors'].append(sensor)

        # Configurações extras do YAML
        ardu_scenario['octree_min'] = scenario_cfg.get('octree_min', 0.02)
        ardu_scenario['octree_max'] = scenario_cfg.get('octree_max', 5.0)
        ardu_scenario['show_viewport'] = scenario_cfg.get('show_viewport', True)

        # 3. Cria o runner com o scenario correto
        self.runner = ArduBiguaSimRunner(
            self.profile,
            ardu_scenario,
            gps_origin=GPS_ORIGIN,
            show_viewport=scenario_cfg.get('show_viewport', True),
            verbose=False,
        )

        # 4. Conecta o env do runner na interface para publicação ROS2
        #    O scenario do env tem os sensores certos com ros_publish do YAML
        self.interface.env = self.runner._env
        # Usa o scenario do runner (sensores corretos) mas com ros_publish do YAML
        runner_scenario = self.runner._env._scenario
        for i, agent in enumerate(runner_scenario['agents']):
            for sensor in agent['sensors']:
                # Seta ros_publish baseado no YAML
                yaml_match = next(
                    (s for s in agent_cfg.get('sensors', [])
                     if s['sensor_type'] == sensor['sensor_type']),
                    None
                )
                sensor['ros_publish'] = yaml_match['ros_publish'] if yaml_match else False

        self.interface.scenario = runner_scenario
        self.interface.initialized = True
        self.interface.sensors = self.interface.create_sensor_list()

        # 5. Cria publishers e subscribers ROS2
        self._sensor_publisher_create()
        self._control_subscribers_create()
        self._spin_setup(agent_cfg)

        self.get_logger().info(f"ArduBridge pronto: {agent_type} | {len(self.interface.sensors)} sensores")

        # 6. Roda bridge em thread separada
        threading.Thread(target=self._run_bridge, daemon=True).start()

    def _spin_setup(self, agent_cfg):
        """Prepare to turn a sensor between simulation steps, or don't.

        The Mid-360 is simulated by spinning a depth camera (see
        hydrone_bringup's livox_mimic_node), which is a phase 3/4 thing: the
        HolybroX500 flown in phases 1 and 2 has a DepthCamera of its own, the
        ZED's, and turning THAT would quietly rotate the depth behind
        zed_mimic, the point cloud and the visual odometry. So the phase says
        whether to spin at all, and nothing happens unless it is one that flies
        the Kopis.
        """
        self._spin_sensor = None
        self._spin_pub = None

        self.declare_parameter('phase', 1)
        self.declare_parameter('spin_sensor', 'DepthCamera')
        self.declare_parameter('spin_step_deg', 60.0)
        # sensors.py: "It will be applied in approximately three ticks." The
        # yaw a frame was RENDERED at is therefore the one commanded a few
        # steps earlier, and that -- not the one commanded now -- is what gets
        # published for the mimic to rotate by.
        self.declare_parameter('spin_apply_ticks', 3)

        phase = int(self.get_parameter('phase').value)
        if phase not in SPINNING_PHASES:
            self.get_logger().info(
                f"phase {phase}: no sensor spin (phases "
                f"{SPINNING_PHASES} fly the spinning-lidar airframe)")
            return

        name = self.get_parameter('spin_sensor').value
        # env.agents is keyed by the BATCH name ('uav0-id0'); the ROS topics
        # use the underscored one. Try both rather than hardcode either.
        base = agent_cfg['agent_name']
        env = self.runner._env
        key = next((k for k in (f"{base}-id0", base) if k in env.agents), None)
        if key is None:
            self.get_logger().error(
                f"phase {phase} wants to spin '{name}' but no agent matching "
                f"'{base}' is in the environment ({list(env.agents)}); "
                "flying without the spin")
            return
        sensor = env.agents[key].sensors.get(name)
        if sensor is None:
            self.get_logger().error(
                f"phase {phase} wants to spin '{name}' but agent '{key}' has "
                f"no such sensor ({list(env.agents[key].sensors)}); flying "
                "without the spin")
            return

        self._spin_sensor = sensor
        self._spin_step = float(self.get_parameter('spin_step_deg').value)
        # A pipeline of commanded yaws: what comes out is what was commanded
        # `spin_apply_ticks` steps ago, i.e. what the sensor is actually at
        # now. Pre-filled with 0 because that is where it starts.
        depth = max(1, int(self.get_parameter('spin_apply_ticks').value))
        self._spin_pending = deque([0.0] * depth, maxlen=depth)
        self._spin_yaw = 0.0

        # Alongside the sensor's own topics: /biguasim/<agent>/<sensor>/spin,
        # the same shape as the /camera_info companion. The prefix is taken
        # from the sensor's OWN publisher rather than rebuilt, so the spin
        # topic cannot land somewhere the depth topic is not.
        prefix = next((s.agent_name for s in self.interface.sensors
                       if s.name == name), None)
        if prefix is None:
            self.get_logger().error(
                f"'{name}' publishes no ROS topic (ros_publish false?), so a "
                "spin report would have nothing to sit beside; flying without "
                "the spin")
            return
        topic = f"{prefix}/{name}/spin"
        self._spin_pub = self.create_publisher(Vector3Stamped, topic, 10)
        self.get_logger().info(
            f"phase {phase}: spinning '{name}' {self._spin_step:g} deg/step, "
            f"reporting it on {topic} ({depth}-tick apply delay)")

    def _spin_step_and_publish(self):
        """Turn the sensor one step and say where it actually is.

        Called once per SIMULATION STEP, not once per loop iteration: a pass
        where no PWM arrived advances no time and must not advance the sweep.

        Published BEFORE the sensors of the same step, deliberately. The mimic
        pairs a frame with the last spin sample at or before the frame's own
        stamp, so the sample has to be the earlier of the two or every frame
        would be matched to the previous step's pose.
        """
        if self._spin_sensor is None:
            return

        effective = self._spin_pending[0]
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.vector.z = effective          # [roll, pitch, yaw], degrees
        self._spin_pub.publish(msg)

        self._spin_yaw = (self._spin_yaw + self._spin_step) % 360.0
        self._spin_pending.append(self._spin_yaw)
        self._spin_sensor.rotate([0.0, 0.0, self._spin_yaw])

    def _run_bridge(self):
        """Roda o loop do ArduPilot e publica no ROS2 a cada frame."""
        bridge = self.runner._bridge
        env = self.runner._env
        agent = self.runner._agent_name
        dt = self.runner._dt
        profile = self.profile

        bridge.bind()
        motor_cmds = [0.0] * profile.num_motors
        raw = env.step(motor_cmds)
        sim_time = 0.0

        self.get_logger().info("Bridge UDP iniciada, aguardando ArduPilot...")

        try:
            while True:
                frame, pwm = bridge.receive_pwm()
                if frame is None:
                    continue

                motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)

                self._spin_step_and_publish()

                raw = env.step(motor_cmds)
                sim_time += dt

                # Manda estado pro ArduPilot
                agent_state = raw[agent][0]
                json_state = bridge.build_json_state(agent_state, sim_time)
                if json_state:
                    bridge.send_state(json_state)

                # Publica no ROS2
                try:
                    self.interface.publish_sensor_data(raw)
                except Exception as e:
                    self.get_logger().warn(f"Erro publicando sensores: {e}")

        except KeyboardInterrupt:
            pass
        finally:
            bridge.close()

    def _sensor_publisher_create(self):
        for sensor in self.interface.sensors:
            # agent_name comes from config.yaml (agents[0].agent_name), carries
            # the biguasim batch suffix -> e.g. "auv0_id0". Topics land under the
            # launch namespace: /biguasim/<agent_name>/<sensor.name>.
            topic = f"{sensor.agent_name}/{sensor.name}"
            sensor.publisher = self.create_publisher(sensor.message_type, topic, 10)
            self.get_logger().info(f"Publisher: {topic}")

    def _control_subscribers_create(self):
        for ag in self.interface.scenario['agents']:
            name = ag['agent_name'].replace('-', '_')
            self.create_subscription(
                Float64MultiArray,
                f"{name}/command_control",
                lambda msg, a=name: self.interface.send_control_command(a, list(msg.data)),
                10
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArduBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()