#!/usr/bin/env python3

import collections
import threading
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray

from biguasim.ardubridge.bridge import ArduPilotBridge
from biguasim.ardubridge import ArduBiguaSimRunner, VEHICLE_REGISTRY
from biguasim_main.interface import BiguaSimInterface
from hydrone_msgs.action import Gripper

GPS_ORIGIN = (33.810313, -118.393867)
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

        # Bases móveis da Fase 1: sorteadas por seed (bloco 'bases' do
        # config.yaml). Aqui, e não na interface, porque com init=False quem
        # cria o env é o runner acima — e o spawn precisa do env pronto e do
        # bridge ainda parado.
        self.interface.spawn_bases()
        self.interface.sensors = self.interface.create_sensor_list()

        # 5. Cria publishers e subscribers ROS2
        self._sensor_publisher_create()
        self._control_subscribers_create()

        # Comandos para o mundo (garra, ...) rodam NA THREAD DO SIMULADOR,
        # entre dois env.step: o env do BiguaSim não é thread-safe.
        self._world_cmds = collections.deque()
        self._agent_name = agent_cfg['agent_name']
        self._gripper_closed = False
        # Reach Offset (cm) mandado com todo comando da garra; ver
        # interface.gripper. [] = usar o padrão do blueprint.
        self.declare_parameter('gripper_reach_cm', [0.0, 0.0, -12.0])
        reach = list(self.get_parameter('gripper_reach_cm').value)
        self._gripper_reach = reach if len(reach) == 3 else None
        # Mesma interface que gripper_dynamixel_node oferece no drone real.
        ActionServer(self, Gripper, '/hydrone/gripper/command',
                     self._exec_gripper,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.REJECT,
                     callback_group=ReentrantCallbackGroup())

        self.get_logger().info(f"ArduBridge pronto: {agent_type} | {len(self.interface.sensors)} sensores")

        # 6. Roda bridge em thread separada
        threading.Thread(target=self._run_bridge, daemon=True).start()

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
                while self._world_cmds:
                    self._world_cmds.popleft()()
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

                # /clock — o mesmo sim_time que vai para o ArduPilot acima.
                #
                # Este loop avanca sim_time em dt por passo de FDM, e o
                # simulador roda em camera lenta: MEDIDO 2026-09-28, com as tres
                # cameras ligadas passam ~0,35 s de simulacao por segundo de
                # parede. Um prazo contado no relogio de parede mede, portanto,
                # uma coisa que o drone nao viveu — e o orcamento de 10 minutos
                # da prova e exatamente um desses prazos: 600 s de parede davam
                # ao veiculo 209 s de voo.
                #
                # Topico ADICIONAL de proposito. Nenhum carimbo de sensor muda
                # (interface.system_time continua True), nenhum no passa a usar
                # use_sim_time, nenhum timer deste stack muda de ritmo. Quem
                # quiser tempo de simulacao le daqui. No drone real ninguem
                # publica isto, e e assim que a missao sabe que deve usar o
                # relogio de parede — sem parametro e sem saber onde esta.
                self._publish_clock(sim_time)

        except KeyboardInterrupt:
            pass
        finally:
            bridge.close()

    def _exec_gripper(self, gh):
        """Fecha/abre a garra no simulador e espera o comando ser aplicado.

        Não há sensor de pega no BiguaSim: `holding` repete o comando. Quem
        precisa saber se pegou de verdade confirma por visão (kit sumiu da
        base) — phase2_mission_node faz isso.
        """
        close = bool(gh.request.close)
        done = threading.Event()

        def apply():
            self.interface.gripper(close, self._agent_name, self._gripper_reach)
            done.set()
        self._world_cmds.append(apply)
        res = Gripper.Result()
        if not done.wait(timeout=5.0):
            res.success, res.message = False, "simulator did not tick in 5 s"
            gh.abort()
            return res
        self._gripper_closed = close
        gh.publish_feedback(Gripper.Feedback(position=1.0 if close else 0.0))
        res.success, res.holding = True, close
        res.message = "closed (sim: no grasp sensor)" if close else "opened"
        gh.succeed()
        return res

    def _publish_clock(self, sim_time):
        msg = Clock()
        msg.clock.sec = int(sim_time)
        msg.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
        self._clock_pub.publish(msg)

    def _sensor_publisher_create(self):
        self._clock_pub = self.create_publisher(
            Clock, '/clock',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
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
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()