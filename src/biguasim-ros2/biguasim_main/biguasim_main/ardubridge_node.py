#!/usr/bin/env python3

import threading
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from biguasim.ardubridge import ArduBiguaSimRunner, VEHICLE_REGISTRY
from biguasim_main.interface import BiguaSimInterface

GPS_ORIGIN = (33.810313, -118.393867)
PACKAGE_NAME = "Competition"
WORLD = "CompetionMap"

class PublishingArduBiguaSimRunner(ArduBiguaSimRunner):
    """Subclasse que injeta um callback de publicação ROS2 no loop principal."""
    def set_state_callback(self, callback): self._state_callback = callback

    def run(self) -> None:
        bridge = self._bridge
        env = self._env
        agent = self._agent_name
        dt = self._dt

        bridge.bind()
        motor_cmds = [0.0] * self._profile.num_motors
        raw = env.step({agent: motor_cmds})
        
        sim_time = 0.0

        print(f"Running {self._profile.name} SITL bridge (Ctrl-C to stop)...")
        try:
            while True:
                frame, pwm = bridge.receive_pwm()
                if frame is None:
                    continue

                motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)
                raw = env.step({agent: motor_cmds})
                sim_time += dt

                json_state = bridge.build_json_state(raw[agent][0], sim_time)
                bridge.send_state(json_state)

                if hasattr(self, '_state_callback') and self._state_callback:
                    self._state_callback(raw)

        except KeyboardInterrupt:
            print("Bridge stopped.")
        finally:
            bridge.close()


class ArduBridgeNode(Node):
    def __init__(self):
        super().__init__('ardubridge_node')
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        if not file_path:
            raise RuntimeError("params_file nao encontrado!")

        # 1. Interface Inicial (sem inicializar o env ainda)
        self.interface = BiguaSimInterface(file_path, init=False, node=self)

        # 2. Patch de _get_agent_id — suporta 'auv0', 'auv_id0', 'holybrox500', etc.
        def novo_get_agent_id(agent_name):
            if '_id' in agent_name:
                base, idx = agent_name.split('_id')
                return base, int(idx)
            match = re.search(r'^([a-zA-Z]+)(\d+)$', agent_name)
            return (match.group(1), int(match.group(2))) if match else (agent_name, 0)
        self.interface._get_agent_id = novo_get_agent_id

        # 3. Configurações do Scenario
        scenario_cfg = self.interface.scenario
        agent_type = scenario_cfg['agents'][0]['agent_type']
        match = next((k for k in VEHICLE_REGISTRY if k.upper() == agent_type.upper()), None)
        if match is None:
            raise RuntimeError(f"Veiculo desconhecido: {agent_type}")
        profile = VEHICLE_REGISTRY[match]

        scenario = PublishingArduBiguaSimRunner.build_scenario(
            profile, package_name=PACKAGE_NAME, world=WORLD,
            ticks_per_sec=scenario_cfg.get('ticks_per_sec', 720),
            location=scenario_cfg['agents'][0].get('location', [0, 0, 0.3]),
            rotation=scenario_cfg['agents'][0].get('rotation', [0, 0, 0]),
        )

        # 4. Runner
        self.runner = PublishingArduBiguaSimRunner(
            profile, scenario, gps_origin=GPS_ORIGIN,
            show_viewport=scenario_cfg.get('show_viewport', True), verbose=False,
        )

        # 5. Descobre o nome real do agente dentro do env e sincroniza tudo
        dyn_dict = self.runner._env._dynamics_dict
        real_keys = list(dyn_dict.keys())
        real_agent_key = real_keys[0] if real_keys else None

        if real_agent_key is None:
            raise RuntimeError("Nenhum agente encontrado no _dynamics_dict!")

        # Mapeia 'auv' → chave real para que _get_command_shape funcione
        if 'auv' not in dyn_dict:
            dyn_dict['auv'] = dyn_dict[real_agent_key]
            self.get_logger().warn(f"Patch dynamics: mapeando 'auv' -> '{real_agent_key}'")

        # Atualiza o scenario para usar o nome real do agente no env
        # (evita que create_sensor_list procure uma chave que não existe)
        original_agent_name = scenario_cfg['agents'][0]['agent_name']
        self.interface.scenario['agents'][0]['agent_name'] = real_agent_key
        self.get_logger().warn(
            f"agent_name atualizado: '{original_agent_name}' -> '{real_agent_key}'"
        )

        # 6. Finalização
        self.interface.env = self.runner._env
        self.interface.initialized = True
        self.interface.sensors = self.interface.create_sensor_list()

        self._sensor_publisher_create()
        self._control_subscribers_create()
        self.runner.set_state_callback(self._publish_callback)

        self.get_logger().info(f"ArduBridge pronto para: {agent_type} (agente: {real_agent_key})")
        threading.Thread(target=self._run_bridge, daemon=True).start()

    def _publish_callback(self, raw: dict):
        """
        Adaptador thread-safe: extrai o estado do agente certo e
        repassa para publish_sensor_data no formato que ela espera.
        
        raw tem formato: { agent_key: state_data, ... }
        publish_sensor_data espera: { agent_base: [state_per_idx, ...], ... }
        
        Aqui montamos o dict no formato correto com base nos sensores registrados.
        """
        self.interface.publish_sensor_data(raw)

    def _run_bridge(self):
        with self.runner:
            self.runner.run()

    def _sensor_publisher_create(self):
        for s in self.interface.sensors:
            topic = f"{s.agent_name.replace('-', '_')}/{s.name}"
            s.publisher = self.create_publisher(s.message_type, topic, 10)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()