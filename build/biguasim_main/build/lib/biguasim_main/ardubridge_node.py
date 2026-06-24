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
    def set_state_callback(self, callback): self._state_callback = callback
    def run(self) -> None:
        bridge, env, agent, dt = self._bridge, self._env, self._agent_name, self._dt
        bridge.bind()
        motor_cmds = [0.0] * self._profile.num_motors
        raw = env.step({agent: motor_cmds})
        sim_time = 0.0
        try:
            while True:
                frame, pwm = bridge.receive_pwm()
                if frame is None: continue
                motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)
                raw = env.step({agent: motor_cmds})
                sim_time += dt
                bridge.send_state(bridge.build_json_state(raw[agent][0], sim_time))
                if hasattr(self, '_state_callback'): self._state_callback(raw)
        except KeyboardInterrupt: pass
        finally: bridge.close()

class ArduBridgeNode(Node):
    def __init__(self):
        super().__init__('ardubridge_node')
        self.declare_parameter('params_file', '')
        file_path = self.get_parameter('params_file').get_parameter_value().string_value
        if not file_path: raise RuntimeError("params_file nao encontrado!")

        self.interface = BiguaSimInterface(file_path, init=False, node=self)
        
        # Patch de ID: Garante que 'auv0_id0' vira base 'auv0' e id 0
        def novo_get_agent_id(agent_name):
            if '_id' in agent_name:
                parts = agent_name.split('_id')
                return parts[0], int(parts[1])
            return agent_name, 0
        self.interface._get_agent_id = novo_get_agent_id

        scenario_cfg = self.interface.scenario
        agent_type = scenario_cfg['agents'][0]['agent_type']
        match = next((k for k in VEHICLE_REGISTRY if k.upper() == agent_type.upper()), None)
        if match is None: raise RuntimeError(f"Veiculo desconhecido: {agent_type}")
        profile = VEHICLE_REGISTRY[match]
        
        scenario = PublishingArduBiguaSimRunner.build_scenario(
            profile, package_name=PACKAGE_NAME, world=WORLD,
            ticks_per_sec=scenario_cfg.get('ticks_per_sec', 720),
            location=scenario_cfg['agents'][0].get('location', [0, 0, 0.3]),
            rotation=scenario_cfg['agents'][0].get('rotation', [0, 0, 0]),
        )

        self.runner = PublishingArduBiguaSimRunner(
            profile, scenario, gps_origin=GPS_ORIGIN,
            show_viewport=scenario_cfg.get('show_viewport', True), verbose=False,
        )

        # Patch de Dinâmica: Mapeia o nome esperado pela interface para 'HolybroX500'
        dyn_dict = self.runner._env._dynamics_dict
        # O nome do agente esperado pela interface é o que está no YAML
        expected_name = self.interface.scenario['agents'][0]['agent_name'].split('_id')[0]
        
        # Mapeia explicitamente o nome esperado para o nome real do modelo
        if expected_name not in dyn_dict:
            dyn_dict[expected_name] = dyn_dict['HolybroX500']
            self.get_logger().info(f"Patch: mapeando '{expected_name}' -> 'HolybroX500'")

        self.interface.env = self.runner._env
        self.interface.initialized = True
        self.interface.sensors = self.interface.create_sensor_list()

        self._sensor_publisher_create()
        self._control_subscribers_create()
        self.runner.set_state_callback(self.interface.publish_sensor_data)
        
        threading.Thread(target=self._run_bridge, daemon=True).start()

    def _run_bridge(self):
        with self.runner: self.runner.run()

    def _sensor_publisher_create(self):
        for s in self.interface.sensors:
            s.publisher = self.create_publisher(s.message_type, f"{s.agent_name}/{s.name}", 10)

    def _control_subscribers_create(self):
        for ag in self.interface.scenario['agents']:
            name = ag['agent_name'].replace('-', '_')
            self.create_subscription(Float64MultiArray, f"{name}/command_control",
                lambda msg, a=name: self.interface.send_control_command(a, list(msg.data)), 10)

def main(args=None):
    rclpy.init(args=args)
    node = ArduBridgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()