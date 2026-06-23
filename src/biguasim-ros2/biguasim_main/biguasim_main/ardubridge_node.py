#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

# Importa a biblioteca original do BiguaSim
from biguasim.ardubridge import ArduBiguaSimRunner, VEHICLE_REGISTRY

PACKAGE_NAME = "Competition"
WORLD = "CompetionMap"
GPS_ORIGIN = (33.810313, -118.393867)

class ArduBridgeNode(Node):
    def __init__(self, vehicle_name="BlueROV2"):
        super().__init__('ardubridge_node')
        self.get_logger().info(f"Iniciando ArduBridge para veículo: {vehicle_name}")

        # Puxa o profile do veículo do seu registro (igual ao test.py)
        match = next((k for k in VEHICLE_REGISTRY if k.upper() == vehicle_name.upper()), None)
        if match is None:
            self.get_logger().error(f"Veículo desconhecido: {vehicle_name}")
            sys.exit(1)
            
        self.profile = VEHICLE_REGISTRY[match]
        
        # Monta o cenário
        self.scenario = ArduBiguaSimRunner.build_scenario(
            self.profile,
            package_name=PACKAGE_NAME,
            world=WORLD,
            ticks_per_sec=720,
            location=[0, 0, 0.3],
            rotation=[0.0, 0.0, 0.0],
        )

        # Inicializa o Runner
        self.runner = ArduBiguaSimRunner(
            self.profile,
            self.scenario,
            gps_origin=GPS_ORIGIN,
            show_viewport=True,
            verbose=False,
        )

    def start_bridge(self):
        """Função bloqueante que roda a simulação e a ponte UDP"""
        try:
            self.get_logger().info("Abrindo o BiguaSim e aguardando pacotes do ArduPilot SITL...")
            with self.runner:
                self.runner.run()  # Isso aqui roda o loop infinito da sua classe ArduBiguaSimRunner
        except KeyboardInterrupt:
            self.get_logger().info("Simulação interrompida pelo usuário.")


def main(args=None):
    rclpy.init(args=args)

    # Você pode alterar 'BlueROV2' pro drone que quiser subir (DJIMatrice, etc)
    node = ArduBridgeNode(vehicle_name="HolybroX500")
    
    # Executa a ponte
    node.start_bridge()

    # O código só chega aqui quando o runner.run() for interrompido (Ctrl+C)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()