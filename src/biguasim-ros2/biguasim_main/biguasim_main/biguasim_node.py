
from biguasim_main.interface import BiguaSimInterface

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray

class BiguaSimNode(Node):
    def __init__(self):
        super().__init__('biguasim_node')
        self.declare_parameter('params_file', '')
        
        file_path = self.get_parameter('params_file').get_parameter_value().string_value

        self.subscribers = dict()

        self.interface = BiguaSimInterface(file_path, node=self)
        self.sensor_publisher_create()
        self.control_subscribers_create()
        
        #TODO: Make sure it doesnt tick to fast
        #Tick Timer
        period = self.interface.get_time_warp_period()
        print("Time Warp Period:", period)
        self.timer = self.create_timer(period, self.tick_callback)
        self.callback_in_progress = False

        # /clock — o TEMPO DO SIMULADOR, publicado a cada tick.
        #
        # POR QUE ISSO EXISTE. O simulador roda em camera lenta: com as tres
        # cameras ligadas o loop sustenta ~70 Hz de parede e cada tick avanca
        # 1/ticks_per_sec de tempo simulado, entao passam ~0,35 s de simulacao
        # por segundo de parede (MEDIDO 2026-09-28: 69,6 Hz com ticks_per_sec
        # 200). Qualquer prazo contado no relogio de parede mede, portanto, uma
        # coisa que o drone nao viveu — e o orcamento de 10 minutos da prova e
        # exatamente um desses prazos. Em 600 s de parede o veiculo voava 209 s.
        #
        # Publicado como topico ADICIONAL de proposito: nenhum carimbo de sensor
        # muda (self.system_time continua True na interface), nenhum no passa a
        # usar use_sim_time, e portanto nenhum timer deste stack muda de ritmo.
        # Quem quiser tempo de simulacao le daqui; quem nao ler continua como
        # estava. No drone real ninguem publica isto, e e assim que a missao
        # sabe que deve voltar ao relogio de parede.
        self.clock_pub = self.create_publisher(
            Clock, '/clock',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))

        self.get_logger().info('Tick Started')

 
    def control_subscribers_create(self):
        """
        Define a subscriber for each agent, based on his name, to receive control commands.
        """
        scenario = self.interface.scenario

        for agent_cfg in scenario['agents']:
            agent_cfg_name = agent_cfg['agent_name'].replace('-', '_')
            topic_base = f"{agent_cfg_name}/command_control"
            _ = self.create_subscription(
                Float64MultiArray,
                topic_base,
                lambda msg, agent_name=agent_cfg_name : self.control_callback(msg, agent_name),
                10
            )

    def sensor_publisher_create(self):
        """
        Define a publisher for each agent sensor, based on his name and sensor name, to receive 
        control commands.
        """
        for sensor in self.interface.sensors:
            sensor.publisher = self.create_publisher(sensor.message_type, f"{sensor.agent_name}/{sensor.name}", 10) 
        
  
    def adjust_timer(self, new_period):
        self.get_logger().info(f'Adjusting timer period to {new_period} seconds')
        self.timer.cancel()
        self.timer = self.create_timer(new_period, self.tick_callback)

    def control_callback(self, msg, agent_name):
        """
        Send a message to the specified agent.
        """
        self.interface.send_control_command(agent_name, list(msg.data))

    def tick_callback(self):
        state = self.interface.tick()
        self.interface.publish_sensor_data(state)
        self.publish_clock(state)

    def publish_clock(self, state):
        """state['t'] e o relogio do simulador, em segundos."""
        t = state.get('t')
        if t is None:
            return
        msg = Clock()
        msg.clock.sec = int(t)
        msg.clock.nanosec = int((t - int(t)) * 1e9)
        self.clock_pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = BiguaSimNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
