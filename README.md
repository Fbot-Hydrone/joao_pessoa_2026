# Hydrone ROS2 — CBR Flying Robot League 2026

Stack ROS2 Humble em Python para a competição **Flying Robot League** da RoboCup Brasil.

---

## Arquitetura de nós

```
hydrone_bringup  (launch)
│
├── hydrone_vision        ← ZED2 + ArUco + MediaPipe + pyzbar
├── hydrone_controller    ← MAVROS bridge (ArduPilot / PixHawk)
├── hydrone_nav           ← Planejamento de rota + pouso de precisão
└── hydrone_mission       ← Máquina de estados — orquestra tudo
```

### Tópicos principais

| Tópico | Tipo | Descrição |
|--------|------|-----------|
| `/hydrone/mission_state` | `MissionState` | Estado e pontuação atual |
| `/hydrone/vision/landing_bases` | `LandingBase` | Bases detectadas pela câmera |
| `/hydrone/vision/human_gesture` | `HumanGesture` | Gesto do operador (Fase 3) |
| `/hydrone/vision/qr_codes` | `QRCode` | QR codes lidos (Fase 4) |
| `/hydrone/controller/cmd_pose` | `PoseStamped` | Setpoint de posição |
| `/hydrone/nav/status` | `String` | Estado da navegação |

---

## Pré-requisitos

```bash
# ROS2 Humble
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# Python
pip install mediapipe pyzbar opencv-python numpy
```

---

## Build

```bash
cd ~/hydrone_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Execução por fase

### Fase 1 — Localização e Mapeamento
```bash
ros2 launch hydrone_bringup hydrone.launch.py phase:=1 open_hardware:=true
```

### Fase 2 — Transporte de Pacotes
```bash
ros2 launch hydrone_bringup hydrone.launch.py phase:=2 open_hardware:=true
```

### Fase 3 — Interação Humano-Enxame
```bash
# Um drone
ros2 launch hydrone_bringup hydrone.launch.py phase:=3

# Dois drones (pontuação compartilhada, 3 bases cada)
ros2 launch hydrone_bringup hydrone.launch.py phase:=3 use_two_drones:=true
```

### Fase 4 — Navegação em Espaço Confinado
```bash
ros2 launch hydrone_bringup hydrone.launch.py phase:=4 open_hardware:=true
```

---

## Iniciar missão manualmente (após launch)

```bash
# Inicia a fase via serviço ROS2
ros2 service call /hydrone/mission/start hydrone_msgs/srv/SetPhase \
  "{phase: 1, open_hardware: true, use_two_drones: false}"

# Abortar em emergência
ros2 service call /hydrone/mission/abort std_srvs/srv/Trigger "{}"
```

---

## Pontuação (regras CBR 2026)

| Fase | Ação | Pontos |
|------|------|--------|
| 1 | Visita única a uma base | +20 |
| 1 | Visita repetida | -5 |
| 2 | Kit levantado | +40 |
| 2 | Kit entregue corretamente | +20 |
| 2 | Kit derrubado no lugar errado | -5 |
| 3 | Visita única a uma base | +20 |
| 3 | Visita repetida (1 drone) | -5 |
| 3 | Visita repetida (2 drones) | -10 |
| 4 | Labirinto percorrido | +50 |
| 4 | QR code único detectado | +20 |
| **Todos** | Retorno autônomo | ×2 |
| **Todos** | Open hardware | ×2 |

---

## Estrutura do workspace

```
hydrone_ws/
└── src/
    ├── hydrone_msgs/          ← Mensagens e serviços customizados
    │   ├── msg/LandingBase.msg
    │   ├── msg/MissionState.msg
    │   ├── msg/HumanGesture.msg
    │   ├── msg/QRCode.msg
    │   └── srv/SetPhase.srv
    ├── hydrone_bringup/       ← Launch files
    │   └── launch/hydrone.launch.py
    ├── hydrone_vision/        ← Detecção de bases, gestos, QR
    │   └── hydrone_vision/vision_node.py
    ├── hydrone_controller/    ← Bridge MAVROS
    │   └── hydrone_controller/controller_node.py
    ├── hydrone_nav/           ← Navegação e pouso de precisão
    │   └── hydrone_nav/nav_node.py
    └── hydrone_mission/       ← Máquina de estados (orquestrador)
        └── hydrone_mission/mission_node.py
```

---

## Próximos passos

- [ ] Integrar driver do gripper/garra para Fase 2
- [ ] Calibrar câmera ZED2 e substituir projeção normalizada por projeção real no `vision_node`
- [ ] Ajustar coordenadas dos waypoints do labirinto (`MAZE_*` em `nav_node.py`) com medição real da arena
- [ ] Testar classificador de gestos com o operador real e ajustar limiares em `_classify_gesture()`
- [ ] Adicionar nó `ardupilot` de interface com o SITL para testes em simulação (Gazebo)
