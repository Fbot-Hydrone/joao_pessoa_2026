import biguasim

config = {
    "package_name": "Competition",
    "world": "CompetionMap",                            
    "main_agent": "uav0",                               
    "frames_per_sec": False,
    "octree_min": 0.02,
    "octree_max": 5.0,                              
    "agents":[                                          
        {                                               
            "agent_name": "uav0",                       
            "agent_type": "HolybroX500",                
            "sensors": [                                
                {
                    "sensor_type": "DynamicsSensor",
                    "socket": "IMUSocket",
                    "configuration": {
                        "UseCOM": True,
                        "UseRPY": False  
                    }
                }     
            ],                    
            "dynamics" : {
                "batch_size" : 1,
            },                                        
            "control_abstraction": 'accel',                    
            "location" : [0, 0, 1],   
            "rotation": [0.0, 0.0, 0]               
        }
    ],
    "window_width":  1280,
    "window_height": 720
}

# 1. Caminho ajustado: Limpei o prefixo longo e adicionei o _C no final
caminho_blueprint = "Blueprint'/Game/Maps/arena__2_/BP_base.BP_base_C'"

posicoes_das_bases = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    [1.0, 1.0, 0.0]
]

with biguasim.make(scenario_cfg=config, verbose=True, show_viewport=True) as env:

    for coord in posicoes_das_bases:
        env.send_world_command(
            "CustomCommand", 
            string_params=["SpawnMesh", caminho_blueprint],
            num_params=coord
        )

    # Roda um tick extra só para garantir que a mesh spawna antes do drone começar a se mexer
    env.tick()

    while True:
        # Comando vazio para o drone (HolybroX500 com accel geralmente espera 4 valores)
        command = [0, 0, 0, 0] 
        
        state = env.step(command)
