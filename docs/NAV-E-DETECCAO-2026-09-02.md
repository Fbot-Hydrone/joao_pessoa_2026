# A navegação depois da simplificação, e onde a detecção trava

Escrito em 2026-09-02, no fim de uma sessão longa de medição. Substitui a
descrição de navegação do `NAVIGATION.md` naquilo que mudou; o resto daquele
documento (o VIO, a centragem pela barriga, os acoplamentos de parâmetros)
continua valendo.

**Leia a seção 3 antes de mexer na detecção.** É o único gargalo que sobrou, e
o motivo dele não é o algoritmo.

---

## 1. O que a missão faz hoje

```
arma -> decola para takeoff_alt -> voa o U -> pousa no que confirmou
     -> volta para takeoff_alt -> repete -> volta para a base de decolagem
```

**Uma altitude só.** `takeoff_alt` (2,5 m) é a altura de decolagem, de cruzeiro,
de varredura e da hover de confirmação. O drone só sai dela para pousar, e volta
logo depois. Havia cinco parâmetros de altura diferentes (`survey_alt_m`,
`level2_climb_m`, `confirm_alt_m`, `confirm_clearance_m`, `confirm_alt_max_m`) e
nenhum deles comprava nada que a barriga não resolvesse de uma altura fixa.

**A escada tem dois níveis**, ambos o mesmo U:

| nível | forma |
|---|---|
| 1 | U a `takeoff_alt`, três lados, dois giros nos cantos |
| 2 | o mesmo U de novo, sobre uma arena que a primeira passada já mapeou |

Havia quatro. Os dois que saíram estão na seção 4.

**Chegada é posição E rumo, medidas da pose.** Um canto do U são dois setpoints
no mesmo lugar: um que só gira, e depois a perna. Testar chegada só por
distância fazia o setpoint de giro "chegar" no instante em que era emitido — a
distância já era zero — e a perna seguinte saía com o veículo ainda girando.

    MEDIDO 2026-09-02, U completo, 137 amostras em movimento:
        girando E transladando ao mesmo tempo: 36% do voo

O `yaw_tol_deg` existia declarado no nó e **não tinha nenhum leitor**; era do
estado `ROTATE`, que também saiu. Agora a chegada exige os dois, e o log diz
`in place, still turning: N deg to go`.

**O planejador aceita espaço desconhecido.** `plan_allow_unknown` era `False`,
o que torna `unknown` intransponível — e numa arena aberta a maioria dos voxels
nunca foi atingida por um raio. O A* recusava um destino cuja própria célula
nunca havia sido medida:

    MEDIDO 2026-09-02: uma base real, CONFIRMADA a 0.75 de confiança, reportou
    "no way round it exists in the map" três vezes numa arena de 8x8 m VAZIA,
    e foi para a blacklist. Nada estava no caminho.

O comentário dentro do `_goto_via_map` já dizia que esta fase voa com
`allow_unknown=True`; o parâmetro nunca tinha sido ligado. O argumento que fecha
a questão: quando o planejamento falha, o fallback é a linha reta, que cruza
espaço desconhecido **sem perguntar nada** — recusar planejar por causa de
desconhecido não tornava o voo mais seguro, só trocava um caminho que desvia do
que é conhecido por um que o ignora.

**Velocidade.** `WP_SPD` e `WP_ACC` a 0,6 m/s e `ATC_RATE_WPY_MAX` a 15 °/s. Na
prática o sim entrega menos que isso — mediana medida de **0,19 m/s** — porque o
render segura o loop abaixo do tempo real.

## 2. Isto funciona, e está medido

Corrida completa de 2026-09-02, `--ground-truth`, seed 10:

| etapa | resultado |
|---|---|
| altitude | mediana 2,50 m; **0%** do tempo fora de ±0,15 m |
| U | giro completo antes de cada perna |
| planejamento | **nenhuma** recusa na corrida inteira |
| confirmação pela barriga | 3 de 3, seis olhadas cada |
| pouso | erro de **0,01 a 0,12 m**, altura exata nas três |

**Tudo que é detectado vira pouso preciso.** A verificação de altura confirma
cada um: uma base de altura `h` tem o topo em `h + ground_z`, e o veículo repousa
0,12–0,14 m acima disso. Os três pousos bateram; os pousos inválidos de corridas
anteriores erraram 1,30 m e são inconfundíveis por esse critério.

## 3. O gargalo: a ZED não identifica a maioria das bases

**3 de 6 bases detectadas, e o motivo é geométrico, não algorítmico.**

A ZED aponta para a frente e voa a 2,5 m, então o que ela vê de uma base é a
**lateral azul**. O anel e a cruz amarelos — que são a identidade do pad — estão
no **topo**.

O `_evaluate` do `pad_detector` monta a máscara azul, tira os contornos e no
quarto teste exige amarelo dentro do contorno:

```python
if yellow_px == 0:
    return None
```

MEDIDO na corrida, os três maiores contornos de cada quadro:

```
area=0.183  sol=0.97  asp=3.0  yfrac=0.000
area=0.104  sol=0.38  asp=3.3  yfrac=0.000
area=0.014  sol=0.97  asp=3.0  yfrac=0.006
```

Os dois maiores são paredes de base vistas de lado: azuis, grandes, **sem uma
única pixel amarela dentro**. O terceiro é o topo de um pad ao longe, pequeno
demais, e reprova no `yellow_frac_min = 0.02` por pouco — 0,006.

As imagens de depuração mostram isso direto: quadros com **quatro bases nítidas**
e o overlay dizendo `forward 0 pad(s)`.

### A confirmação pela câmera de baixo tem o problema oposto

Ela vê o topo sempre, mas na altura de confirmação o pad **transborda o quadro** e
o anel fica de fora. A varredura polar exige `ring_cov >= 0.55` e reprova, com o
pad debaixo do drone: `down 0 pad(s)`.

### A prova cruzada

Numa corrida do mesmo dia, com a barriga projetando posição pelo rangefinder
(`range_as_depth`), o nível 1 sozinho entregou **5 de 6**. Desligada, entrega 3.
As duas que somem são exatamente as que só o topo revela.

E a precisão das duas rotas, na mesma arena:

| fonte | base | distância | erro |
|---|---|---|---|
| frontal, plano do chão | 1,29 m de altura | 7,7 m | **1,06 m** |
| barriga, rangefinder | 1,29 m de altura | 2,0 m | **0,04 m** |

A projeção da frontal cruza o raio com `z = ground_z`. Para uma base elevada esse
plano está errado, e o erro cresce com a distância porque o ângulo fica raso.

### Caminhos, do mais barato ao mais caro

1. **Inclinar a ZED para baixo** (20–30°). Uma linha no `rotation` do sensor no
   `config.yaml`. Ela passa a ver topos em vez de paredes, o `yfrac` deixa de ser
   zero e nada mais no pipeline muda. Vale igual no drone real, onde é questão de
   como a câmera é parafusada. **Não testado.**
2. **Casar RGB e profundidade por estampa de tempo.** O `_cb_depth` guarda o
   último quadro recebido e o `_cb_image` usa o que estiver lá, sem comparar
   estampas. A `depth_registered` roda a ~2,5 Hz contra ~3,3 Hz da RGB, então
   muitos quadros caem no fallback do plano do chão — que é a rota que erra.
3. **Baixar `yellow_frac_min`**, que reprovou topos distantes por 0,006 contra
   0,02.
4. **Religar a barriga como fonte de posição** (`project_position` +
   `range_as_depth`). Funciona e está medido; foi desligada porque um falso
   positivo dela vira pouso, e essa barreira precisa existir antes.
5. **YOLO.** Para o simulador não paga: o detector clássico acerta quando enxerga.
   Para a **arena real** provavelmente é obrigatório — o `pad_detector.py`
   documenta que lá o amarelo lido pela ZED dá S=44 contra um limiar de 110, e a
   banda *"admits ZERO pixels"*. Cor absoluta não funciona naquela iluminação.

## 4. O que foi removido, e por quê

**Nível 3 (girar no lugar + relevo do octomap + reposicionar para viewpoints).**
Rodou em todas as corridas de 2026-09-01 e 09-02 e produziu **zero candidatas** —
o `relief_candidates` nunca devolveu um ponto. Era minutos de voo que não podiam
contribuir com uma base, e era o que fazia o veículo parecer perdido na arena.

**Nível 4 (corta-grama de 42 pontos).** Achava bases, mas planejava pontos
**fora da arena** (`plan_bounds` é ±5 m, a arena é ±4) e custava várias vezes o
voo do U.

**Vinte parâmetros.** De 65 para 45 no nó da missão. A maioria eram remendos em
volta do bug do planejador da seção 1 — política de retentativa, cooldown,
adiamento, subir e tentar de novo, sobrevoar por cima. Corrigida a causa, nenhum
deles tinha função.

**A lição, porque ela custou horas:** três vezes nesta sessão o caminho rápido
foi construir em volta de um sintoma, e três vezes a resposta apareceu em uma
linha assim que o código foi instrumentado para dizer o que via — as contagens do
`relief_candidates`, o `goal raw=occupied` do planejador, o `yfrac=0.000` do
detector. Meça antes de corrigir.

## 5. Aberto

- **Exposição** — corrigida no `RGBCamera.cpp` da engine (`ManualExposure`,
  `ExposureBias`), que é **outro repositório**. Sem essa build a imagem estoura e
  a detecção não funciona de jeito nenhum. Ver `config.yaml` do biguasim.
- **Relógio de tentativa** — a Fase 1 dá 30 minutos e a missão não tem noção de
  tempo.
- **Verificação de altura no pouso** — os dados existem (`pad.height_measured` e
  a assinatura de 0,12–0,14 m) e nada compara. Uma corrida anterior reportou
  6 de 6 tendo pousado em 5 e uma vez no chão.
- **Descida em malha aberta** — quando o `LAND` do ArduPilot assume, o setpoint
  para e o pad deixa de ser rastreado.

## Relacionados

- [`NAVIGATION.md`](NAVIGATION.md) — a navegação anterior, e as barreiras que
  continuam valendo
- [`PHASE1-MISSION.md`](PHASE1-MISSION.md) — a máquina de estados
- [`LANDING-SITES.md`](LANDING-SITES.md) — o detector e os dois `field_mode`
