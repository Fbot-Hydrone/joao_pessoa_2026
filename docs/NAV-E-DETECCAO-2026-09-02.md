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

## 3. O gargalo era a detecção — medido, e não era o que se pensava

**Esta seção foi reescrita em 2026-09-02, à tarde.** A versão anterior dizia que
o problema era `yellow_px == 0` (a ZED vendo só a lateral azul) e que o caminho
mais barato era inclinar a câmera. As duas coisas estavam erradas, e o que
mostrou isso foi medir em vez de ler código.

### Como foi medido

150 quadros da ZED gravados em voo, cada um com RGB, profundidade e pose, e com
as bases **rotuladas pela mesma `sample_bases(seed)` que semeia a arena** — ou
seja, verdade, não anotação. A profundidade descarta as ocluídas comparando o
**Z óptico** esperado com o medido (comparar com a distância radial erra 30% na
borda de um FOV de 90° e reprova quase tudo). Sobram **404 aparições de base
visíveis**, alcance mediano 6,4 m, as seis alturas representadas.

Isso é o que permite dizer "achou N de 404" em vez de "pareceu melhor".

### O que a ZED realmente vê

Ela vê o topo. Nos quadros de cruzeiro o anel e a cruz estão nítidos e grandes.
O `yfrac` medido nos maiores contornos não é zero — 0,052, 0,068, 0,102, 0,132.

A base é uma **caixa**: um topo claro com as marcações, sobre paredes laterais
do mesmo matiz. A banda azul admitia `V >= 50`, que aceita as duas, então topo e
parede voltavam num **único contorno em L** e tudo abaixo de `_evaluate` passava
a medir uma forma que não é um pad.

    MEDIDO, 404 aparições:   topo  V mediana 188      parede  V mediana 61

E a assinatura disso é `solidity`, não `no_yellow`:

| gate | com V >= 50 | com V >= 160 |
|---|---|---|
| solidity | 22,9% | 1,5% |
| detectado | 19,3% | 38,9% |

### Por que os limiares não resolviam sozinhos

Varrer `min_solidity` e `yellow_frac_min` **com o contorno errado** move de
17,4% para 18,9% — nada. Foi isso que disse que o contorno, e não o limiar, era
o problema. As distribuições explicam: `solidity` de base verdadeira tem mediana
0,900 contra 0,844 de não-base — ele **não separa nada** nesta cena. Quem separa
é `yellow_frac` (0,023 contra 0,000).

Depois do footprint certo, os mesmos limiares passam a pagar:

    V >= 160                          157/404 = 38,9%   falsos 15
    + yellow_frac_min 0.02 -> 0.006   166/404 = 41,1%   (6-8 m: 45 -> 54)
    + ring_cov_min    0.55 -> 0.35    171/404 = 42,3%   (4-6 m: 112 -> 113)

**19,3% -> 42,3%, com os falsos positivos indo de 7 para 15.** Nada disso é
algoritmo novo: são três números na frontal do `phase1.launch.py`, dois dos
quais o nó passou a expor.

### O que ficou de fora, e o custo

`V >= 160` é **brilho absoluto**, exatamente o tipo de limiar que este repo já
viu quebrar quando a exposição mudou. Foi escolhido medindo contra duas
alternativas que não têm esse defeito:

| corte | recall | falsos |
|---|---|---|
| absoluto `V >= 160` | 42,3% | 15 |
| relativo ao quadro, `0,75 * p95(V do azul)` | 42,1% | 36 |
| relativo por caixa, `0,85 * p95(V)` | 42,6% | 33 |

Recall igual, **falso positivo dobrado** nas duas adaptativas: num quadro sem
azul claro o corte desce e admite chão. E subir a confiança para compensar
derruba o recall sem derrubar os falsos (42,6% -> 31,2% com falsos 33 -> 24),
o que diz que os falsos delas são estruturalmente parecidos com pad.

**As que ainda se perdem são as bases na sombra:** as 97 aparições que não
chegam a nenhum contorno medem V (p90 local) com mediana **90**. É o limiar
absoluto que as descarta, e é aí que está o próximo ganho — provavelmente
segmentando o plano do topo pela profundidade, que já está no quadro e não
depende de brilho nenhum. Não feito.

### Confirmado em voo

Corrida completa depois da mudança, `--phase1 --ground-truth`, seed 10:

| | antes | depois |
|---|---|---|
| bases achadas | 3 de 6 | **4 de 6** |
| pousos | 3 | **4** |
| `solidity` disparando | 22,9% das aparições | 3 vezes na corrida inteira |

As quatro bases entraram no mapa a 2, 5, 11 e 16 cm da posição verdadeira, e os
quatro pousos bateram a altura da base ao centímetro:

    #1  repousou -0,03 m   base 5, esperado -0,02
    #2  repousou +0,21 m   base 1, esperado +0,21
    #3  repousou +0,30 m   base 0, esperado +0,30
    #4  repousou +0,86 m   base 2, esperado +0,86

As duas que faltam (base 3 e base 4) estão nos cantos distantes, e a base 4 é a
mais baixa da arena (0,20 m) — quase rente ao chão, que é o caso em que o topo
some primeiro.

### O que continua valendo da versão anterior

A confirmação pela barriga tem o problema oposto: na altura de confirmação o pad
transborda o quadro e `ring_cov >= 0.55` reprova com o pad debaixo do drone. E a
projeção do frontal pelo plano do chão erra 1,06 m numa base de 1,29 m vista de
7,7 m, porque o plano está errado para base elevada. **Nenhum dos dois foi
mexido aqui** — esta rodada é só detecção.

Uma tentativa de casar RGB e profundidade por estampa foi feita e **revertida**:
o `dt` entre os dois streams tem piso de 306 ms (offset sistemático, não jitter),
uma tolerância de 250 ms recusou todo quadro, o frontal caiu no plano do chão
sempre e o drone passou a pousar fora da base. Fica aberto se os 306 ms são de
carimbo ou de conteúdo — a resposta decide o conserto.


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
