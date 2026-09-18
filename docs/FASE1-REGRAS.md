# Fase 1 — o que as regras exigem

Extraído de `docs/REGRAS-CBR-2026.pdf` (RoboCup Brasil, Flying Robot League,
CBR 2026), seção **FASE 1 - LOCALIZAÇÃO E MAPEAMENTO**. Este arquivo existe
porque a missão foi afinada durante semanas contra a métrica errada — "quantos
pousos válidos" — e as regras pontuam outra coisa.

## A tarefa

> O drone deve sair da base de decolagem e percorrer a arena enquanto detecta
> as **6 (seis) bases de pouso**. O drone deve detectar cada base existente e
> pousar **1 (uma) vez em cada uma** das bases detectadas. Depois disso, o
> drone deve **retornar à base de decolagem e pousar**.

São **sempre seis**: 2 suspensas + 4 móveis no solo. A base de decolagem não é
uma delas. Por isso `target_bases` é 6.

## A pontuação, que é o que importa

| | |
|---|---|
| `+20` | por base visitada **pela primeira vez** |
| `-5` | por **pouso repetido** numa base já visitada |
| `x2` | por **retornar autonomamente** à base de decolagem e pousar nela (só se o saldo for positivo) |
| `x2` | por drone **open-hardware** |
| **480** | máximo: `6 x 20 x 2 x 2` |

E a linha que muda tudo:

> **Obs.: Caso o drone pouse fora da base, a tentativa será encerrada.**

## O que conta como "pousar" — a definição que faltava aqui

O PDF define o verbo, e a definição tem uma exigência que este arquivo não
registrava:

> **DEFINIÇÃO:** Entende-se por pousar quando o drone toca todas as partes do
> trem de pouso na base, de forma que seja visível que o mesmo se apoia na base
> para se manter em uma posição estável e **com hélices desligadas**.

> **DEFINIÇÃO:** Visitar uma base é o ato do drone identificar (por visão) **e
> pousar** em uma base de pouso específica.

Encadeando as duas: sem hélice parada não há pouso, sem pouso não há visita,
sem visita não há `+20`. Um toque-e-arranca não vale nada.

**Isso estava quebrado, e em silêncio.** Nas 262 aterrissagens de
`logs/param_sweep/` e `logs/seed_sweep/`, ZERO chegaram a `armed=False` — todas
saíram por "descended and stopped" e rearmavam 4 s depois, abaixo do
`DISARM_DELAY` de 10 s (`mav.parm:281`). O log dizia seis bases; a súmula diria
nenhuma. O estado `DISARM` existe por causa desta seção: nada é contado como
pouso antes de `/mavros/state` reportar `armed=False`.

## O que isso implica para a missão

**Pousar fora não custa uma base — custa a corrida.** E junto vai o `x2` da
volta, que sozinho vale tanto quanto as seis bases. Uma corrida cautelosa que
visita quatro bases e volta para casa marca `4*20*2*2 = 320`. Uma corrida
gulosa que visita cinco e põe uma perna para fora marca no máximo `200`, sem
volta — e `0` se ainda não tinha pontuado.

Consequências diretas, cada uma medida sobre as 48 corridas em
`logs/param_sweep/`:

1. **O veto de centralização se paga.** Com ele desligado, 3 de 6 corridas
   pousaram fora; com 60 cm, 1 de 6. Média de 233 pontos contra 160. A 60 cm
   ele quase nunca dispara — só barra o caso absurdo.

2. **Voltar para casa é metade do placar.** `mission_budget_s` e
   `return_reserve_s` existem por isso: a decisão de voltar tem que sair cedo
   o bastante para o voo de volta caber nos 10 minutos.

3. **Não pousar duas vezes na mesma base.** `-5` cada. O mapa já marca
   visitadas; é o que impede a missão de reciclar a mesma base.

4. **Ir atrás da sexta base pode não valer a pena.** Uma base a mais são
   `+20`, e depois do `x2x2` são 80 pontos. Mas perder a volta são 240. Se o
   orçamento está apertado, voltar ganha.

## Tempo

- 30 minutos para até **3 tentativas**; vale a **melhor**.
- **Cada tentativa pode levar até 10 minutos** — é o `mission_budget_s`.
- A tentativa termina quando o drone pousa automaticamente (fora da base ou na
  base de decolagem), quando o piloto retoma o controle, ou quando o capitão
  declara encerrada.

## Arena e bases

- Arena de **8 x 8 m** (64 m²), rede de proteção em volta.
- Bases móveis: **1 x 1 m**, linha amarela de 5 cm — a borda está a **50 cm**
  do centro, que é o número contra o qual `land_centre_max_cm` é lido.
- Base de decolagem: 2 x 1,5 m.
- Bases podem estar em **qualquer altura de 0 a 1,5 m**, em qualquer lugar da
  arena, inclusive no topo do ambiente confinado da Fase 4, desde que a 0,5 m
  das paredes.
- Posições só são conhecidas na hora. **Medir a arena é proibido.**
- Piso liso e sem características, com possível reflexão da tinta. Luz externa
  variável.

## Restrições que afetam o código

- **Sem GPS/RTK** e sem nada externo à arena para localizar.
- Totalmente **autônomo** durante a tentativa.
- Depois da primeira tentativa a equipe **não pode mais abrir o código nem
  mudar parâmetros**. A única coisa que muda a estratégia é uma **flag** já
  prevista na linha de comando — o que torna `land_centre_max_cm`,
  `mission_budget_s` e `target_bases` serem argumentos de launch uma exigência
  das regras, não conveniência.
