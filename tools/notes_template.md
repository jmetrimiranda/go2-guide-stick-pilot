# Notas da sessão `<sessao>`

Copie para `data/<sessao>/notes.md` e preencha. Valores medidos, não estimados.

## Data, local, pessoas
- Data e hora de início/fim:
- Local e piso (4×4 m livres? tipo de piso):
- Participante: Jorge, olhos abertos, mão que segurou o cabo (D/E):
- Segunda pessoa presente (sim/não, quem):

## Bastão
- **Tubo:** fibra de carbono 3K, Ø ext. 27 mm, Ø int. 25 mm, 1 m, massa ___ g (pesar com soquete e parafusos: ___ g)
- **Peças impressas:** PETG, arquivos de `cad/` conforme `cad/README_CAD.md` (versão final de 22/11/2026):
  - `tower_plug.stl`
  - `socket_27mm.stl`
  - `hinge_pin.stl` e `hinge_clip.stl`
- **Teste de rigidez:** tubo preso na mesa, **0,9 m em balanço, 1 kg pendurado na ponta**
  - flecha δ = ___ mm (medida 1) / ___ mm (medida 2) / ___ mm (medida 3)
  - k = 9,81 / δ = ___ N/mm
- **Bastão flexível (se houve):** material, comprimento, δ = ___ mm, k = ___ N/mm

## Montagem (pronta em 22/11/2026)
- **Fixação no robô:** o `tower_plug` entrou na **torre da v21 do RollCall**, no lugar do mastro do sensor.
  - Está preso por **parafusos Phillips nos 4 furos de baixo** da torre, 51,75 mm abaixo do topo.
  - A seta da tampa aponta para a cabeça do robô.
- **Tubo no soquete:** **2 parafusos atravessam soquete e tubo**, no lugar das abraçadeiras. Bitola dos parafusos: ___
- **Articulação:** **pino impresso com trava em C**, no lugar do parafuso M6, com **fita isolante nas faces da lingueta** para tirar a folga lateral (camadas de fita por face: ___).
- **Conferido no dia:**
  - pitch livre? (sim/não)
  - giro travado, sem folga perceptível? (sim/não)
  - trava em C no lugar? (sim/não)
- **Celular no cabo:** modelo ___, distância da mão ___ cm, orientação: tela para cima, topo para o robô.
- **Onde o celular foi preso para as batidas dos `sync`:** ___

## Software e robô
- Computador que comandou o robô: Orin (contêiner `go2-slam:nivelado`) / outro:
- `--iface`:
- Versões: firmware do Go2 (app), imagem Docker, `unitree_sdk2py`, commit deste repo:
- Sensor Logger: versão, plataforma (Android/iOS), sensores ligados (Accelerometer, Gyroscope, **Gravity**), taxa:
- RollCall/Nav2/CMU/app parados antes? (saída do `check`: "foreign requests: none seen"):
- Desvio de obstáculos no app: desligado (sim/não)
- Robô religado antes da sessão? (o RollCall deixa `SpeedLevel(1)`; persistência não confirmada)
- `timedatectl` no início: relógio sincronizado? algum aviso de "clock stepped" durante a sessão?

## Coleta
- `sync --tag start`: batidas visíveis no diagnóstico (contagem / alturas):
- Blocos rodados (comando exato, `--reps`, `--seed`):
- Tentativas abortadas/puladas e por quê:
- `sync --tag end`:
- Bateria no início/fim (%):

## Problemas e observações
-

## Mudanças no script em relação ao kit original

O CLI e o contrato de dados (formato dos CSVs) não mudaram. Os limites de velocidade e o cronograma das tentativas também não. Tudo foi testado offline com SDK falso (`test/`).

**`go2_turn_pilot.py`**
- **StopMove garantido:** é enviado também em SIGTERM/SIGHUP (terminal fechado, queda do ssh, `docker stop`), com até 3 tentativas confirmadas e um aviso alto se não confirmar.
- **Início só deliberado:** o prompt só aceita Enter, `s` ou `q`. Teclas digitadas durante a tentativa são descartadas.
- **Numeração segura:** a numeração nunca reutiliza um número. Tentativas puladas ou interrompidas ganham uma linha anotada, e os CSVs são salvos mesmo quando a tentativa é abortada (mais `events.json`).
- **Ordem balanceada:** em blocos de 4, perfil × direção sai balanceado (4/4/4/4 com `--reps 8`), e a semente aparece impressa.
- **Cegamento:** o perfil não é mostrado durante a sessão.
- **Novos motivos de abortar:** Move que falha (código de retorno), laço de controle travado por mais de 0,25 s, `lowstate` parado.
- **Relógio:** o controle usa relógio monotônico; os logs continuam com `time.time()`. Saltos do relógio de parede são detectados e anotados.
- **Trava de segurança:** antes de andar, o `run` escuta comandos de outros programas por 2 s e exige `continue` se vir algum. O `check` também mostra esses comandos.
- **`sync`:** salva mesmo com Ctrl-C, não sobrescreve um sync anterior e imprime o diagnóstico das batidas.

**`analyze_pilot.py`**
- **Sincronização:** pares de batidas coerentes entre início e fim, com aviso alto quando cai para uma sincronização só. Saltos de relógio anotados pelo script são tratados.
- **Lags:** `robot_lag_s` e `hand_lag_s` passaram a ser lags de centróide, que não dependem do formato do comando. Os onsets de 30% e o `hand_cue_onset_s` ficaram como colunas extras.
- **Trabalho e energia:** a comparação usa o valor acima da caminhada (`mech_excess_J`, `elec_excess_J`), porque a janela do smooth é 0,8 s maior.
- **Curva e jerk:** `robot_turn_deg` agora é com sinal, sem `abs`. O jerk usa uma janela fixa de 4,12 s.
- **Celular:** falta de `Gravity.csv` faz a análise parar. Buracos maiores que 0,1 s anulam as métricas da mão.
- **Novas saídas no resumo:** tabela por direção, lista de tentativas puladas e seção **T6 checks**.
