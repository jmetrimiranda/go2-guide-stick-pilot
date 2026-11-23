# Notas da sessão `s1`

Sessão de 23/11/2026. Os campos marcados **[JORGE]** ainda precisam de medição ou informação do Jorge.

## Data, local, pessoas
- **Data:** 23/11/2026.
  - `sync_start`: 18:40:38. Tentativas 1–16: 18:46:01 → 18:57:37. `sync_end`: 19:00:42 (horário de Brasília, relógio do Orin).
- **Local:** casa do Jorge, área livre em ambiente interno. Piso: **[JORGE]**.
- **Participante:** Jorge, sozinho, olhos abertos, andando atrás do robô e segurando o fim do tubo. Mão: **[JORGE: D/E]**.
- **Segunda pessoa:** não.
- **Vídeo:** câmera fixa filmando do `sync_start` até o fim. Arquivo: **[JORGE]**.

## Bastão
- **Tubo:** fibra de carbono 3K, Ø ext. 27 mm, Ø int. 25 mm, 1 m, 115 g. Massa com soquete e parafusos: **[JORGE] g**.
- **Peças impressas:** PETG, conforme `cad/README_CAD.md` (versão final de 22/11/2026): `tower_plug.stl`, `socket_27mm.stl`, `hinge_pin.stl` + `hinge_clip.stl`.
- **Teste de rigidez [JORGE]:** tubo preso na mesa, 0,9 m em balanço, 1 kg pendurado na ponta.
  - flecha δ = ___ / ___ / ___ mm
  - k = 9,81 / δ = ___ N/mm
- **Bastão flexível:** não usado. Só o rígido, `--reps 8` (16 tentativas).

## Montagem
- **Torre:** o `tower_plug` está na torre da v21 do RollCall, no lugar do mastro do sensor. Parafusos Phillips nos 4 furos de baixo, a 51,75 mm do topo. A seta aponta para a cabeça.
- **Tubo no soquete:** 2 parafusos atravessando soquete e tubo. Bitola: **[JORGE]**.
- **Articulação:** pino impresso com trava em C, fita isolante nas faces da lingueta (camadas: **[JORGE]**).
- **Celular no cabo:**
  - iPhone 14 preso com fita no tubo, perto da mão, tela para cima, topo apontando para o robô;
  - distância da mão: **[JORGE] cm**.
- **Celular nas batidas de sync:**
  - deitado na placa v21, em cima do robô, **sem fita**, apertado com a mão espalmada;
  - batidas na placa, ao lado dele, com o lado do punho: 3, pausa, 2.

## Software e robô
- **Comando do robô:** Jetson Orin do Go2 (Ubuntu 20.04.5, L4T R35.3.1), em contêineres novos `--rm` da imagem `go2-slam:nivelado`, via `tools/pilot_in_container.sh`. `--iface eth0` (192.168.123.18).
- **Versões:**
  - dentro do contêiner: Python 3.10.12, `unitree_sdk2py` 1.0.1 (`/opt/unitree_sdk2_python` @814556d), cyclonedds 0.10.2;
  - repositório: commit 52f948a + mudanças listadas abaixo;
  - firmware do Go2: **[JORGE: ver no app]**.
- **Sensor Logger:** 1.69, iOS 27.2.1, iPhone 14. Accelerometer, Gyroscope e Gravity (e as versões não calibradas) a 100 Hz. Medido: 101 Hz, maior buraco de 10 ms.
- **Outros programas:**
  - RollCall, Nav2, CMU e app parados; nenhum processo que comanda o robô;
  - `tune_ui` e `termica` continuaram rodando, por decisão do Jorge, e o tune_ui não foi aberto;
  - só apareceu `rt/api/obstacles_avoid/request` api 1002 (SwitchGet, consulta de estado) a ~12 Hz. Isso continua com o app fechado, então deve vir do próprio robô. O Jorge digitou `continue`.
- **Desvio de obstáculos:**
  - estava ligado (`enable: true`);
  - desligado pelo SDK (`tools/obstacles_avoid.py off`) antes do T5, e confirmado desligado antes do T6;
  - **religado** (`on`, conferido `enable: true`) depois da sessão.
- **Relógio:** Orin com NTP sincronizado e nenhum salto durante a sessão. O relógio do iPhone está ~13,9 s à frente do Orin.
- **Taxas medidas no T3:** `sportmodestate` ~300 Hz, `lowstate` ~500 Hz (`t3_check.txt`).
- **Potência medida no T3:**
  - `power_v·power_a` ≈ 35 W deitado e 67 W em pé;
  - o BMS mostra sempre ~20 W a mais, provavelmente cargas fora do sensor de corrente. Esse deslocamento constante se cancela nas medidas "acima da caminhada".

## Coleta
- **`sync --tag start`:** 5 batidas no robô (0,9–2,0 m/s²), 5 casadas com o celular, offset 13,923 s, nenhum pico sobrando.
- **Bloco:** `run --stick rigid --reps 8 --seed 7`, com 16 tentativas: 8 step, 8 smooth, 4 por perfil × direção.
- **Abortadas ou puladas:** nenhuma. O Jorge não usou o controle durante as tentativas.
- **`sync --tag end`:** 5 batidas (1,1–1,5 m/s²), 5 casadas, offset 13,992 s. Deriva de 69,6 ms em 19,7 min.
- **Análise:** `python3 analyze_pilot.py data/s1 --robot-knock-floor 0.5` (ver "Batidas fracas no robô").
- **Bateria:**
  - 37 % no `sync_start` (18:41), 30 % na tentativa 14 (18:56), 26 % no fim (19:04);
  - passou da regra de parar em 30 % por decisão do Jorge, para completar as 16.
  - Log em `battery_log.txt`.
- **Notas subjetivas:**
  - clarity mediana 7 (faixa 5–7, uma em branco);
  - comfort mediana 6 (faixa 5–7).

## Problemas e observações
- **Batidas fracas no robô:**
  - com os nós dos dedos chegam a ~0,3 m/s² na IMU do robô em pé, e não servem;
  - com o lado do punho chegam a 0,5–2 m/s²;
  - o limite padrão da análise (1,5) perde a maioria, por isso a análise roda com `--robot-knock-floor 0.5`;
  - o ruído com o robô em pé e parado fica em ~0,2 m/s²;
  - no T4 de teste, 8 batidas casaram com resíduo de ~21 ms (desvio padrão).
- **O robô vira menos para a direita:**
  - curvas L: `robot_turn_deg` 87–93°;
  - curvas R: 63–70°, todas abaixo de 80°, por isso a conferência do T6 dá WARN;
  - na curva R o robô fica em ~0,5 rad/s com 0,6 comandado; na curva L acompanha (0,57–0,65);
  - andando reto, desvia ~2–3°/s para a esquerda;
  - isso aparece igual nos dois perfis (diferença step − smooth no ângulo ≈ 0 em cada direção), então é do robô em malha aberta, não do formato do comando;
  - limites e comandos não foram alterados.
- **Tentativa 1, `robot_lag_s` = −0,071 s (o único negativo): artefato do atraso por centroide, não uma resposta adiantada do robô.** Verificado em 23/11/2026, depois de ver os dados; **a tentativa não foi excluída**.
  - **O início da curva é normal:** `robot_onset_lag_s` (30 % do pico) = 0,01 s e `robot_t50_s` (post-hoc) = 0,028 s, dentro da faixa das outras tentativas `step` (0,00–0,02 s e 0,007–0,036 s).
  - **O que muda é a forma do sinal nas bordas da janela** do centroide [início − 0,5 s, fim + 1,5 s]:
    - é a única tentativa em que o robô gira um pouco **ao contrário depois de o comando acabar**: área de −0,033 rad em [fim, fim + 1,5 s], contra +0,02 a +0,11 rad nas outras 15;
    - tem também a maior rotação no sentido da curva antes de ela começar: +0,022 rad em [−0,5 s, 0];
    - as duas coisas deslocam o centroide do robô para antes do centroide do comando.
  - **O valor depende da janela:** −0,013 s (fim + 1,0 s), −0,048 s (início em 0), −0,071 s (padrão), −0,127 s (fim + 2,0 s). Nas outras 15 tentativas o atraso fica positivo em todas essas janelas.
  - Ela foi a primeira tentativa do bloco, logo depois do `BalanceStand`.
  - O centroide supõe uma resposta linear, e essa contra-rotação no fim a quebra. Para o atraso robô–comando nesta tentativa, o início a 30 % e o t50 são mais confiáveis.
- **T5 (`data/teste`, sem bastão):**
  - nas tentativas 006 e 008 o Jorge usou o controle por falta de espaço;
  - depois disso foram usadas duas marcas de partida no chão, partindo do lado contrário ao da curva;
  - o teste de Ctrl-C com o robô andando não foi feito e não será feito (ver "Limitações").
- **Gravação do iPhone parou cedo no primeiro T4** (17:13:15–17:13:41). Na sessão, o Bloqueio Automático ficou em "Nunca" e a gravação cobriu tudo.
- **Mensagem `[Reader] take sample error`** do SDK apareceu uma vez no início de um `run` do T5, sem efeito.

## Limitações
- **Ctrl-C com o robô andando não foi testado no robô real, e o teste não será feito** (decisão do Jorge, 23/11/2026).
  - O que foi verificado no robô:
    - o StopMove confirmado (código 0) no fim de todas as tentativas do T5 e do T6;
    - um Ctrl-C na contagem regressiva (T5, tentativa 002, antes de andar), que gravou a linha `aborted: interrupted`;
    - dois Ctrl-C nos prompts de nota (T5, tentativas 003 e 004), que gravaram as linhas certas.
  - O caminho do Ctrl-C durante o movimento (KeyboardInterrupt → `safe_stop`) só foi testado offline, com o SDK falso (`test/test_pilot.py`).
- SIGKILL ou queda de energia não podem ser tratados pelo script. O que o Go2 faz quando os comandos Move param de chegar não foi confirmado em fonte oficial.
- **Piloto de engenharia, não estudo com usuários:** n = 1 (o próprio autor, vidente, olhos abertos), uma sessão, um bastão (rígido), em casa. As medidas são descritivas, sem estatística inferencial.

## Mudanças no script em relação ao kit original

Ver `tools/notes_template.md`, seção "Mudanças no script", para a lista feita em 22/11. Mudanças de 23/11:
- **`go2_turn_pilot.py`, `read_line()`:** cada pergunta agora sai numa linha própria, e a resposta vai depois de `> `.
  - Motivo: com `DETACH=1` + `docker attach`, o `docker logs` não mostrava a pergunta sem quebra de linha, e o Jorge ficou sem ver o `Type 'continue'`.
  - Comprovado num contêiner no Orin.
  - Teste ajustado em `test/test_pilot.py` (37/37).
- **`tools/probe_state.py`:** leitura passiva de SOC, potência e postura. Só assina tópicos.
- **`tools/obstacles_avoid.py`:** `get`, `off` e `on` do desvio de obstáculos pelo `ObstaclesAvoidClient` oficial do SDK. Não envia movimento.
- CLI, contrato de dados (formato dos CSVs), limites e cronograma **inalterados**.
