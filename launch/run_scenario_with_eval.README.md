# run_scenario_with_eval.launch (tudo automático)

Este launch faz, em **um comando**:

1) roda um cenário do `gazebo_scenario_v2_split`
2) abre o `debug_plot_v2_walls.py`
3) grava `.bag`
4) salva métricas + séries + gráficos em uma pasta

## Pré-requisito (scripts no pacote)

Garanta que estes scripts estão no seu pacote ROS1 (ex.: `dynamic_obstacle_avoidance/scripts/`) e executáveis:

- `debug_plot_v2_walls.py`  (abre o plot e desenha as paredes virtuais)
- `experiment_bag_and_metrics_runner.py` (grava bag + métricas + gráficos)
- `gazebo_scenario_A*.py`, `gazebo_scenario_B*.py`, `gazebo_scenario_C*.py` (cenários do zip)
- `gazebo_scenario_v2_lib.py`

E que estão instalados no `CMakeLists.txt` (recomendado):

```cmake
catkin_install_python(PROGRAMS
  scripts/debug_plot_v2_walls.py
  scripts/experiment_bag_and_metrics_runner.py
  scripts/gazebo_scenario_A1_single_static.py
  # ... os demais cenários ...
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## Como rodar

Exemplo: cenário A1

```bash
roslaunch dynamic_obstacle_avoidance run_scenario_with_eval.launch \
  scenario_script:=gazebo_scenario_A1_single_static.py \
  method_tag:=APF_CLASSICO \
  output_dir:=/home/kelvin/mestrado_runs \
  timeout_s:=300
```

Cenário B2 (crossing):

```bash
roslaunch dynamic_obstacle_avoidance run_scenario_with_eval.launch \
  scenario_script:=gazebo_scenario_B2_crossing.py \
  method_tag:=LYU_APF \
  output_dir:=/home/kelvin/mestrado_runs \
  timeout_s:=300
```

## O que sai na pasta

`<output_dir>/<run_name>/`

- `run.bag`
- `summary.json`
- `timeseries.json`
- `timeseries.csv`
- `traj_2d.png/pdf` (inclui `wall_L_*` e `wall_R_*`)
- `dmin_timeseries.png/pdf`
- `bearing_timeseries.png/pdf`
- `yaw_timeseries.png/pdf`
- `v_timeseries.png/pdf`
- `w_timeseries.png/pdf`
- `compute_timeseries.png/pdf`
- `cew_timeseries.png/pdf`
- `cte_timeseries.png/pdf`
- `run_info.json`

## Observação importante (encerrar tudo)

O nó `experiment_bag_and_metrics_runner` está com `required="true"`.
Então quando ele termina (goal/colisão/timeout), o roslaunch encerra o cenário e o debug plot automaticamente.
