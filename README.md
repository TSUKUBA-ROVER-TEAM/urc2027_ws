# URC2027 workspace

ROS 2 Jazzy / Ubuntu 24.04 向けのロボット制御ワークスペースです。

## セットアップ

ROS 2、colcon、rosdep、vcs、just を用意して実行します。

```bash
source /opt/ros/jazzy/setup.bash
just deps
just build
source install/setup.bash
```

## 起動

```bash
# 実機（足回り・アーム・グリッパー）
just run real

# アーム（RobStride）＋ITR
just run arm_itr

# 射出機構（ODrive）＋ITR（RobStrideはモック）
just run launcher_itr

# 実機＋ODrive射出機構
just run real use_odrive_launcher:=true
```

実機launchはゲームパッド操作も起動します。
`arm_itr` はODriveを無効化し、`launcher_itr` はRobStrideの通信・復帰処理を起動しません。
両モードともITR全体（足回り・ITR接続のアーム軸・グリッパー）を実機で使用します。
CAN接続は `itr_can_interface` と、各モードの `robstride_can_interface` /
`odrive_can_interface` で変更できます。`launcher_itr` は `odrive_node_id` も指定できます。

一部だけを接続して試す場合：

```bash
just run real arm_only:=true     # RobStrideのみ実機、ITRはモック
just run real drive_only:=true   # ITRのみ実機、RobStrideはモック
just run real launcher_only:=true # ODrive＋ゲームパッド操作のみ
just run odrive_test             # ODrive単体、ゲームパッド操作なし
```

`launcher_only` は他の単体試験モードより優先され、ODriveを自動で有効化します。
`arm_only` と `drive_only` は `drive_only` が優先されます。
ODriveは `use_odrive_launcher` で個別に有効化します。

## CAN接続

| 接続先              | 既定インターフェース | 設定引数                                 |
| ------------------- | -------------------- | ---------------------------------------- |
| ITR基板             | `can0`               | `itr_can_interface`                      |
| RobStride           | `can1`               | `robstride_can_interface`                |
| ODrive（node ID 0） | `can1`               | `odrive_can_interface`、`odrive_node_id` |

RobStrideとODriveはClassic CAN・1 Mbpsで同じバスを共有します。
RobStrideは29bit拡張フレーム、ODriveは11bit標準フレームのCANSimpleを使います。
終端抵抗はバス両端の2か所に配置します。

PCはRobomas基板の上位CAN（FDCAN2）へ接続します。
上位通信はCAN FD・BRSなし・1 Mbps対応のファームウェアを使ってください。
足回りのESCはU3（FDCAN1、Classic CAN 1 Mbps）に接続し、
IDは左前・右前・左後・右後の順に1・2・3・4です。基板IDの既定値は3です。

## ゲームパッド操作

添字は `/joy` の `axes` / `buttons` の0始まりです。

| 操作                     | 入力                                                            |
| ------------------------ | --------------------------------------------------------------- |
| 前後・左右・旋回         | `axes[4]`・`axes[3]`・`axes[0]`                                 |
| arm_1 / arm_2 / arm_3    | 対応する `buttons[1]` / `[2]` / `[3]` を1つ押しながら `axes[7]` |
| arm_y                    | `axes[6]`                                                       |
| arm_4 正方向・負方向     | `axes[2]`・`axes[5]`                                            |
| グリッパー正方向・負方向 | `buttons[4]`・`buttons[5]`                                      |
| ODrive射出               | `buttons[8]`                                                    |

ODriveはノード起動後の最初の有効な押下で **0 radへ移動** し、
到達後の次の押下から **10回転（20π rad）** ずつ進めます。
0への移動は、エンコーダのゼロ設定とは異なります。

起動・Joy通信復帰後は、ボタンを一度離してから押してください。
押しっぱなしでは繰り返さず、移動中の追加入力は破棄します。
再操作は到達後に0.1秒以上離してから行い、間隔は最短1秒です。
新しい位置情報と指令の購読先がない場合は指令を送りません。
実機のODriveはactivate時にインデックス探索を行います。

## 電源断・原点設定

RobStrideは通信復帰後、現在位置へ同期してコントローラを自動復帰します。
初期化・停止・原点設定中は保持が解除されるため、アームを外部で支持してください。

ODriveの電源断は上流へ通知されず、コントローラの自動リセットには未対応です。
電源を戻した後にlaunchを終了・再起動してください。

RobStrideの原点設定は、基準姿勢でアーム全体を支持してから実行します。
対象はarm_1〜3です。失敗時は停止を維持し、原因を確認してください。

```bash
ros2 service call /arm_calibration/arm_1_joint/set_zero std_srvs/srv/Trigger '{}'
# 必要な軸の設定成功後に操作を再開
ros2 service call /arm_calibration/resume std_srvs/srv/Trigger '{}'
```

`/arm_position_controller/recovery_ready` が `true` に戻り、保持を確認してから
支持を外してください。原点の電源断後の保持は実機で確認してください。

## 状態確認・開発

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states
just test
just format
```

CAN ID・減速比・制御上限は `urc2027_description/urdf/`、
コントローラ設定は `urc2027_bringup/config/`、外部依存は `build_depends.repos` にあります。
実機アームへの外部位置指令は `/arm_position_controller/position_targets` を使い、
Recovery Managerを介して送ってください。

`just run launcher_itr`（および `drive_only:=true`）では、arm_4専用の
`arm_4_position_controller` も起動します。arm_4は従来どおりトリガー
（axes[2] / axes[5]）で操作でき、最初の関節位置を受け取ってから指令を送ります。
arm_1〜3のRobStrideとRecovery Managerは起動しません。
