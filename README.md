# Mikata Arm用RTコンポーネント

大阪電気通信大学  
澤崎 悠太，升谷 保博  
2019年11月25日

## はじめに

- ROBOTIS社のアクチュエータDynamixel XM430を5個使った関節4軸，グリッパ1軸のロボットアームMikata ArmのためのRTコンポーネントです．
- Dynamixelアクチュエータ用の汎用RTコンポーネント[Dynamixel](https://github.com/MasutaniLab/Dynamixel)と組み合わせて使うことを想定しています．
- ロボットアームへの指令を関節位置または手先位置で与えることができます．同時に動作時間を指定し，現在位置から指令位置に動作する際に，全ての関節が同じ時間で動作が完了するように速度指令を与えています．
- 以下の環境で開発，動作確認しています．
  - Windows 10 64bit版
  - Visual Studio 2015
  - OpenRTM-aist 1.2.0 64bit版

## 処理の概要

- `armJointTarget`ポートに入力があると，位置指令をXM430の内部表現に変換し，指定された時間で動作が完了するように各アクチュエータの速度指令を決定し，`goalPosition`と`movingSpeed`に出力する．
- `armTipTarget`ポートに入力があると，逆運動学を解き，さらに，位置指令をXM430の内部表現に変換し，指定された時間で動作が完了するように各アクチュエータの速度指令を決定し，`goalPosition`と`movingSpeed`に出力する．逆運動学が解けない場合には，`armStatus`ポートの出力値のbit2を1にする．
- `presentPosition`ポートに入力があると，XM430の内部表現を関節角度と手先位置に変換して，`armJoint`と`armTip`ポートへ出力します．
- `moving`ポートに入力があると，アームの状態などを`armStatus`ポートへ出力します．

## 仕様

### 入力ポート
- armTipTarget
  - 型: RTC::TimedDoubleSeq（要素数6）
  - 概要： 手先位置の指令値．[0] x [m], [1] y [m], [2] z [m], [3] ピッチ角 [rad], [4] ハンド開閉（0～1），[5] 動作時間 [s]
- armJointTarget
  - 型: RTC::TimedDoubleSeq（要素数6）
  - 概要： 関節角度の指令値．[0]～[4] 第1～5関節[rad]．[5] 動作時間 [s]
- presentPosition
  - 型: RTC::TimedUShortSeq（要素数5）
  - 概要： Dynamixel RTCから受け取る各アクチュエータの現在角度 [XM430の内部表現]
- moving
  - 型: RTC::TimedUShortSeq（要素数5）
  - 概要： Dynamixel RTCから受け取る各アクチュエータの状態．0: 停止中，1: 動作中，>1: エラー

### 出力ポート
- armTip
  - 型: RTC::TimedDoubleSeq（要素数5）
  - 概要： 手先位置．[0] x [m], [1] y [m], [2] z [m], [3] ピッチ角 [rad], [4] ハンド開閉（0～1）
- armJoint
  - 型: RTC::TimedDoubleSeq（要素数5）
  - 概要： 関節位置．[0]～[4] 第1～5関節[rad]．[5] 動作時間 [s]
- goalPosition
  - 型: RTC::TimedUShortSeq（要素数5）
  - 概要： Dynamixel RTCへ送る各アクチュエータの角度指令 [XM430の内部表現]
- movingSpeed
  - 型: RTC::TimedUShortSeq（要素数5）
  - 概要： Dynamixel RTCへ送る各アクチュエータの速度指令 [XM430の内部表現]
- armStatus
  - 型: RTC::TimedUShort
  - 概要： アームの状態（ビットごとに意味を割り当てている）．
    - bit0: 下位のDynamixelが一つでもmovingならば1
    - bit1: アームの動作中ならば1（動き始めの時間差を考慮）
    - bit2: 上位からの指令が不適切ならば（逆運動学が解けない等）ならば1．
    - bit3: 下位のDynamixelが一つでもエラーならば1

### コンフィギュレーション

なし

## インストール

- [OpenRTM-aist 1.2.0](https://www.openrtm.org/openrtm/ja/download/openrtm-aist-cpp/openrtm-aist-cpp_1_2_0_release)をインストール．
- [GitHubのMikataArmWrapperのリポジトリ](https://github.com/MasutaniLab/MikataArmWrapper)をクローンかダウンロードする．
- CMake
  - ビルドディレクトリはトップ直下の`build`
  - ConfigureはVisual Studio 64bit
- `build\MikataArmWrapper.sln`をVisual Studioで開く．
- ビルド

## 使い方

- Dynamixel RTCを実行．
- トップディレクトリのMikataArmWrapper.batを実行．
- Dynamixel RTCとポートを接続．
  - Dynamixel:presentPosition → MikataArmWrapper:presentPosition
  - Dynamixel:moving → MikataArmWrapper:moving
  - MikataArmWrapper:goalPosition → Dynamixel:goalPosition
  - MikataArmWrapper:movingSpeed → Dynamixel:movingSpeed

## 既知の問題・TODO

- ロボットアーム制御機能共通インタフェースへの対応