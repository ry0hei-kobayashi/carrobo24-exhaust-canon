# 起動方法

## マップ生成（Hector SLAM）

### terminal 1 シミュレータ起動

```Bash
roslaunch compe_pkg competition.launch devel:=true percategory:=0
```

### terminal 2 キーボード操作を起動

```Bash
roslaunch tam_hsr_utils bring_up.launch simulator:=true
```

### terminal 3 Hector SLAMの起動

```Bash
rosrun hsr_hector make_map.sh
```

### terminal 4 マップ保存ノードの起動

```Bash
rosrun navigation_start save_map.py _map_name:=<map name>
```

## マップ共有

マップデータを他のPCに共有す場合は以下のディレトリにコピーしてください.

- `~/ros_ws/src/5_skills/pumas-navigation-docker/navigation/navigation_start/maps`
- `~/ros_ws/src/5_skills/pumas-navigation-docker/navigation/navigation_start/maps/prohibition_maps`

## マップ生成（Cartographer）

### terminal 1 シミュレータ起動

```Bash
roslaunch compe_pkg competition.launch devel:=true
```

Gazeboの左下部の▶をクリック

### terminal 2 キーボード操作を起動

```Bash
roslaunch tam_hsr_utils bring_up.launch simulator:=true
```

### terminal 3 Cartographerの起動

```Bash
roslaunch cartographer_toyota_hsr hsr_2d.launch
```

### terminal 4 ナビゲーションシステムの起動

```Bash
roslaunch navigation_start navigation_cartographer.launch
```

### terminal 5 マップの反映（HSRを少し動かした後実行）

```Bash
rosservice call /map_augmenter/get_augmented_map "{}"
```

### terminal 6 マップ保存ノードの起動

```Bash
rosrun navigation_start save_map.py _map_name:=<map name>
```

## 既知環境タスク（コンペティション開発）

### terminal 1 シミュレータ起動

```Bash
roslaunch compe_pkg competition.launch devel:=true
```

コンペティション本番直前に設定するシード値を公開します．
それまでは，適宜変更して開発してください．

Gazeboの左下部の▶をクリック

### terminal 2 キーボード操作を起動

```Bash
roslaunch tam_hsr_utils bring_up.launch simulator:=true
```

### terminal 3 ナビゲーションシステムの起動

```Bash
roslaunch navigation_start navigation.launch map_name:=<map name>
```

### terminal 4 task実行

```Bash
roslaunch compe_pkg task.launch
```

## 未知環境タスク（エクストラ課題のみ使用可）

### terminal 1 シミュレータ起動

```Bash
roslaunch compe_pkg competition.launch devel:=true
```

Gazeboの左下部の▶をクリック

### terminal 2 キーボード操作を起動

```Bash
roslaunch tam_hsr_utils bring_up.launch simulator:=true
```

### terminal 3 Cartographerの起動

```Bash
roslaunch cartographer_toyota_hsr hsr_2d.launch
```

### terminal 4 ナビゲーションシステムの起動

```Bash
roslaunch navigation_start navigation_cartographer.launch
```

### terminal 5 マップの反映（HSRを少し動かした後実行）

```Bash
rosservice call /map_augmenter/get_augmented_map "{}"
```

### terminal 6 task実行

```Bash
roslaunch compe_pkg task.launch
```

## コンペティション（既知環境）

### terminal 1 シミュレータ起動

```Bash
roslaunch compe_pkg competition.launch
```

Gazeboの左下部の▶をクリック

### terminal 2 キーボード操作を起動

```Bash
roslaunch tam_hsr_utils bring_up.launch simulator:=true
```

### terminal 3 ナビゲーションシステムの起動

```Bash
roslaunch navigation_start navigation.launch map_name:=<map name>
```

### terminal 4 タスクの実行

```Bash
roslaunch compe_pkg task.launch
```

`start >>`が表示されたら，Enterでタスク開始 \
(`start >>`が表示されるまで時間がかかります)\
終了時刻になると自動的に落ちます．\
注意）シミュレータの時間は現実時間よりも遅く進んでいます．

## 撮影した画像の保存先

撮影された画像データは`io/images`のディレクトリで保存されます．保存された画像フォルダーの中に`data.csv`と`make_video_py.py`が生成されています．
`0`のフォルダーには`camera0`（TidyUpのTask1部屋全体を撮影）の画像データが保存されます．\
`1`のフォルダーには`camera1`（TidyUpのTask1部屋のテーブル周辺を撮影）の画像データが保存されます．

## 動画への変換

### 動画変換方法

```Bash
roslaunch compe_pkg make_video.launch data_name:=<画像フォルダー名> video_time:=<ビデオの長さ>
```

`video_time`はデフォルトで10.0分に設定されています.

### CSVファイルの追記例

CSVファイル（io/images/"画像フォルダー"/data.csv）の追記例を以下に示します．

|frame|score|hit|drop|object_id|
|:---:|:---:|:---:|:---:|:---:|
|490|0|0|0|51|
|880|20|0|0|0|
|1274|20|1|0|0|
|1490|20|1|0|18|
|1595|20|1|1|0|

### フォントについて

動画作成にコーポレート・ロゴ（ラウンド）ver2（フォントデータVersion 10.0.50）フォントを使用しています．\
https://logotype.jp/font-corpmaru-old-v2.html

上記のフォントは，SIL OPEN FONT LICENSE Version 1.1のライセンスで提供されています．
