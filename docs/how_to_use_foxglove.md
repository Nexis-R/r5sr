# foxglove操縦画面の開き方
#### 1. Foxglove Studioを開く
![foxglove_logo](../images/foxglove/foxglove_logo.png)

#### 2. 接続を開く
![fg1](../images/foxglove/fg1.png)

#### 3. Foxglove WebSocketを選択する。
WebSocket URLはデフォルトの```ws://localhost:8765```のままでOK。
その後```Open```を押すと接続される。
![fg2](../images/foxglove/fg2.png)

# foxgloveのレイアウト変更方法
#### 1. LAYOUTボタンを押す
![fg3](../images/foxglove/fg3.png)
#### 2. 任意のレイアウトを開く
r5srの場合は```teleopmain```・```teleopsub-robocup```・```rms_r5sr```が現在登録されています。
![fg7](../images/foxglove/fg7.png)

# foxgloveのレイアウト追加方法
#### 1. LAYOUTボタンを押す
![fg3](../images/foxglove/fg3.png)
#### 2. Import from fileを押す
![fg4](../images/foxglove/fg4.png)
#### 3. r5srのパッケージ内にあるレイアウトを追加する
```~/r5sr/scripts/foxglove_layout```にあるレイアウトを追加する。
最後に右上のOPENを押すと追加される。
![fg5](../images/foxglove/fg5.png)
#### 4. PERSONALの下にレイアウトが追加されているか確認する
![fg6](../images/foxglove/fg6.png)

# 画面の増やし方
左側のアイコンを右クリックして```New Window```を押す。これをすることによってFoxgloveの画面を増やすことができる。
![fg8](../images/foxglove/fg8.png)
