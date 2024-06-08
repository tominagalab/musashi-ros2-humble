# basestation  
コーチボックス関係のパッケージを配置しているディレクトリです．

## ディレクトリ構成  
<pre>
basestation
├── README.md
├── player_controller
├── player_server
└── refereebox_client
</pre>

## パッケージリスト  
|Package name|Details|
|---|---|
|player_controller|各プレイヤーへのデータ送信を行うパッケージ|
|player_server|各プレイヤーからのデータ受信を行うパッケージ|  
|refereebox_client| レフェリーボックスと通信を行うパッケージ|  

## RefereeBoxとCoachBox間通信について
RefereeBoxとはTCPで送受信を行います．  
### コマンドフォーマット詳細  
- RefereeBoxからはJSON形式の文字列データがバイナリデータで送られてきます．
- JSONフォーマットは以下の形式を持ちます  
```
{
  command: {コマンド文字列}
  targetTeam: {ターゲットチーム文字列}
}
```
- 上記の文字列の末尾には終端を表す'\0'がつけられて送られてきます．
- {コマンド文字列}には"WELCOME"や"START"などの文字列が入ります．  
- {ターゲットチーム文字列}には"224.16.32.*"で各チームに割り振られているIPアドレスの文字列データが入ります．あるいは __空（""）__ の場合があります． 
  - 空（""）の場合は試合中の両チーム宛に送っていることを意味するコマンドになります．  
  （例）commandが"START"や"DROP_BALL"ではtargetTeamは空です．  
  - ターゲットチーム文字列（"224.16.32.*"）が入っている場合は，そのチームにするコマンドになります．  
  （例）commandが"KICKOFF"で，targetTeamが"224.16.32.44"の場合は，チームHibikino-Musashiがキックオフであることを意味するので，自チームのキックオフポジションに移動し，"START"コマンドを待機する必要がある． 

### コマンド一覧  
|command|targetTeam|  
|-------|----------|  
|"START"|""|  
|"STOP"|""|  
|"DROP_BALL"|""|  
|"HALF_TIME"|""|  
|"END_GAME"|""|  
|"GAME_OVER"|""|  
|"PARK"|""|  
|"FIRST_HALF"|""|  
|"SECOND_HALF"|""|  
|"FIRST_HALF_OVERTIME"|""|
|"SECOND_HALF_OVERTIME"|""|
|"RESET"|""|
|WELCOME|"224.16.32.*"|
|KICKOFF|"224.16.32.*"|
|FREEKICK|"224.16.32.*"|
|GOALKICK|"224.16.32.*"|
|THROWIN|"224.16.32.*"|
|CORNER|"224.16.32.*"|
|PENALTY|"224.16.32.*"|
|GOAL|"224.16.32.*"|
|REPAIR|"224.16.32.*"|
|YELLOW_CARD|"224.16.32.*"|
|DOUBLE_YELLOW|"224.16.32.*"|
|RED_CARD|"224.16.32.*"|
|SUBSTITUTION|"224.16.32.*"|
|IS_ALIVE|"224.16.32.*"|

## CoachBoxとPlayer間の通信について  
CoachBoxとPlayerはUDPで通信を行っています．  
- 具体的な通信処理については"musashi_player/communication/communication.cpp"を参照してください．  
  - UDPの受信処理については"recv"関数で行われています．  
  - UDPの送信処理については"send"関数で行われています．  
### CoachBox→Playerへの通信  
CoachBoxはRefereeBoxから送られてきたコマンドに基づいて，各プレイヤーへコマンドを送信します．  
この時，Hibikino-Musashi内で取り決められたコマンドフォーマットに変換して送る必要があります．  
### Player→CoachBoxへの通信  
各プレイヤーからは各プレイヤーの状態データが格納された文字列データが送られてくる．  
（※バイナリデータになっていないことに注意）  
#### 通信フォーマット　　
カンマ（","）区切りで以下の順に整数文字が入った文字列で送られてくる  

|index|value|detail|
|-----|-----|-----| 
|1|color|チームカラー．CYANなら0，MAGENTAなら1|
|2|id|ロボットのID|
|3|action|ロボットのアクション（Action名前空間の定数値）|
|4|state|ロボットのステート（State名前空間の定数値）|
|5|ball.distance|ボールとの直線距離|
|6|ball.angle|ボールの角度|
|7|goal.distance|ゴールとの直線距離|
|8|goal.angle|ゴールの角度|
|9|myGoal.distance|自身のゴールとの直線距離|
|10|myGoal.angle|自身のゴールの角度|
|11|position.x|自己位置x座標|
|12|position.y|自己位置y座標|
|13|position.angle|姿勢θ|
|14|role|ロボットのロール（Role名前空間の定数値）|
|15|haveBall|ボール保持の有無|
|16|moveto_position.x|ロボットの目標x座標|
|17|moveto_position.y|ロボットの目標y座標|
|18|moveto_position.angle|ロボットの目標姿勢θ|
|19|obstacle.distance|障害物までの直線距離|
|20|obstacle.angle|障害物の角度|

コーチボックス側では，受信後に”,”でsplitして文字列から整数値への変換が必要になる．  
一つの変数が何文字なのかは","でsplitするまでわからない．  
__いずれバイナリデータで送受信する方法に修正し通信速度の高速化を図る必要がある__  

## 実装したい機能  
- キッカー差動ボタン  
- コンパス校正開始ボタン  
- パーティクル再配布ボタン  
- RoleとColorの可視化   