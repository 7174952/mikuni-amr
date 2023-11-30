# mikuni-amr
icartminiシステムから改良：GUI増強、地図生成GUI化、地図・ルート編集ツール改良、自律走行画面改良

#2023/10/27
1.Add origental motor driver

2.Add self running function

3.corrected matrix functions

4.Add gui operating fuction

#2023/11/25
1. fixed gndlib
2. add amr_status: running status GUI
3. make map OK/ show running status NG

#2023/11/29
1.自己位置推定不具合解消した
2.地図上からPARTICLE重み計算不具合解消した
3.自律走行:NG(未検証)
#2023/11/30
1.change _unit,_plane type from QPointF() => struct PixelIndex{row, coloumn}
2.Make map: OK, Show status GUI:OK, 自己位置推定:OK
  自律走行：NG（未検証)
