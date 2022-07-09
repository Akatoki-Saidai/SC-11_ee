# 概要

一定ずつrpmを増やしたり減らしたりする プログラム：

```arduino
for(int i = 100; i <= 2500; i = i + 50){ 
    volume = i;  //可変抵抗の値を1.0で掛けて変数volumeに入れる．値を調整したい場合は倍率を変更する．
    sprintf(message, "Pulse Width: %d micro sec", volume);  //シリアルモニタに表示するメッセージを作成
    Serial.println(message);  //可変抵抗の値をシリアルモニタに表示
    esc.writeMicroseconds(volume);  // パルス幅 `volume` のPWM信号を送信する
    delay(500);
  }
```

ここで制御を行う．