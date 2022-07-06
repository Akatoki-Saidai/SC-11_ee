# 概要

一定のrpmを維持するプログラムである．

`loop`関数内部にある以下のコード：

```arduino
esc.writeMicroseconds(1200);
delay(1000);
```

は別に不必要．むしろ`setup`関数内に記述して一度だけ実行しても同じ挙動をする．