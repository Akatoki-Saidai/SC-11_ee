# SC-11_ee

## Arduino Pin使用番号

## 各種センサ動作確認結果

<table>
    <tr>
        <td width="20%" rowspan="3">三軸加速度</td>
        <td>analogRead()で電圧を読んでシリアルモニタに表示することはできた</td>
    </tr>
    <tr>
        <td>ふってみてデータを見た感じ変化はしているのでちゃんと取れてる？</td>
    </tr>
    <tr>
        <td>生データを表示しているのでわかりにくいので，[m/s^2]表記への変換が必要</td>
    </tr>
    <tr>
        <td rowspan="3">3軸角速度</td>
        <td>https://n.mtng.org/ele/arduino/tutorial017.html</td>
    </tr>
    <tr>
        <td>レジスタ等の変更が必要らしい</td>
    </tr>
    <tr>
        <td>単位は [°/sec]</td>
    </tr>
    <tr>
        <td rowspan="4">BME280</td>
        <td>高度がマイナスになるのは海面気圧が問題？</td>
    </tr>
    <tr>
        <td>SC-10で購入したSwitchScience製のBME280で動作確認</td>
    </tr>
    <tr>
        <td>高度は↓のサイトを参照</td>
    </tr>
    <tr>
        <td>https://keisan.casio.jp/exec/system/1203469826</td>
    </tr>
        <tr>
        <td rowspan="3">DRV8835</td>
        <td>モーターを動かす時間がなかったのでLDEでテスト</td>
    </tr>
    <tr>
        <td>「加速」「減速」以外はLEDでは正常に動作</td>
    </tr>
    <tr>
        <td>「加速」は「減速」は確認できなかった（エラーは出てない）</td>
    </tr>
</table>

## 参考文献