# スチュワートプラットフォームロボット

## これは何？
本来は直動アクチュエータを用いて作る[スチュワートプラットフォーム](https://ja.wikipedia.org/wiki/%E3%82%B9%E3%83%81%E3%83%A5%E3%83%AF%E3%83%BC%E3%83%88%E3%83%97%E3%83%A9%E3%83%83%E3%83%88%E3%83%95%E3%82%A9%E3%83%BC%E3%83%A0)をサーボモータで近似して動作するように作ったロボットです。

<img src="https://github.com/hine/servo-stewartplatform/blob/images/sp_image_1.jpg?raw=true" alt="Stewertplatform Robot" width="200px">

## スチュワートプラットフォームの特徴

スチュワートプラットフォームは、アミューズメントパークのライドアトラクションの動く座席などにも用いられている構造です。

一般に想像されるロボットアームのようなシリアルリンク構造に比べ、以下のような特徴があります。

- １点に力が集中しにくく、強い。
- 一つ一つの関節の誤差が蓄積されず、むしろ平準化されるため、精度の低いサーボモータを用いても全体としての精度を出しやすい。
- かっこいい（主観）。

## このキットの特徴

- MDFをレーザーカッターで切り出した部品と、比較的入手しやすい部品のみのキット（***5,000円程度での頒布を予定***）
- [安価なサーボモータ](https://www.vstone.co.jp/robotshop/index.php?main_page=product_info&products_id=3569)（1つ600円程度）を用い、低価格で作ることができる
- [ヴイストン株式会社](https://www.vstone.co.jp/)のArduino互換ロボットコントロール基板「[V-duino](https://www.vstone.co.jp/robotshop/index.php?main_page=product_info&products_id=5039)」を採用。
- [V-Sido CONNECT RC](https://www.asratec.co.jp/products/v-sido-connect/v-sido-connect-rc/)で用いられているV-Sidoプロトコル互換のソフトウェアを実装済み。IK（Inverse Kinematics）指令のみで動かすことが可能

___

## このロボットを用いた作例

### スチュワートプラットフォームMAZE

LeapMotionを用いて手の傾きを検知、プラットフォームの上に載せた物理迷路を手の傾きと同じように動かし、ビー玉をゴールまで届けるゲーム。

Global Game Jamの開場の一つである「[電子デバイスGGJ](https://game-creators.jp/column/11255/)」で制作され、[NT広島](http://wiki.nicotech.jp/nico_tech/index.php?NT%E5%BA%83%E5%B3%B62019)などでも展示されました。

<img src="https://github.com/hine/servo-stewartplatform/blob/images/sp_application_1.jpg?raw=true" alt="Stewrtplatform Maze" width="200px">

[実際に遊んでくれた映像](https://twitter.com/KKKKKKKKKULA/status/1091569659827019782)（※他の方のTwitterへのツイートへのリンクです。）

### IMUを用いて人の手の追従を行う

スチュワートプラットフォームロボットを横向きに用いて、操縦者の手につけたセンサーで手の傾きを検知、プラットフォーム先につけたハンドを同じように動かすデモです。

<img src="https://github.com/hine/servo-stewartplatform/blob/images/sp_application_2.jpg?raw=true" alt="Stewrtplatform Maze" height="200px">

[動作動画](https://youtu.be/wbBaG8WA2ME)

___

## コントロール基板に搭載されているアプリケーションについて

近日公開予定です。

## サンプルコードについて

上記のスチュワートプラットフォームMAZEに用いたUnityでのサンプルコード、並びにPythonでのサンプルコードを公開予定です。

