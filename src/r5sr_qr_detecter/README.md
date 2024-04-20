# r5sr_qr_detecter
カメラ映像によるQR認識ノード

### 依存関係
todo: ここに依存関係を書いておく。
```bash
```

### Node
#### qr_detector_node
QRコードメイン認識ノード
/image_rawをsubしてOpenCVを用いて映像加工をし、その後/image_processedとしてPubする。