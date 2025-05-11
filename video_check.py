import cv2

def main():
    # カメラ初期化（0はデフォルトのカメラ）
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)  # 幅
    cap.set(4, 480)  # 高さ

    if not cap.isOpened():
        print("カメラが開けません")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("フレームを取得できませんでした")
            break

        # フレーム表示
        cv2.imshow("リアルタイム映像", frame)

        # 'q' キーで終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 終了処理
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
